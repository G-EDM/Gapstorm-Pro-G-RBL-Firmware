//############################################################################################################
/*

 ______     _______ ______  _______
|  ____ ___ |______ |     \ |  |  |
|_____|     |______ |_____/ |  |  |
___  _  _ ____ __ _ ___ ____ _  _
|--' |--| |--| | \|  |  [__] |\/|

This is a beta version for testing purposes.
Only for personal use. Commercial use or redistribution without permission is prohibited. 
Copyright (c) Roland Lautensack    


*/
//############################################################################################################


#include "Grbl.h"
#include <stdlib.h>
#include "ili9341_tft.h"

enum planner_msg {
    PLANNER_OK = 0,
    PLANNER_ERR_UNKNOWN,
    PLANNER_ERR_BROKEN_WIRE,
    PLANNER_ERR_SHORT,
    PLANNER_ERR_SHORT2,
    PLANNER_ERR_MP_TIMEOUT,
    PLANNER_ERR_RETRACT_END,
    PLANNER_WIRE_CONTACT
};

std::map<planner_msg, const char*> planner_messages = {
    { PLANNER_OK,              "" },
    { PLANNER_ERR_UNKNOWN,     "*Unknown error"},
    { PLANNER_ERR_BROKEN_WIRE, "*Check wire"},
    { PLANNER_ERR_SHORT,       "*Short circuit timeout"},
    { PLANNER_ERR_SHORT2,      "*Shorted 2"},
    { PLANNER_ERR_MP_TIMEOUT,  "*Motionplan timeout"},
    { PLANNER_ERR_RETRACT_END, "*Retract limit"},
    { PLANNER_WIRE_CONTACT,    "Wire contact made!" }
};


typedef struct flush_config {
  int     steps_wanted           = 0;
  int     steps_done_back        = 0;
  int     steps_done_forward     = 0;
  bool    pwm_was_disabled       = false;
  int64_t flush_retract_timer_us = 0;
} flush_config;

typedef struct planner_config {
  int  short_circuit_max_duration_us = 0;
  int  line_to_line_confirm_counts   = 0;
  int  early_exit_on_plan            = 0;
  int  max_reverse_depth             = 0;
  uint8_t sinker_axis_mask           = 0; //..
} planner_config;

typedef struct flush_settings {
  bool  spark_disabled  = false;
  int   interval        = 0;
  int   offset_steps    = 0;
  float distance        = 0.0;
} flush_settings;

typedef struct {
    int    arc_counter;
    bool   first_contact_made;
    bool   is_flushing;
    int8_t motion_plan;
    int    total_retraction_steps;
} planner_state;

typedef struct retraction_config {
  int soft_retract_start = 0;
  int hard_retract_start = 0;
  int steps_done         = 0;
  int steps_total        = 0;
  int steps_case_3       = 0;
  int steps_case_5       = 0;
  int steps_case_0       = 0;
  int early_exit_confirmations = 0;
  bool enable_early_exit       = false;
  int steps_per_mm       = 0;
} retraction_config;


planner_msg pause_reason = PLANNER_OK;


planner_state     DRAM_ATTR _state;
retraction_config DRAM_ATTR rconf;

DRAM_ATTR volatile planner_config plconfig;
DRAM_ATTR volatile flush_config   flconf;
DRAM_ATTR volatile flush_settings flushing;

DRAM_ATTR std::atomic<bool> process_paused( false );



int DRAM_ATTR short_circuit_estop_reset_count = 0; // to prevent a single wrong ADC reading from resetting the timer
int DRAM_ATTR no_load_steps                   = 0;


G_EDM_PLANNER planner = G_EDM_PLANNER();

G_EDM_PLANNER::G_EDM_PLANNER(){
    run_simulated = false;
    gconf.gedm_retraction_motion = false;
    position_history_reset();
    reset_planner_state();
}


void G_EDM_PLANNER::pause( bool redraw ){ // this is super chaotic... Need to make a clean solution..
    if( get_is_paused() ) return;
    if( is_machine_state( STATE_PROBING ) ){
        gconf.edm_pause_motion   = false;
        short_circuit_start_time = 0;
        return;
    }

    arcgen.secure_on_off( true, 0 );   // disable pwm
    wire_feeder.secure_on_off( true ); // stop wire

    gconf.gedm_flushing_motion = false; // better not enter a pause while flushing... Hard return..
    enforce_redraw.store( true );
    process_paused.store( true );

    if( pause_reason > PLANNER_OK ){ // print pause reason if due to error
        debuglog( planner_messages[pause_reason] );
        pause_reason = PLANNER_OK;
    }

    while( true ){ // busy loop...
        vTaskDelay(10);
        if( !gconf.edm_pause_motion || get_quit_motion( true ) ) break;
    } // pause ended

    // reset some things
    gconf.edm_pause_motion        = false;
    no_load_steps                 = 0;
    _state.total_retraction_steps = 0;
    set_retraction_steps();
    gsense.reset_sensor_global();
    reset_flush_retract_timer();

    // if forced to quit early return
    if( get_quit_motion( true ) || run_simulated ){ 
        process_paused.store( false );
        return; 
    }

    // used for recovery after a pause, something like a soft restart
    was_paused          = true;
    pause_recover_count = 0;

    if( ! get_is_paused() ){ return; } // Don't remember why. Only set here and in configure().  configure() is called before a job starts. Keep it just in case

    if( settings.get_setting_bool( PARAM_ID_SPINDLE_ENABLE ) ){
        wire_feeder.secure_on_off( false );
    }

    vTaskDelay(10);
    process_paused.store( false );
    arcgen.secure_on_off( false, 4 );
    reset_flush_retract_timer();
    short_circuit_start_time = 0;
    gsense.reset_sensor_global();
    get_motion_plan();

}





bool IRAM_ATTR G_EDM_PLANNER::short_circuit_estop(){
    // check for max retraction distance
    if( is_system_op_mode( OP_MODE_EDM_WIRE ) && _state.total_retraction_steps >= rconf.steps_case_0 ){
        pause_reason = PLANNER_ERR_RETRACT_END;
        gconf.edm_pause_motion = true;
    }
    // check for max duration
    if( short_circuit_start_time != 0 ){
        if ( esp_timer_get_time() - short_circuit_start_time > plconfig.short_circuit_max_duration_us ){ 
            pause_reason           = PLANNER_ERR_SHORT;
            gconf.edm_pause_motion = true; 
        }
    }
    return true;
}


void IRAM_ATTR G_EDM_PLANNER::pre_process_history_line_forward( Line_Config &line ){
    reset_rconf();
    get_motion_plan(); 
    if( !_state.first_contact_made ){
        reset_flush_retract_timer();
        _state.total_retraction_steps = 0;
        no_load_steps = 0;
    } 
}



//###############################
//###############################
int IRAM_ATTR G_EDM_PLANNER::get_motion_plan( bool enforce_fresh ){

    if( run_simulated ){
        _state.motion_plan = MOTION_PLAN_FORWARD;
        return 1;
    }

    _state.motion_plan = get_calculated_motion_plan( _state.first_contact_made ? enforce_fresh : true );
    
    //################################################################
    // Check for negative plan that indicates the seek phase is done
    //################################################################
    if( _state.motion_plan < MOTION_PLAN_ZERO ){
        _state.motion_plan = (_state.motion_plan ^ (_state.motion_plan >> 7)) - (_state.motion_plan >> 7); //_state.motion_plan *= -1; // invert it back
        _state.first_contact_made = true;
    }

    //####################################
    // If motion queue timed out retry
    // a timeout will return 9 as plan
    //####################################
    if( _state.motion_plan == MOTION_PLAN_TIMEOUT && !gconf.edm_pause_motion ){
        int tmp_counter = 0;
        while( _state.motion_plan == MOTION_PLAN_TIMEOUT ){
            if( get_quit_motion( true ) ) break;
            arcgen.secure_on_off( true, 100 );
            arcgen.secure_on_off( false, 2 );
             _state.motion_plan = get_calculated_motion_plan( _state.first_contact_made ? false : true );
             if( ++tmp_counter > 5 || gconf.edm_pause_motion ){
                pause_reason = PLANNER_ERR_MP_TIMEOUT;
                //debuglog( planner_messages[PLANNER_ERR_MP_TIMEOUT] );
                gconf.edm_pause_motion = true;
                _state.motion_plan     = MOTION_PLAN_SOFT_SHORT;
                break;
            }
        }

    }
    


    //####################################################################################
    //
    // Monitor no load feeds to stop on wire breaks. The sensor class will add a
    // soft hold after every forward plan to gain some time to adjust to new feedback.
    // soft hold and feed both happen on a broken wire
    //
    //####################################################################################
    if( _state.total_retraction_steps > 0 || _state.motion_plan > MOTION_PLAN_HOLD_SOFT ){
        no_load_steps = 0;
    } else if( no_load_steps > rconf.steps_case_0 ){
        no_load_steps = rconf.steps_case_0;
    }

    if( _state.motion_plan <= MOTION_PLAN_HOLD_SOFT ){

        if( 
            ( no_load_steps >= rconf.steps_case_0 )
            && is_system_op_mode( OP_MODE_EDM_WIRE ) // only in wire mode
            && _state.first_contact_made             // only after it has the first contact
            && _state.total_retraction_steps <= 5    // only if there are no retraction leftovers
            && position_history_is_at_final_index()  // only if it is at the final index
            && !gconf.gedm_retraction_motion         // trigger wire break only in forward motion
        ){
            pause_reason           = PLANNER_ERR_BROKEN_WIRE;
            gconf.edm_pause_motion = true;
            _state.motion_plan     = MOTION_PLAN_SOFT_SHORT;
            no_load_steps          = 0;
        }
        
    }




    // Set or monitor short circuit timeout
    if( !is_machine_state( STATE_PROBING ) ){
        if( _state.motion_plan >= MOTION_PLAN_HARD_SHORT ){ // don't count soft retractions as short circuit..
            if( short_circuit_start_time == 0 ){
                short_circuit_start_time = esp_timer_get_time();
            }
            short_circuit_estop_reset_count = 0;
        } else if ( _state.motion_plan <= MOTION_PLAN_HOLD_HARD ){
            short_circuit_start_time = 0;
        }
    }





    return _state.motion_plan;
}


bool G_EDM_PLANNER::get_is_paused(){
    return process_paused.load();
}




/** 
  * called while stepping and if probing is active; checks the probe state and 
  * inserts a pause until a positive is confirmed 
  * see sensors.cpp for more details
  **/
bool IRAM_ATTR G_EDM_PLANNER::probe_check( Line_Config &line ){
    if( is_machine_state( STATE_PROBING ) ){
        no_load_steps   = 0; // reset wire break for probing
        line.step_delay = process_speeds.PROBING;
        get_motion_plan();
        line.motion_plan_enabled     = false;
        line.enable_position_history = false;
        reset_flush_retract_timer();
        arcgen.secure_on_off( false, 4 );
        if( _state.motion_plan > MOTION_PLAN_FORWARD ){
            line.skip_feed = true;
        } else{
            line.skip_feed = false;
        }
        if( run_simulated || probing.get( PROBE_TOUCHED ) ){
            set_machine_state( STATE_BUSY ); // probe touched; degrade the state to busy
            memcpy(sys_probe_position, sys_position, sizeof(sys_position));
            arcgen.secure_on_off( true, 0 );
            return true;
        }
    }
    return false;
}



void IRAM_ATTR G_EDM_PLANNER::reset_rconf(){
    rconf.soft_retract_start = 0;
    rconf.hard_retract_start = 0;
    rconf.steps_done         = 0;
    rconf.steps_total        = 0;
}

// this one is only called after flushing
bool IRAM_ATTR G_EDM_PLANNER::reset_short_circuit_protection(){
    short_circuit_start_time        = 0;
    short_circuit_estop_reset_count = 0;
    _state.motion_plan              = MOTION_PLAN_FORWARD;
    _state.total_retraction_steps   = 0;
    no_load_steps                   = 0;
    //gsense.reset_sensor_global();
    reset_rconf();
    return true;
}

// resets some stuff, checks for short circuit timeout and pause resumes..
bool IRAM_ATTR G_EDM_PLANNER::pre_process_history_line( Line_Config &line ){
    line.skip_feed         = false;
    line.ignore_feed_limit = false;
    if( pause_recover_count < 10 ){
        ++pause_recover_count;
        short_circuit_start_time = 0;
        gsense.reset_sensor_global();
    }
    short_circuit_estop();
    if( was_paused ){
        gsense.reset_sensor_global();
        short_circuit_start_time = 0;
        was_paused = false;
        if( !gconf.gedm_retraction_motion ){
            reset_rconf();
            rconf.hard_retract_start = 1;
            return false; // force backward motion after resume
        }
    } 
    return true;
}


void G_EDM_PLANNER::position_history_reset(){
    has_reverse                    = 0;
    was_paused                     = false;
    pause_recover_count            = 0;
    _state.total_retraction_steps  = 0;
    position_history_index_current = 1; 
    position_history_index         = 0;
    position_history_is_between    = false;
    planner_is_synced              = false;
    _state.arc_counter             = 0;
    _state.first_contact_made      = false;
    memset(position_history, 0, sizeof(position_history[0]));
    push_break_to_position_history();
    push_current_mpos_to_position_history();
    position_history_force_sync();
    wire_feeder.reset();
}
















void G_EDM_PLANNER::reset_planner_state(){
    _state.arc_counter        = 0;
    _state.first_contact_made = false;
    was_paused                = false;
    pause_recover_count       = 0;
    set_ignore_breakpoints( false );
    short_circuit_start_time = 0;
}



bool G_EDM_PLANNER::position_history_move_forward( bool no_motion, Line_Config &line ){
    // get the previous position object
    if( !position_history_is_at_final_index() ){
        position_history_work_get_next( false ); 
        if( has_reverse > 0 ){ --has_reverse; }
    } else {
        has_reverse = 0;
    }
    if( position_history_is_break( position_history[position_history_index_current] ) ){
        return true;
    }
    bool _success = move_line( position_history[position_history_index_current], line );
    return _success;
}

/** this function exits if a short is canceled and also changes the z axis position **/
bool G_EDM_PLANNER::position_history_move_back(){
    // get the previous position object
    bool _success = true;
    // make sure it is not a break point
    bool previous_is_final    = future_position_history_is_at_final_index( true );
    uint16_t index_w_previous = position_history_work_get_previous( true ); // peek the previous index without changing the work index
    bool is_stop = ( 
        ( previous_is_final )
        || position_history_is_break( position_history[index_w_previous] ) // check if the previous index is a block position
    ) ? true : false;

    position_history_work_get_previous( false );
    if( !is_stop ){
        //position_history_work_get_previous( false );
    } else {
        return false;
    }

    Line_Config line;
    line.step_delay              = process_speeds.RETRACT;
    line.motion_plan_enabled     = true;
    //line.ignore_z_motion         = gconf.gedm_single_axis_drill_task ? false : true;
    line.ignore_feed_limit       = true;
    line.enable_position_history = true;
    ++has_reverse;

    _success = move_line( position_history[position_history_index_current], line );

    return _success;
}

//#############################################################################
// used only for homing to ignore breaks while seeking the limit switches in 
// positive direction
//#############################################################################
void G_EDM_PLANNER::set_ignore_breakpoints( bool ignore_break ){
    position_history_ignore_breakpoints = ignore_break;
}
bool G_EDM_PLANNER::position_history_is_break( int32_t* target ){
    // positive positions are interpreted as invalid
    // only while homing they are allowed
    if( position_history_ignore_breakpoints ){
        return false;
    }
    bool is_history_break = false; 
    for( int i = 0; i < N_AXIS; ++i ){
        if( target[i] > 0 ){
            is_history_break = true;
            break;
        }
    }
    return is_history_break;
}

uint16_t G_EDM_PLANNER::position_history_get_previous( bool peek ){
    uint16_t index = position_history_index;
    if (index == 0) {
        index = POSITION_HISTORY_LENGTH;
    }
    index--;
    if( ! peek ){
        position_history_index = index;
    }
    return index;
}
uint16_t G_EDM_PLANNER::position_history_get_next(){
    position_history_index++;
    if (position_history_index == POSITION_HISTORY_LENGTH) {
        position_history_index = 0;
    }
    return position_history_index;
}
uint16_t G_EDM_PLANNER::position_history_work_get_previous( bool peek ){
    uint16_t index = position_history_index_current;
    if (index == 0) {
        index = POSITION_HISTORY_LENGTH;
    }
    index--;
    if( ! peek ){
        position_history_index_current = index;
    }
    return index;
}
uint16_t G_EDM_PLANNER::position_history_work_get_next( bool peek ){
    uint16_t index = position_history_index_current;
    index++;
    if (index == POSITION_HISTORY_LENGTH) {
        index = 0;
    }
    if( ! peek ){
        position_history_index_current = index;
    }
    return index;
}
void IRAM_ATTR G_EDM_PLANNER::position_history_force_sync(){
    position_history_index_current = position_history_index;
}
/**
  * In reverse mode this checks if the previous work index is allowed to be used
  * if the previous index is the real final index the history run a full cycle backwards
  * 
  * In forward mode it checks if the next work index is the final index
  **/
bool G_EDM_PLANNER::position_history_is_at_final_index(){
    return position_history_index == position_history_index_current ? true : false;
}
bool G_EDM_PLANNER::future_position_history_is_at_final_index( bool reverse ){
    if( reverse ){
        uint16_t previous_w_index = position_history_work_get_previous( true );
        if( previous_w_index == position_history_index ){
            return true;
        } return false;
    }
    uint16_t next_w_index = position_history_work_get_next( true );
    if( next_w_index == position_history_index ){
        return true;
    } return false;
}
uint16_t IRAM_ATTR G_EDM_PLANNER::get_current_work_index(){
    return position_history_index_current;
}

uint16_t IRAM_ATTR G_EDM_PLANNER::push_to_position_history( int32_t* target, bool override, int override_index ){
    // move one step forward
    if( ! override ){ position_history_get_next(); }
    int index = override ? override_index : position_history_index;
    //position_history[ index ] = target;
    memcpy( position_history[ index ], target, sizeof(target[0]) * N_AXIS );
    return index;
}

/** 
  * Takes the target and line config, pushes the line to the history and syncs the history
  * All positions that are passed to this function are pushed to the history object
  * Not all motions use the history object and some are calling the move line function without storing
  * the positions in the history
  **/
bool IRAM_ATTR G_EDM_PLANNER::process_stage( int32_t* target, Line_Config &line ){
    // push the target to the history buffer
    uint16_t index = push_to_position_history( target, false, 0 );
    bool _success  = position_history_sync( line );
    if( ! _success || system_block_motion ){
        // if the position could not be reached 
        // it is necessary to update the history element for this position
        target = sys_position;
        //override_target_with_current( target );
        push_to_position_history( target, true, index );
    }
    return _success;
}

//########################################################################################
// Copies the current machine position into the target array
// Looks like it is only used on failures
//########################################################################################
void G_EDM_PLANNER::override_target_with_current( float* target ){
    float* current_position = system_get_mpos();
    memcpy(target, current_position, sizeof(current_position[0])*N_AXIS );
}

//########################################################################################
// target contains the target positon in mm
// __target will be filled with the targetposition in step position
//########################################################################################
void IRAM_ATTR G_EDM_PLANNER::convert_target_to_steps( float* target, int32_t* __target ){
    for( int axis=0; axis<N_AXIS; ++axis ) {
        __target[axis] = (int32_t)lround( (double)target[axis] * (double)g_axis[axis]->steps_per_mm.get() );
    }
}

//########################################################################################
// Add the current machine position to the history cache
//########################################################################################
void G_EDM_PLANNER::push_current_mpos_to_position_history(){
    int32_t target[N_AXIS]; // work target
    memcpy(target, sys_position, sizeof(sys_position));
    push_to_position_history( target, false, 0 );
}

/** 
  * breaks are ignored in forward direction but prevent the history from moving back 
  * This is just a cheap and easy way to prevent M3/M4 up/downs from becoming a problem
  * Z axis is dispatched in floating operation and would not follow the UP/DOWN path in the history
  * That would result in a crash. The easy way to prevent this is to add a break and 
  * stop the history from retractring further back
  * A break is basically just a position block with all positive coords
  * The only motion that uses positive targets is the homing motion
  * therefore while homing breaks are ignored
  * in normal operation there are only negative targets (all negative space)
  * Make sure your machine is set to all negative space!
  **/
void G_EDM_PLANNER::push_break_to_position_history(){
    int32_t target[N_AXIS];
    for( int axis = 0; axis < N_AXIS; ++axis ){ target[axis] = 1; }
    push_to_position_history( target, false, 0 );
}


bool G_EDM_PLANNER::position_history_sync( Line_Config &line ){
    gconf.gedm_retraction_motion  = false;
    gconf.gedm_planner_sync     = true;
    bool _success               = true;
    bool motion_ready           = false;
    bool current_is_last_block  = false;
    int  direction              = 0; // 0 = forward, 1 = backward
    int  last_direction         = 0; 
    bool enable_history         = line.enable_position_history; // history is only useful for wire/sinker edm    
    _state.total_retraction_steps = 0;

    motion_ready = position_history_is_at_final_index();

    while( !motion_ready ){

        motion_ready = false;

        if( get_quit_motion() ){ 
            _success = false;
            break; 
        }

        if( enable_history ){

            // default direction is forward
            // if history is enabled it can retract backwards
            // Note: in a backward retraction the line backwards is canceled as soon as the short
            // circuit is canceled. It results in a success=false
            // the line was not fully finished and therefore returns false
            // but this is actually a success in cancelling the short circuit
            direction = _success 
                         ? get_success_direction( last_direction ) 
                         : get_failure_direction( last_direction );

        }

        // move in the wanted direction
        _success = process_direction( direction, line );
        last_direction = direction;
        current_is_last_block = position_history_is_at_final_index();

        // break conditions
        if( _success ){
            if( direction == 0 ){
                // this is the same for motion with and without history
                // if the last block was succesfull in forward direction
                // it is finished
                if( current_is_last_block ){
                    motion_ready = true;
                    break;
                }
            }
        } else {
            if( ! enable_history && current_is_last_block ){
                // if it is a non history motion and reached the last block exit and return the success state
                motion_ready = true;
                break;
            }
        }
    }
    gconf.gedm_planner_sync      = 0;
    gconf.gedm_retraction_motion = false;
    gconf.gedm_flushing_motion   = false;
    gsense.add_to_sense_queue( SENSE_UNLOCK ); // just in case
    // to be safe that history is synced
    // even after a hard motion cancel etc.
    position_history_force_sync();
    return _success;
}

/**
  * 
  * Success and failure only refers to the last line
  * If a line was finished it is a success
  * if a line was not finished it is a failure
  * A backward motion to cancel short circuits don't need to run the full line
  * So a failure in a backward motion always indicates that the short was canceled before the line finished
  * 
  **/
int G_EDM_PLANNER::get_success_direction( int last_direction ){
    // last motion executed succesfull
    switch (last_direction){
        case 0:
            // last success motion was in forward direction
            // this can be a normal feed or a recover forward motion 
            // if this is still within a recovery it needs some extra checks
            // to keep it in recovery until at the initial position
            return 0; // keep going forward
            break;
    
        case 1:
            // last success motion was backward
            // this motion was not enough to cancel a short circuit
            // a short was not canceled and needs more retraction
            no_load_steps = 0;
            // no matter what history depth is set
            // if within an arc it is overwritten
            // an arc is seen as a single line here
            if( 
                !is_system_op_mode( OP_MODE_EDM_SINKER ) && // allow unlimited retraction to the breakpoint in sinker mode
                has_reverse >= _state.arc_counter+plconfig.max_reverse_depth 
            ){ //MAX( _state.arc_counter, plconfig.max_reverse_depth ) ){
                return 0; // force forward
            }
            return 1;
            break;
    }
    return 0;
}
int G_EDM_PLANNER::get_failure_direction( int last_direction ){
    // last motion failed
    switch (last_direction){
        case 0:
            // last failed motion was in forward direction
            // this indicates that a normal feed motion 
            // or a forward recover motion created a short
            // a recover motion follows a retraction with the goal to get back to the initial position
            return 1; // no matter the details the next move is backwards/retraction
            break;

        case 1:
            // last failed motion in backward direction
            // this motion canceled a short circuit
            // there are no other options for a failed backward motion except the 
            // successfull cancelation of short circuits
            // the backward line was not fully drawn since the short was gone somewhere within the line
            //delayMicroseconds(200); // #todo #review this delay is very old. Maybe it should be removed?
            no_load_steps = 0;
            return 0; // back to forward???
            break;
    }
    return 1;
}
bool G_EDM_PLANNER::process_direction( int direction, Line_Config &line ){
    bool _success = true;
    switch (direction){
        case 0:
            return position_history_move_forward( false, line );
            break;
        case 1:
            gconf.gedm_retraction_motion = true;
            _success = position_history_move_back();
            gconf.gedm_retraction_motion = false;
            return _success;
            break;
    }
    return false;
}




















void IRAM_ATTR G_EDM_PLANNER::wire_line_end( Line_Config &line ){
    bool deep_check = false;
    int pass_counter = 0;
    int total_counts = round(plconfig.line_to_line_confirm_counts/2);
    if( position_history_is_at_final_index() ){ 
        if( _state.arc_counter == 0 ){
            total_counts = plconfig.line_to_line_confirm_counts;
            deep_check = true;
        }
    }
    // at the final position
    while( true ){
        get_motion_plan( true );
        if( _state.motion_plan <= deep_check ? MOTION_PLAN_FORWARD : MOTION_PLAN_HOLD_SOFT ){ 
            ++pass_counter; 
        } 
        else { pass_counter = 0; }
        if( pass_counter >= total_counts || _state.motion_plan >= MOTION_PLAN_SOFT_SHORT || get_quit_motion() ){ break; }
    }
}



bool IRAM_ATTR G_EDM_PLANNER::process_wire( Line_Config &line ){

    if( !pre_process_history_line( line ) ){
        return false;
    }  
    
    
    if( gconf.gedm_retraction_motion ){

        //########################################################################################
        // Inside a retraction / Moving back in history
        //########################################################################################

        line.step_delay        = process_speeds.WIRE_RETRACT_SOFT;
        line.ignore_feed_limit = false;

        get_motion_plan();
        if( _state.motion_plan > plconfig.early_exit_on_plan ){
            rconf.early_exit_confirmations = 0;
        }

        if( rconf.enable_early_exit && _state.motion_plan <= plconfig.early_exit_on_plan ){ 
            //if( ++rconf.early_exit_confirmations > 1 ){ 
                return false; 
            //}
        }

        if( rconf.soft_retract_start || rconf.hard_retract_start ){
            line.ignore_feed_limit   = true;
            line.step_delay          = _state.motion_plan == MOTION_PLAN_HARD_SHORT ? process_speeds.WIRE_RETRACT_HARD : process_speeds.WIRE_RETRACT_SOFT;
            if( rconf.hard_retract_start && rconf.steps_total >= rconf.steps_case_5 - 60 ){
                // only a few high speed steps before switching to soft speed
                line.step_delay = process_speeds.WIRE_RETRACT_SOFT;
            }
            if( --rconf.steps_total>0 ){ return true; } 
        }

        if( _state.motion_plan < MOTION_PLAN_HOLD_HARD ){
            return false;
        } 
        return true;
        
    } else {

        //########################################################################################
        // Moving forward in history
        //########################################################################################
        
        rconf.early_exit_confirmations = 0;
        
        pre_process_history_line_forward( line );

        switch ( _state.motion_plan ){

            case MOTION_PLAN_HARD_SHORT: // 
                rconf.steps_total        = rconf.steps_case_5;
                rconf.hard_retract_start = 1;
                return false;
            break; // unreachable code

            case MOTION_PLAN_SOFT_SHORT: // 
                rconf.steps_total        = rconf.steps_case_3;
                rconf.soft_retract_start = 1;
                return false;
            break; // unreachable code
        
            case MOTION_PLAN_HOLD_HARD: // 
                line.skip_feed  = true;
                line.step_delay = process_speeds.EDM;
            break;

            case MOTION_PLAN_HOLD_SOFT: // 
                line.skip_feed  = true;
                line.step_delay = process_speeds.EDM;
            break;

            default: // 
                line.skip_feed  = false;
                line.step_delay = process_speeds.EDM;
            break;

        }

        if( ! _state.first_contact_made ){
            no_load_steps          = 0;
            line.ignore_feed_limit = true;
            line.step_delay        = process_speeds.INITIAL;
        } 


    }
    return true;
}


void G_EDM_PLANNER::configure(){
    _state.arc_counter = 0;
    process_paused.store( false );
    reset_flush_retract_timer();
};

//#############################################################
// Set the sinker axis; On XYUV X and Y will run combined
// with U and V. Motion in X will be XU and in Y it is YV
//#############################################################
void G_EDM_PLANNER::set_sinker_axis( int axis ){
    sinker_axis = axis;
}
void IRAM_ATTR G_EDM_PLANNER::set_retraction_steps(){
    // calculate the steps based on the axes with the most steps per mm
    // it is a hacky solution but works ok..
    uint8_t axis = is_system_op_mode( OP_MODE_EDM_WIRE ) ? axes_active.x.index : sinker_axis; //XYUV should all have the same step resolution
    rconf.steps_case_3 = MAX( 2, motor_manager.convert_mm_to_steps( settings.get_setting_float( PARAM_ID_RETRACT_S_MM ),   axis ) );
    rconf.steps_case_5 = MAX( 2, motor_manager.convert_mm_to_steps( settings.get_setting_float( PARAM_ID_RETRACT_S_MM ),   axis ) );
    rconf.steps_case_0 =         motor_manager.convert_mm_to_steps( settings.get_setting_float( PARAM_ID_BROKEN_WIRE_MM ), axis   );
    rconf.steps_per_mm = motor_manager.convert_mm_to_steps( 1.0, axis );
}


int32_t IRAM_ATTR G_EDM_PLANNER::get_retraction_steps( float travel_mm ){
    // XU and YV should all have the same step resolution
    return round( travel_mm * g_axis[sinker_axis]->steps_per_mm.get() );
}



void IRAM_ATTR G_EDM_PLANNER::reset_flush_retract_timer(){
    flconf.flush_retract_timer_us = esp_timer_get_time();
}


//###########################################################
// Interrupts a line and does a flushing retraction on the
// sinker axis in the oposite direction of where the sinker
// line is headed to.
// Note: This retraction is only for single axis sinker jobs
//###########################################################
bool IRAM_ATTR G_EDM_PLANNER::do_flush_if_needed(){
    if( flushing.interval > 0 && ( esp_timer_get_time() - flconf.flush_retract_timer_us >= flushing.interval ) ){
        if( !_state.first_contact_made ){
            reset_flush_retract_timer();
            return false;
        }
        reset_flush_retract_timer();
        return flush_begin();
    } 
    return false;
}



bool G_EDM_PLANNER::flush_end(){
    if( flconf.pwm_was_disabled ){
        arcgen.secure_on_off( false, 0 ); // re-enable PWM
    }
    gconf.gedm_flushing_motion = false; // unflag flushing motion
    reset_short_circuit_protection();
    reset_flush_retract_timer(); // reset the timer
    return true;
}


void G_EDM_PLANNER::flush_reenable_pwm(){
    // this function is called a few steps before the final flush return position is reached
    gsense.add_to_sense_queue( SENSE_UNLOCK ); // unlock the sensor loop
    if( flconf.pwm_was_disabled ){
        arcgen.secure_on_off( false, 0 ); // re-enable PWM
    }
    flconf.pwm_was_disabled = false;
}



bool G_EDM_PLANNER::flush_begin(){
    // lock sense loop
    gconf.gedm_flushing_motion = true;  // flag to inform that a flushing motion is performed
    flconf.steps_done_back     = 0;
    flconf.steps_done_forward  = 0;
    if( flushing.spark_disabled ){
        arcgen.secure_on_off( true, 0 ); // turn spark off while retracting; will be re-enabled for the return motion
        flconf.pwm_was_disabled = true;
    } else {
        flconf.pwm_was_disabled = false;
    }
    flconf.steps_wanted = get_retraction_steps( flushing.distance );
    gsense.add_to_sense_queue( SENSE_LOCK );
    return true;
}





/*const int rampSteps    = 100; // needs to be mm to step...
 int steps        = 1000;
 int totaltimeus  = 1000000;
 int accelTime    = ( totaltimeus * ( double( rampSteps ) / steps ) );
 int constantTime = ( totaltimeus - ( 2 * accelTime ) ); // should always be the default step delay
 double mpi2         = M_PI_2;

    if (rampSteps * 2 > steps) {
        rampSteps = steps / 2;
    }

// accel; accelstep is the current index of the accel ramp
// how to decide if decel or steady? Planner receives line after line from a blocking loop
// while planner runs the line the outer loop does not parse anything and there is no way to set a flag or something like that
// would require some peek into the next line etc. Much work.
const int accelstep = 0; // up to rampstep
double interval = ( ( accelTime / rampSteps ) * ( sin( mpi2 * ( ( double ) accelstep / rampSteps ) ) ) );

const int decelstep = rampSteps - 1; // minus up to rampstep for reverse
double interval = ( ( accelTime / rampSteps ) * ( sin( mpi2 * ( ( double ) accelstep / rampSteps ) ) ) );

double steadyInterval = (constantTime / (steps - 2 * rampSteps));
      */  


/*enum motion_state {
    MOTION_STATE_STEADY = 0,
    MOTION_STATE_ACCEL  = 1,
    MOTION_STATE_DECEL  = 2    
};*/



//########################################################################################
// Runs the line based on target step positions (int32_t)
// todo: convert all the longs to int? Maybe?
//#########################################################################################
bool IRAM_ATTR G_EDM_PLANNER::move_line( int32_t* target, Line_Config &line ){

    static int64_t stepped_at;

    if( system_block_motion ) return false;


    // constants for accel / decel
    // time to accel to final speed



    //static motion_state current_state = MOTION_STATE_STEADY;

    line.direction_bits   = 0;
    line.step_event_count = 0;
    line.step_count       = 0;

    bool _success           = true;
    int  current_step_delay = line.step_delay; // backup
    int  sync_steps         = 0;

    int32_t target_steps[N_AXIS], position_steps[N_AXIS];
    memcpy( position_steps, sys_position, sizeof( sys_position ) );


    for( int axis=0; axis<N_AXIS; ++axis ){
        line.line_math[axis].counter = 0.0;
        target_steps[axis]           = target[axis];
        line.line_math[axis].steps   = labs( target_steps[axis] - position_steps[axis] );
        line.step_event_count        = MAX( line.step_event_count, line.line_math[axis].steps );
        // Bug: the division was performed as integer division resulting in invalid deltas if the values where <1 casting the delta result to float was missing and low negatives where rounded creating sudden direction
        // Thanks to Nikolay for finding this bug
        line.line_math[axis].delta = ( ( float ) ( target_steps[axis] - position_steps[axis] ) ) / g_axis[axis]->steps_per_mm.get();
        if (line.line_math[axis].delta < 0.0) {
            line.direction_bits |= bit(axis);
        } 
    }


    if( line.step_event_count <= 0 ){
        if( is_system_mode_edm() ){
            if( gconf.gedm_retraction_motion ){
                return true;
            } 
            /*if( position_history_is_at_final_index() ){ gconf.edm_process_finished = true; }*/
            return true;
        }
        delayMicroseconds(10);
        return false;
    }


    motor_manager.motors_direction( line.direction_bits ); // set the current direction for all motors



    


    int accel_time_step = 1;//us ; this accel is only to not smoothen out the initial steps. Not more.
    int accel_steps     = line.motion_plan_enabled ? 0 : ( rconf.steps_per_mm >> 3 ); // accel is somehow really bad for edm. Even like 10 steps for accel is messing around; disable for cutting moves
    if( accel_steps > 0 && ( accel_steps * 2 ) >= line.step_event_count ){
        // ensure accel and deaccel...
        accel_steps = MAX( 1, int( line.step_event_count / 2 ) - 4 );
        accel_time_step  = 2; // ratio?... yeah.. well...
    }



    stepped_at = esp_timer_get_time();

    // the final loop to pulse the motors
    while( line.step_count < line.step_event_count ){
        
        // reset defaults
        line.step_delay = current_step_delay; // restore previous step delay
        line.skip_feed = false;
        
        if( gconf.edm_pause_motion ){ // insert pause on request
            pause();                  // enter pause busy loop
        }

        if( // break conditions
            get_quit_motion()   || // estop, motion turned off
            probe_check( line ) || // probe touch
            ( !line.ignore_limit_switch && GRBL_LIMITS::limits_get_state() ) // limits check
         ){ _success = false; break; }



        // Only used for lines with motionplan enabled
        // normal jogmotions etc. will not access this section
        if( line.motion_plan_enabled ){
            if( !gconf.gedm_flushing_motion ){ // normal process motion
                _success = process_wire( line );
                if( ! _success ){ break; }
                if( !gconf.gedm_retraction_motion ){
                    if( line.enable_flushing ){
                        if( do_flush_if_needed() ){
                            // return false to exit forward motion and enter a retraction
                            _success = false;
                            break;
                        }
                    }
                }
                if( !line.ignore_feed_limit && line.step_delay < process_speeds.EDM ){
                    line.step_delay = process_speeds.EDM;
                }
            } else { // flushing motion
                line.step_delay        = process_speeds.RAPID;
                line.ignore_feed_limit = true;
                line.skip_feed         = false;
            }
        }


        // Most precise accelereration known to mankind, also single line stuff.. todo: much
        // todo: keep track across multiple lines
        // todo: use s-curve profile
        // note: line.step_count increments from 0 to whatever max for that line
        if( accel_steps > 0 ){ // poor mans accel enabled for that line

            if( line.step_count <= accel_steps ){ // example: 99 done; 100 accel

                // accel
                line.step_delay = current_step_delay + ( accel_time_step * ( accel_steps - line.step_count ) );

            } else if( ( line.step_event_count - line.step_count ) <= accel_steps ){ // 1000 total - 300 done = 700left

                // decel
                line.step_delay = current_step_delay + ( accel_time_step * ( accel_steps - ( line.step_event_count-line.step_count ) ) );

            } else { line.step_delay = current_step_delay; } // steady

        }








        if( line.skip_feed || esp_timer_get_time() < stepped_at+line.step_delay ){

            //##################################################################
            // Keep waiting if step delay is not reached or feed is skipped
            //##################################################################
            continue;

        } else {

            //#####################################################################
            // At this point we know that a step will get fired
            // and can do some stuff like internal counting of retraction steps etc.
            //#####################################################################
            if( line.enable_position_history ){

                if( gconf.gedm_flushing_motion ){


                     //#####################################################################
                     // Flushing active
                     //#####################################################################

                    if( gconf.gedm_retraction_motion ){ 

                        // Note: If the wanted distance is not possible it will not reach the wanted step_num and enforce a forward motion once the limit is reached
                        // retracting
                        if( flconf.steps_done_back >= flconf.steps_wanted ){
                            // wanted position reached; retraction fully done
                            // returning this function with success false will break the retraction and return to forward
                            _success = false;
                            break;
                        }
                        ++flconf.steps_done_back;

                    } else {

                        int mplan = 1; // default; ignore until pwm is back on

                        if( 
                            flconf.steps_done_forward >= flconf.steps_done_back - flushing.offset_steps*3 
                        ){
                            //reset_short_circuit_protection();
                            flush_reenable_pwm();
                            mplan = get_calculated_motion_plan( true ); //get_motion_plan( true )
                        } 
                        
                        // back forward
                        if( flconf.steps_done_forward >= flconf.steps_done_back-flushing.offset_steps || ( mplan >= 4 && flconf.steps_done_back > 2 ) ){
                            // back to the initial position minus the offset steps
                            flush_end();
                            _success = false; //
                            break;
                        }

                        ++flconf.steps_done_forward;

                    }

                } else {

                     //#####################################################################
                     // Normal process move
                     //#####################################################################

                    if( gconf.gedm_retraction_motion ){
                        ++_state.total_retraction_steps;
                        ++rconf.steps_done;
                    } else {
                        ++no_load_steps; // very dirty.. assuming a load until it reset, This resets all around the code and is not accurate at all #todo
                        if( _state.total_retraction_steps > 0 ){
                            --_state.total_retraction_steps;
                        }
                    }

                }
            }

        }


        // Bresenham line algo
        sync_steps = 0; // the number of parallel steps tol be fired: 1 to N_AXIS is possible
        for (int axis = 0; axis < N_AXIS; axis++) {

            line.line_math[axis].counter += line.line_math[axis].steps;
            //if( line.line_math[axis].counter > line.step_event_count ) { // "">" could it be ">="?
            if( line.line_math[axis].counter >= line.step_event_count ) { // "">" could it be ">="?

                line.line_math[axis].counter -= line.step_event_count;
                if( line.direction_bits & bit(axis) ) {
                    sys_position[axis]--;
                } else {
                    sys_position[axis]++;
                }
                motor_manager.motor_step( axis );
                // here should be something to equalize delay based on how many axes are stepped
                // having a single axis moving vs 4 axes is a big difference that may require some compensation
                // even on short circuit I don't think that it would be a good idea to early exit this loop
                // not sure how it would affect the position if skipped mid loop at this point
                // io theory it should find the final position again while retracting and going back but not risking it for now...
                ++sync_steps;
            } 

        }



        




        stepped_at = esp_timer_get_time();

        if( sync_steps > 1 && line.motion_plan_enabled && !gconf.gedm_flushing_motion ){
            stepped_at += process_speeds.RAPID; // if there where multiple steps give it some extra time
        }

        delayMicroseconds(1);
        ++line.step_count;

    } // ./end of while loop

    


    if( _success && !run_simulated && !gconf.gedm_flushing_motion ){
        // Only used in wire mode to do a little extra stuff on a line end in forward direction
        if( is_system_op_mode( OP_MODE_EDM_WIRE ) && !gconf.gedm_retraction_motion ){
            wire_line_end( line );
        }
    }


    // delay the minimum required and finish the line. 
    // stepped_at is set with the last step and it would not fire a step with the next line
    // if the time for it has not yet passed
    // but it can set the direction pins too early as this happens outside the stepping loop
    // so let's just finish the line with the correct timing end
    int delay_rest = process_speeds.RAPID - ( esp_timer_get_time() - stepped_at );
    if( delay_rest > 0 ){ delayMicroseconds( delay_rest ); } // 
    return _success;
}



/** 
  * This is the main gateway for lines
  * All normal lines are passed through this except for some special motions
  **/
uint8_t IRAM_ATTR G_EDM_PLANNER::plan_history_line( float* target, plan_line_data_t* pl_data ) {
    bool _success = true;
    // exit clean on aborts and motionstops
    if( get_quit_motion() ){
        idle_timer = esp_timer_get_time();
        override_target_with_current( target );
        gcode_core.gc_sync_position();
        gconf.gedm_planner_line_running = false;
        return false;
    }

    gconf.gedm_planner_line_running = true; 

    Line_Config line;

    if( pl_data->is_arc ){
        line.is_arc = true;
        ++_state.arc_counter;
    } else {
        line.is_arc = false;
        _state.arc_counter = 0;
    }

    // change the step delay / speed for this line; This may be overwritten in the process and is just a default value
    if( pl_data->step_delay ){
        line.step_delay = pl_data->step_delay;
    } else{
        line.step_delay = process_speeds.RAPID;
    }

    // set the default line configuration
    line.ignore_limit_switch = pl_data->use_limit_switches ? false : true; // defaults is no limits except for homing!

    if( 

        ( !is_machine_state( STATE_PROBING ) 
        && !pl_data->motion.systemMotion) 
        && is_system_mode_edm()

    ){ 

        if( ! run_simulated ){
            line.enable_position_history = true;
            line.step_delay              = process_speeds.EDM;
            line.motion_plan_enabled     = true;

            if( is_system_op_mode( OP_MODE_EDM_SINKER ) ){ // flushing interval needed only in sinker mode
                line.enable_flushing = true;
            }

        } else {
            line.enable_position_history = false;
            line.motion_plan_enabled     = false;
            line.step_delay              = process_speeds.RAPID;
        }

    } else if( is_machine_state( STATE_PROBING ) ){
        line.step_delay = process_speeds.PROBING;
    }

    if( line.motion_plan_enabled ){
        set_retraction_steps();
    }

    int32_t __target[N_AXIS];
    convert_target_to_steps( target, __target ); 

    _success = process_stage( __target, line );
    if( ! _success || system_block_motion ){
        override_target_with_current( target ); // if the line failed or motion got canceled override the target with the current position to update the position inside the gcode parser
    }
    position_history_force_sync(); // redundant? After sync this is also called.. Keept it for now..
    gcode_core.gc_sync_position(); 
    idle_timer = esp_timer_get_time();
    gconf.gedm_planner_line_running = false;


    return _success;
}




void G_EDM_PLANNER::set_simulation( bool simulate ){
    run_simulated = simulate;
}



bool setting_change_notify_callback_planner( setget_param_enum param_id, settings_container data ){

    switch( param_id ){

        // flushing configs runs on a copy of the settings data for performance reasons
        case PARAM_ID_FLUSHING_FLUSH_NOSPRK: 
            flushing.spark_disabled = data.value == 0 ? false : true;
        break;
        case PARAM_ID_FLUSHING_INTERVAL: 
            flushing.interval = data.fvalue > 0.0 ? round( data.fvalue * 1000000.0 ) : 0;
        break;
        case PARAM_ID_FLUSHING_DISTANCE: 
            flushing.distance = data.fvalue;
        break;
        case PARAM_ID_FLUSHING_FLUSH_OFFSTP: 
            flushing.offset_steps = data.value;
        break;

        // converting the distances to steps for performance and less in-loop computation
        case PARAM_ID_BROKEN_WIRE_MM: 
        case PARAM_ID_RETRACT_S_MM:
        case PARAM_ID_RETRACT_H_MM:
            planner.set_retraction_steps();
        break;

        case PARAM_ID_EARLY_RETR:
            rconf.enable_early_exit = data.value == 0 ? false : true;
        break;

        case PARAM_ID_EARLY_X_ON:
            plconfig.early_exit_on_plan = data.value;
        break;

        case PARAM_ID_SHORT_DURATION:
            plconfig.short_circuit_max_duration_us = data.value * 1000; // convert from ms to us
        break;

        case PARAM_ID_MAX_REVERSE:
            plconfig.max_reverse_depth = data.value;
        break;

        case PARAM_ID_LINE_ENDS:
            plconfig.line_to_line_confirm_counts = data.value;
        break;

        case PARAM_ID_SIMULATION: // planner makes a copy for... right. performance..
            planner.set_simulation( data.value == 1 ? true : false );
        break;


        default: break; 
    }
    
    return true;

}


void G_EDM_PLANNER::settings_init(){

    notify_callbacks[ SETTING_NOTIFY_PLANNER ] = setting_change_notify_callback_planner; // add callback


    settings.add( PARAM_ID_SIMULATION,            SETTING_TYPE_BOOL,  0,                             0.0, 0.0, 0.0, SETTING_NOTIFY_PLANNER );
    settings.add( PARAM_ID_EARLY_RETR,            SETTING_TYPE_BOOL,  DEFAULT_ENABLE_EARLY_EXIT?1:0, 0.0, 0.0, 0.0, SETTING_NOTIFY_PLANNER );
    settings.add( PARAM_ID_FLUSHING_FLUSH_NOSPRK, SETTING_TYPE_BOOL,  1,                                       0.0,                          0.0, 0.0,     SETTING_NOTIFY_PLANNER );
    settings.add( PARAM_ID_FLUSHING_INTERVAL,     SETTING_TYPE_FLOAT, 0,                                       0.0,                          0.0, 2000.0,  SETTING_NOTIFY_PLANNER ); // stored in seconds for user experience, micros required later
    settings.add( PARAM_ID_FLUSHING_DISTANCE,     SETTING_TYPE_FLOAT, 0,                                       DEFAULT_FLUSHING_DISTANCE,    0.0, 200.0,   SETTING_NOTIFY_PLANNER );
    settings.add( PARAM_ID_FLUSHING_FLUSH_OFFSTP, SETTING_TYPE_INT,   20,                                      0.0,                          1.0, 2000.0,  SETTING_NOTIFY_PLANNER );
    settings.add( PARAM_ID_BROKEN_WIRE_MM,        SETTING_TYPE_FLOAT, 0,                                       DEFAULT_RETRACTION_MAX_DIST,  0.0, 100.0,   SETTING_NOTIFY_PLANNER ); // 
    settings.add( PARAM_ID_RETRACT_S_MM,          SETTING_TYPE_FLOAT, 0,                                       DEFAULT_RETRACTION_SOFT_DIST, 0.0, 100.0,   SETTING_NOTIFY_PLANNER ); // 
    settings.add( PARAM_ID_RETRACT_H_MM,          SETTING_TYPE_FLOAT, 0,                                       DEFAULT_RETRACTION_HARD_DIST, 0.0, 100.0,   SETTING_NOTIFY_PLANNER ); //
    settings.add( PARAM_ID_SHORT_DURATION,        SETTING_TYPE_INT, DEFAULT_SHORTCIRCUIT_MAX_DURATION_US/1000, 0.0,                          0.0, 20000.0, SETTING_NOTIFY_PLANNER ); // stored in ms, converted to us later
    settings.add( PARAM_ID_MAX_REVERSE,           SETTING_TYPE_INT, DEFAULT_MAX_REVERSE_LINES,                 0.0,                          1.0, ( float ) POSITION_HISTORY_LENGTH-1, SETTING_NOTIFY_PLANNER ); // 
    settings.add( PARAM_ID_LINE_ENDS,             SETTING_TYPE_INT, DEFAULT_LINE_END_CONFIRMATIONS,            0.0,                          0.0, 0.0,     SETTING_NOTIFY_PLANNER ); // 
    settings.add( PARAM_ID_EARLY_X_ON,            SETTING_TYPE_INT, DEFAULT_EARLY_EXIT_ON_PLAN,                0.0,                          1.0, 4.0,     SETTING_NOTIFY_PLANNER );

    // initial build; planner uses hard copies for performance instead of accessing the settings object with the lock all the time
    flushing.interval                      = 0;
    flushing.distance                      = DEFAULT_FLUSHING_DISTANCE;
    flushing.offset_steps                  = DEFAULT_FLUSHING_OFFSET_STEPS;
    flushing.spark_disabled                = true;
    plconfig.line_to_line_confirm_counts   = DEFAULT_LINE_END_CONFIRMATIONS;
    plconfig.max_reverse_depth             = DEFAULT_MAX_REVERSE_LINES;
    plconfig.short_circuit_max_duration_us = DEFAULT_SHORTCIRCUIT_MAX_DURATION_US; // convert from ms to us
    plconfig.early_exit_on_plan            = DEFAULT_EARLY_EXIT_ON_PLAN;
    rconf.enable_early_exit                = DEFAULT_ENABLE_EARLY_EXIT;

    planner.set_retraction_steps();
    planner.set_simulation( false );

}


void G_EDM_PLANNER::init(){
    settings_init();
}