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

#include "ili9341_tft.h"
#include "Motors/Motors.h"
#include "Protocol.h"
#include "gedm_dpm_driver.h"
#include "gpo_scope.h"
#include "widgets/ui_interface.h"
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <assert.h>
#include <string.h>

GUI ui_controller;

TaskHandle_t ui_task_handle    = NULL;
TaskHandle_t drill_task_handle = NULL;

void IRAM_ATTR debuglog( const char* line, uint16_t task_delay ){ // add a line to the console
    if( !start_logging_flag.load() ) return;
    charge_logger( line );
    if( task_delay ){ vTaskDelay(task_delay); }

}
void IRAM_ATTR remote_control( int data ){ // can be used for some functions to perform some stuff from both cores by submitting a command to a wait queue
    if( remote_control_queue != NULL ){
        xQueueSend( remote_control_queue, &data, 50000 );
    } 
}
void update_scope_stats(){ // this function is called on a page change (either keaboard closed or page navigation) scope will redraw with the new values
    arcgen.lock();
    gscope.update_config( 
        settings.get_setting_int( PARAM_ID_SLOW_CFD_AVG_SIZE ), 
        settings.get_setting_int( PARAM_ID_VDROP_THRESH ),
        settings.get_setting_float( PARAM_ID_DUTY ),
        settings.get_setting_float( PARAM_ID_FREQ ),
        percentage_to_adc( settings.get_setting_float( PARAM_ID_SETMIN ) ),
        percentage_to_adc( settings.get_setting_float( PARAM_ID_SETMAX ) )
    );
    arcgen.unlock();
}


GUI::GUI(){ 

    change_sinker_axis_direction(0); 

}

void GUI::start_ui_task(){ // creates the main ui task that will take care of all the user interactions

    xTaskCreatePinnedToCore( ui_task, "ui_task", UI_TASK_STACK_SIZE, this, TASK_UI_PRIORITY, &ui_task_handle, UI_TASK_CORE ); 

}


void GUI::run_once_if_ready(){              // runs once after the ui is ready
    api::push_cmd("$HCT\r");                // force machine to autohome at center position of each axis on startup
    api::push_cmd("G28.1\r\n");             // set current position to home position
    reset_machine_state();                  // reset machine state (remove alarm and set state to idle etc.)
    api::reset_probe_points();              // delete probe positions
    api::reset_work_coords();               // reset work coordinates
    api::update_speeds();                   // create the step delays for the mm/min speeds
    reset_defaults();                       // reset defaults
    gcode_core.sync_xyz_block();            // sync something
}

void GUI::reset_defaults(){
    int data = 1;
    probing.probe_reset();                        // reset probe variables
    set_system_op_mode( OP_MODE_NORMAL ); // set operation mode to normal
    enforce_redraw.store( false );        // unflag enforce redraw
    planner.reset_planner_state();        // reset planner things
    ui.emit(RESET,&data);                 // emit reset event to ui
    api::push_cmd("G91 G54 G21\r\n");     // send some g codes
}

void GUI::reset_after_job(){
    debuglog("Resetting",DEBUG_LOG_DELAY);
    set_system_op_mode( OP_MODE_NORMAL ); // set operation mode to normal
    int page = PAGE_FRONT;
    settings.change_setting( PARAM_ID_DPM_ONOFF, 0 );     // disable dpm
    vTaskDelay(50);
    settings.change_setting( PARAM_ID_PWM_STATE, 0 );     // disable arc generator
    settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 0 ); // stop the wire if still running
    system_block_motion = true;                           // set stop motion flag
    wire_feeder.reset();                                  // reset wire motors
    planner.position_history_reset();                     // reset planner position history
    enforce_redraw.store( false );                        // unflag enforce restart
    wait_for_idle_state();                                // wait for idle state
    reset_defaults();                                     // reset defaults
    G_SENSORS::unlock_motion_switch();                    // unlock and reread motion switch
    ui.emit(SHOW_PAGE, &page );                           // show the frontpage
}


//#########################################################
// enables DPM, resets some stuff, pushes break to planner
// loads process page etc.
// shared across all modes before the process starts
//#########################################################
void GUI::pre_process_defaults(){
    debuglog("Prepare process",DEBUG_LOG_DELAY);
    gconf.edm_pause_motion = false;
    int page = PAGE_IN_PROCESS;
    ui.emit(SHOW_PAGE, &page );    // show the process page
    settings.change_setting( PARAM_ID_DPM_ONOFF, 1 );
    vTaskDelay(DEBUG_LOG_DELAY);
    arcgen.pwm_off();
    planner.push_break_to_position_history();
    arcgen.pwm_on();
    vTaskDelay(200);
}



void GUI::alarm_handler(){
    set_system_op_mode( OP_MODE_NORMAL );                     // set system operation mode to normal
    settings.change_setting( PARAM_ID_PWM_STATE, 0 );           // disable arc generator
    if( settings.get_setting_bool( PARAM_ID_SPINDLE_ENABLE ) ){ 
        settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 0 );
    } // stop wire feeder
    //dpm_driver.power_on_off( 0, 3 ); // turn DPM off
    settings.change_setting( PARAM_ID_DPM_ONOFF, 0 );
    vTaskDelay(100);
    int error_code = get_alarm();                             // get error code
    ui.emit(ALARM,&error_code);                               // show alarm page
    gconf.gedm_disable_limits = true;                         // ensure limits are ignored until reset is performing
    reset_machine_state();                                    // reset machine state (remove alarm and set to idle)
    reset_after_job();                                        // call the reset after job for cleanup
    vTaskDelay(1);
    gconf.gedm_disable_limits = false;                        // re-enable limit switches
    remote_control( CMD_READ_LIMITSWITCH );                                      // re-read limit switch state
    debuglog(" Recovery done");                               // recovery from alarm done
}

void clean_reboot(){
    Serial.println("Reboot requested");
    arcgen.end();        // disable PWM
    gsense.sensor_end(); // shut down sensors
    Serial.end();        // end serial
    ui.exit();           // cleanup ui
    ESP.restart();       // restart esp
}


#define MPOS_UPDATE_INTERVAL 200000 // 1s = 1000ms = 1000000us
IRAM_ATTR bool position_changed( int64_t &current_micros ){ // todo: build a flag triggered from the stepper class instead of looping and comparing the position..

    static DRAM_ATTR int64_t last_mpos_update          = 0; // micros
    static DRAM_ATTR int32_t sys_last_position[N_AXIS] = {0,};

    if( current_micros - last_mpos_update < MPOS_UPDATE_INTERVAL ){
        return false;
    }
    last_mpos_update = current_micros;
    for( int i = 0; i < N_AXIS; ++i ){
        if( sys_last_position[i] != sys_position[i] ){
            sys_last_position[i] = sys_position[i];
            return true;
            break;
        }
    }
    
    return false;

}

void show_credits(){ // create the credits for the boot screen
    if( ! is_machine_state( STATE_REBOOT ) ){
        // Show credits
        const char* credits[] = {
            ""," --- Credits to the community ---","",
            "@Tanatara", "@8kate", "@Ethan Hall", "@Esteban Algar", "@666", "@td1640", "@MaIzOr", "@gunni", "@DANIEL COLOMBIA", 
            "@charlatanshost", "@tommygun", "@Nikolay", "@renyklever", "Arne S.", "@Zenitheus", "@gerritv", "@cnc", "@TK421", "@AndrewS", "@sarnold04",
            "@Shrawan Khatri", "@Alex Treseder", "@VB","","","", " --- Welcome ---", "" , "", BRAND_NAME, RELEASE_NAME, FIRMWARE_VERSION // brand
        };
        size_t array_size = sizeof(credits) / sizeof(credits[0]);
        for( int i = 0; i < array_size; ++i ){
            debuglog( credits[i], 30 );
        }
        vTaskDelay(1000);
    }
}


//#########################################################
// The main user interface loop task running on core 0
//#########################################################
void IRAM_ATTR ui_task(void *parameter){

    GUI *ui_controller = (GUI *)parameter;

    static DRAM_ATTR int64_t last_minimal_ui_update_micros = esp_timer_get_time();
    static DRAM_ATTR int64_t micros_recent                 = 0;
    static DRAM_ATTR int64_t time_since                    = 0;
    static DRAM_ATTR int     dummy_item                    = 0; // used for the emit() function as dummy...

    ui_controller->settings_init();

    //#########################################
    // Calculate the step delays 
    //#########################################
    api::update_speeds();
    //#########################################
    // Create the event callbacks
    //#########################################
    ui_controller->setup_events();

    planner.init();

    start_logging_flag.store(true);
    ui.build_phantom();

    while( !ui_controller->get_is_ready() ){ // wait until ui is ready
      vTaskDelay(DEBUG_LOG_DELAY);
    }

    ui.init_sd(); // checks for firmware update too

    if( is_machine_state( STATE_REBOOT ) ){

        debuglog(" Reboot required!", DEBUG_LOG_DELAY );

    } else {


        // Preparing the DPM/DPH module
        dpm_driver.setup( Serial );
        dpm_driver.init();

        // Create the wire module
        wire_feeder.create();

        if( ! is_machine_state( STATE_REBOOT ) ){
            wire_feeder.init(); // skip if restart flag is set
        } 


        // Create arc generator
        arcgen.create(); // needs to be done before loading the last session as ledc is not initialized yet

        // Create the sensors and queues etc.
        gsense.create_sensors(); // needs to be done before loading the settings as the settings library is not created yet

        ui.restore_last_session(); // restores the last session etc., 
        
        // now the settings are loaded
        if( ! is_machine_state( STATE_REBOOT ) ){
            dpm_driver.initial_enable(); // it will try to turn off the dpm if support enabled and disable support if it fails
        } else {
            debuglog(" Reboot required!"); 
            clean_reboot();
        }
        
        // Initial read edm start/stop, limit states..
        remote_control( CMD_READ_ONOFF_SWITCH ); // G_SENSORS::motion_switch_read();
        remote_control( CMD_READ_LIMITSWITCH ); // G_SENSORS::limit_switch_read();
 
        ui_controller->run_once_if_ready();
        position_changed( micros_recent );
        show_credits();

        enforce_redraw.store( false );
        int page = PAGE_FRONT;
        ui.emit(SHOW_PAGE,&page);
        ui.set_is_ready(); // flag is_ready in ui

    }


    //int64_t test_start = 0;
    //bool    test_flag  = false;





    for (;;){

        /*if( esp_timer_get_time() - test_start > 300000 ){
            set_system_op_mode( test_flag ? OP_MODE_EDM : OP_MODE_NORMAL );
            test_flag = !test_flag;
            test_start = esp_timer_get_time();
            vTaskDelay(100); 
            ui.emit(RELOAD_ON_CHANGE,&dummy_item);
            ui.emit(EVENT_TOUCH_PULSE,&dummy_item);
            continue;
        } continue;*/

        vTaskDelay(1);

        // No need to call this with every iteration
        if( is_machine_state( STATE_REBOOT ) ){
            clean_reboot();
            vTaskDelay(1);
            continue;
        } else if( is_machine_state( STATE_PROBING ) ){ // the normal process will never come here since probing is a blocking function inside this task. But let it be.
            vTaskDelay(1000);
            continue;
        }

        if( get_show_error_page() ) { // enter alarm loop and halt all motion
            ui_controller->alarm_handler();
        }

        if( !is_system_mode_edm() ){
          
            //######################################
            // Not in EDM process
            //######################################

            if( is_machine_state( STATE_HOMING ) ){
                vTaskDelay(100);
            }

            micros_recent = esp_timer_get_time();

            if( position_changed( micros_recent ) || enforce_redraw.load() || state_changed() ){
                enforce_redraw.store( false );
                if( motion_switch_changed.load() ){
                    motion_switch_changed.store( false );
                    if( get_quit_motion() && wire_feeder.is_running() ){
                        settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 0 );
                    }
                    debuglog( get_quit_motion() ? "*Motion OFF" : "@Motion ON" );
                }
                ui.emit(RELOAD_ON_CHANGE,&dummy_item);
            }

            if( 
                ui_controller->active_page == PAGE_FRONT && // start only if ui is on the frontpage
                ui_controller->ready_to_run()               // start only if everything else is ready too (this is also called in the process where the page is not the frontpage)
            ){ ui_controller->start_process(); }

            ui.emit(EVENT_TOUCH_PULSE,&dummy_item); // should be at the end; if stuff like factory reset is called that flags the restart variable we want to avoid any more stuff going on

        } else {

            //##########################################
            // In EDM process; slower refresh rates etc.
            //##########################################

            if ( get_quit_motion( true ) || is_machine_state( STATE_IDLE ) ){ // check for break conditions fist

                //########################################################
                // Process finished or aborted
                //########################################################

                if( settings.get_setting_int( PARAM_ID_MODE ) == ( int ) MODE_2D_WIRE ){
                    ui.emit(END_SD_CARD_JOB,&dummy_item); // doesn't need any data.. just use the page.. Doesn't matter if it was a gcode job or not. 
                }

                // Wait until everything is done and idle
                wait_for_idle_state( 100, false );
                
                // Reset stuff
                ui_controller->reset_after_job();

                // Fire the finish functions. Normally if edm_process_finished flag is set it should indicate a clean process end. But so much changed
                // and I haven't looked deeper at it It is possible that the flag is not set correctly anymore.. #todo
                if( get_quit_motion()  ){
                    debuglog("*Process aborted");
                } else{ 
                    debuglog("@Process finished");
                }

                wait_for_idle_state(); // does return early if get_quit_motion() returns true
                
            } else {

                //########################################################
                // Active process still running
                //########################################################

                // If the active page is not the process page it is ok to trigger a full touch event scan and constinue
                if( ui_controller->active_page != PAGE_IN_PROCESS ){ // could be the settings page etc
                    ui.emit(EVENT_TOUCH_PULSE,&dummy_item); // if the keyboard is used it needs this touch monitor
                    vTaskDelay(1);
                    continue;
                }

                micros_recent = esp_timer_get_time();
                time_since    = micros_recent - last_minimal_ui_update_micros;

                if( enforce_redraw.load() ){ // needed?
                    enforce_redraw.store( false );
                    ui.wpos_update(); 
                    vTaskDelay(1);
                    continue;
                }

                // If process is paused or interval time is ok fire some update routines 
                if( gconf.edm_pause_motion || time_since > 280000 ){ //200ms = 100*1000 = 200000uS

                    ui.process_update();
                    if( position_changed( micros_recent ) || gconf.edm_pause_motion ){ // todo: check if position request is ongoing and send only if ui is not already informed
                        ui.wpos_update();
                    }

                    last_minimal_ui_update_micros = esp_timer_get_time();

                } else if( time_since > 100000 ){

                    ui.process_update_sd();

                }

                // Update scope only if process is not not paused
                if( !gconf.edm_pause_motion ){
                    gscope.draw_scope();vTaskDelay(10);
                }

            }

        }

        vTaskDelay(6);

    }

    vTaskDelete(NULL);

}





//##################################################
// 1D Drill/Sinker mode
// Setup everything and push the line to the buffer
//##################################################
void GUI::sinker_drill_single_axis(){

    pre_process_defaults();

    debuglog("Starting sinker job", DEBUG_LOG_DELAY );
    
    int    sinker_axis                = change_sinker_axis_direction( single_axis_mode_direction );
    bool   sinker_axis_move_negative  = true;
    float  cutting_depth              = settings.get_setting_float( PARAM_ID_SINK_DIST );
    float* current_position           = system_get_mpos();
    float  retraction_limit[N_AXIS];

    memcpy( retraction_limit, current_position, sizeof( current_position[0] ) * N_AXIS );

    //############################################################################
    // Get the max possible retraction position for the sinker axis and if XYUV
    // also get the max position for the chained slave axis and create a
    // position target for retractions that is pushed to the position history
    //############################################################################
    debuglog("Deleting sinker axis probe positions", DEBUG_LOG_DELAY );
    uint8_t axis_mask = api::create_xu_yv_axis_mask( sinker_axis );
    for( int _axis = 0; _axis < N_AXIS; ++_axis ){
        if( bitnum_istrue( axis_mask, _axis ) ){
            api::unset_probepositions_for( _axis ); // unset the probe position for the sinker axis
        }
    }

    if( !settings.get_setting_bool( PARAM_ID_SIMULATION ) ){

        if( settings.get_setting_bool( PARAM_ID_SPINDLE_ENABLE ) ){ 
            settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 1 );
        }


        arcgen.pwm_on();
        probe_prepare();

        switch (single_axis_mode_direction){
            case 0: // SINK_Z_NEG
                probing.probe_z_negative();
                break;
            case 1: // SINK_X_POS
                probing.probe_x_positive();
                sinker_axis_move_negative = false;
                break;
            case 2: // SINK_X_NEG
                probing.probe_x_negative();
                break;
            case 3: // SINK_Y_NEG
                probing.probe_y_negative();
                break;
            case 4: // SINK_Y_POS
                probing.probe_y_positive();
                sinker_axis_move_negative = false;
                break;
        }

        if( !get_quit_motion() ){
            for( int _axis = 0; _axis < N_AXIS; ++_axis ){
                if( bitnum_istrue( axis_mask, _axis ) ){
                    if( !sys_probed_axis[_axis] ){
                        debuglog("*Probe failed. Fallback to current.", 2000 );
                        api::auto_set_probepositions();
                        break; // one call to the above function is enough
                    }
                }
            }
        }
        
        probe_done();

    } else {
        if( !get_quit_motion() ){
            debuglog("Sinker job simulation", DEBUG_LOG_DELAY );
            if( 
                   single_axis_mode_direction == 1 // SINK_X_POS
                || single_axis_mode_direction == 4 // SINK_Y_POS
            ){ sinker_axis_move_negative = false; }
            api::auto_set_probepositions();
        }
    }


    if( !get_quit_motion() ){

        debuglog("Preparing line", DEBUG_LOG_DELAY );



    vTaskDelay(500);
    float min_max_possible[2];
    api::get_min_max_xyuvz( sinker_axis, min_max_possible );
    float axis_target_relative = sinker_axis_move_negative ? cutting_depth*-1 : cutting_depth; // the "wanted" target position
    //############################################################################
    // Ensure the target position is withing the possible range and
    // adjust the wanted position the the possible position if needed
    //############################################################################
    //Serial.println(  min_max_possible[0] );
    //Serial.println(  min_max_possible[1] );
    if( sinker_axis_move_negative ){
        if( axis_target_relative < min_max_possible[0] ){
            // wanted position is not available; fallback to max possible
            axis_target_relative = min_max_possible[0];
        }
    } else {
        if( axis_target_relative > min_max_possible[1] ){
            // wanted position is not available; fallback to max possible
            axis_target_relative = min_max_possible[1];
        }
    }
    //############################################################################
    // Create the retraction history object that is used to have a position to 
    // retract to in the planner. With XYUV this needs to be relative to prevent
    // a tilt wire due to inequal distances in the retraction target
    //############################################################################
    for( int _axis = 0; _axis < N_AXIS; ++_axis ){
        if( bitnum_istrue( axis_mask, _axis ) ){
            if( sinker_axis_move_negative ){
                // sinker axis moves negative; retractions are positive and the retraction_limits needs to get more positive
                retraction_limit[_axis] += min_max_possible[1];
            } else {
                // sinker axis moves positive; retractions are negative and the retraction_limit needs to get more negative
                retraction_limit[_axis] += min_max_possible[0];
            }
        }
    }
    //############################################################################
    // Setup planner stuff
    //############################################################################
    int32_t __target[N_AXIS];
    planner.convert_target_to_steps( retraction_limit, __target );
    planner.configure();
    planner.push_break_to_position_history();               // lock the line history by pushing a break to it; while retracting it can only move back in history until a break stops it
    planner.push_to_position_history( __target, false, 0 ); // push retraction max position to the history
    planner.push_current_mpos_to_position_history();        // push current mpos to history
    planner.position_history_force_sync();                  // sync the history index
    vTaskDelay(200);
    //############################################################################
    // Enable DPM/DPH, turn pwm on etc.
    //############################################################################
    if( settings.get_setting_bool( PARAM_ID_DPM_UART ) ){
    //if( dpm_driver.support_enabled() ){ 
        //dpm_driver.power_on_off( 1, 5 );
        settings.change_setting( PARAM_ID_DPM_ONOFF, 1 );
    } else {
        debuglog("*DPM/DPH support not enabled");
        debuglog("*Please turn DPM on manually",1000);
    }
    arcgen.pwm_on();                     // turn pwm on
    planner.reset_flush_retract_timer(); // reset flushing interval time
    //api::push_cmd("M7\r");               // send M7 to toggle drill/sinker mode this needs to be done after the probing stuff!
    set_system_op_mode( OP_MODE_EDM_SINKER );
    //############################################################################
    // Create the GCode
    //############################################################################

    if( !settings.get_setting_bool( PARAM_ID_SIMULATION ) && settings.get_setting_bool( PARAM_ID_SPINDLE_ENABLE ) ){ 
        settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 1 );
    }

    
    gsense.reset_sensor_global();


    float values[N_AXIS] = {0.0,};
    api::send_axismaks_motion_gcode( axis_mask, "G90 G0", " F30\r\n", values ); // move involved sinker axes to zero position
    for( int _axis = 0; _axis < N_AXIS; ++_axis ){
        if( bitnum_istrue( axis_mask, _axis ) ){
            values[_axis] = axis_target_relative; // update relative position
        }
    }
    api::send_axismaks_motion_gcode( axis_mask, "G91 G1", " F30\r\n", values ); // sinker line
    vTaskDelay(1000);

    } else {
        debuglog("Motion disabled. Aborting.", DEBUG_LOG_DELAY );
        //arcgen.change_setting( PARAM_ID_PWM_STATE, 0 );
        reset_after_job();
    }
}






//################################################
// Enter and exit probe mode. 
// It will set the probe PWM/Duty/Voltage/Current
// and resets them after probing 
//################################################
void GUI::probe_done(){
    if( settings.get_setting_bool( PARAM_ID_SPINDLE_ENABLE ) ){ 
        settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 0 );
        wire_feeder.restore();
    }
    arcgen.probe_mode_off();
    dpm_driver.restore_backup_settings(); // all dpm stuff will do nothing internally if disabled
    settings.change_setting( PARAM_ID_DPM_ONOFF, 0 );
    if( get_quit_motion() ) return;
    debuglog("Probe mode off", DEBUG_LOG_DELAY );
}

bool GUI::probe_prepare(){
    debuglog("Probe mode on", DEBUG_LOG_DELAY );
    arcgen.lock();
    // if dpm commjnication is enabled we can adjust
    // voltage and current for probing
    // backup the current settings
    dpm_driver.backup_current_settings(); // all dpm stuff will do nothing internally if disabled
    dpm_driver.set_setting_voltage( settings.get_setting_float( PARAM_ID_DPM_PROBE_V ) );
    dpm_driver.set_setting_current( settings.get_setting_float( PARAM_ID_DPM_PROBE_C ) );
    //dpm_driver.power_on_off( 1, 5 );
    settings.change_setting( PARAM_ID_DPM_ONOFF, 1 );
    debuglog("Setting probe PWM", DEBUG_LOG_DELAY );
    arcgen.probe_mode_on();
    if( !settings.get_setting_bool( PARAM_ID_SIMULATION ) && settings.get_setting_bool( PARAM_ID_SPINDLE_ENABLE ) ){ 
        wire_feeder.backup();
        settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 1 );
    }
    arcgen.unlock();
    if( get_quit_motion() ) return true;
    vTaskDelay(1000);
    return true;
}




//####################################################################
// If the basic requirements are ok it will start the specific process
// if something is missing it will create an info screen
//####################################################################
void GUI::start_process(void){

    info_codes msg_id = INFO_NONE;

    int dummy = 1;

    int edm_mode = settings.get_setting_int( PARAM_ID_MODE );

    if( edm_mode == MODE_2D_WIRE ){

        logger_show_on_line.store( true );

        //###################################
        // Check requirements for 2D wire
        //###################################
        if( !api::machine_is_fully_homed() ){
            msg_id = NOT_FULLY_HOMED;
        } else if( !settings.get_setting_bool( PARAM_ID_SD_FILE_SET ) ){
            msg_id = SELECT_GCODE_FILE;
        }

        if( msg_id != INFO_NONE ){

            //###################################
            // Requirements not ok.
            //###################################
            settings.change_setting( PARAM_ID_PWM_STATE, 0 );
            debuglog( info_messages[msg_id] );

        } else {

            //##########################################
            // Requirements for 2D wire ok. Ready to go.
            //##########################################
            // to reduce complexity and the need for probing
            // it will not require probing
            // if the axes are not probed it will set the current position as probe position
            // but it will not overwrite existing probe positions if there are some set
            // if for example x has a probe position and y not the function
            // will keep the x probe position and use the current y position as y probe position
            debuglog( "Setting probe positions", DEBUG_LOG_DELAY );
            api::auto_set_probepositions();
            //if( !sys_probed_axis[axes_active.x.index] || !sys_probed_axis[axes_active.y.index] ){ api::auto_set_probepositions(); }
            pre_process_defaults();
            debuglog("*Starting 2D wire job",200);
            api::push_cmd("G90 G54\r\n");
            planner.configure();
            set_system_op_mode( OP_MODE_EDM_WIRE );

            if( !settings.get_setting_bool( PARAM_ID_SIMULATION ) && settings.get_setting_bool( PARAM_ID_SPINDLE_ENABLE ) ){ 
                settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 1 ); // start wire
            }
            
            arcgen.pwm_on();
            gsense.reset_sensor_global();


            // all set; start sd card line shifting
            logger_show_on_line.store( false );
            ui.emit(RUN_SD_CARD_JOB,&dummy);

        }

    } else if ( edm_mode == MODE_SINKER ){

        logger_show_on_line.store( true );

        //###################################
        // Check requirements for 1D sinker
        //###################################
        if( msg_id != INFO_NONE ){

            //###################################
            // Requirements not ok.
            //###################################
            settings.change_setting( PARAM_ID_PWM_STATE, 0 );
            debuglog( info_messages[msg_id] );

        } else {

            //############################################
            // Requirements for 1D sinker ok. Ready to go.
            //############################################
            sinker_drill_single_axis();
            logger_show_on_line.store( false );

        }

  }

  logger_show_on_line.store( false );

}

bool GUI::ready_to_run(){
    if (
           get_quit_motion()               // abort or motion off
        || !arcgen.is_running()            // pwm off
        || is_system_mode_edm()            // edm_process running
        || !sensors_task_running()         // i2s not ready
        || !is_machine_state( STATE_IDLE ) // machine not idle
    ){ return false; }
    vTaskDelay(DEBUG_LOG_DELAY);
    return true;
}

bool GUI::get_is_ready(){
    return protocol_ready.load();
}


//###################################
// Check if motion input is blocked
//###################################
bool GUI::motion_input_is_blocked() {
    if( 
        is_system_mode_edm() 
        || get_quit_motion() 
        || is_machine_state( STATE_HOMING )
    ){ 
        return true; 
    }
    return false;
}

//#######################################
// Change the sinker axis
// this needs a review for custom
// axis object like YZ motion only etc..
// pretty sure this is buggy
//#######################################
int GUI::change_sinker_axis_direction( int direction ){

    int axis = 0;

    if( ! axes_active.z.enabled && direction == 0 ){
        ++direction; // fallback to XY; if X or Y is missing too = problem
    }

    if( ! axes_active.x.enabled && ( direction == 1 || direction == 2 ) ){
        direction = 3; // fallback to Y; if Y is missing too = problem
    }

    if( ! axes_active.y.enabled && ( direction == 3 || direction == 4 ) ){
        direction = axes_active.z.enabled ? 0 : 1; // fallback 
    }


    if( !axes_active.y.enabled && !axes_active.y.enabled ){
        direction = 0; // force to z
    }

    single_axis_mode_direction = direction;

    switch (single_axis_mode_direction){
        case 0:
            axis = axes_active.z.index;
        break;
        case 1:
            axis = axes_active.x.index;
        break;
        case 2:
            axis = axes_active.x.index;
        break;
        case 3:
            axis = axes_active.y.index;
        break;
        case 4:
            axis = axes_active.y.index;
        break;
    }
    if( axis >= N_AXIS ){
        axis = 0; // hacky fallback with unkown result if triggered
    }
    planner.set_sinker_axis( axis );
    return axis;
}









//###################################
// Raw setter functions 
//###################################





// not very suitable for the settings object yet
void get_mpos_x( getter_param * item ){
    item->floatValue = system_get_mpos_for_axis( axes_active.x.index );
}
void get_mpos_y( getter_param * item ){
    item->floatValue = system_get_mpos_for_axis( axes_active.y.index );
}
void get_mpos_z( getter_param * item ){
    item->floatValue = system_get_mpos_for_axis( axes_active.z.index );
}
void get_mpos_u( getter_param * item ){
    item->floatValue = system_get_mpos_for_axis( axes_active.u.index );
}
void get_mpos_v( getter_param * item ){
    item->floatValue = system_get_mpos_for_axis( axes_active.v.index );
}
IRAM_ATTR void get_wpos_x( getter_param * item ){
    float *wpos = api::get_wpos();
    item->floatValue = wpos[axes_active.x.index];
}
IRAM_ATTR void get_wpos_y( getter_param * item ){
    float *wpos = api::get_wpos();
    item->floatValue = wpos[axes_active.y.index];
}
IRAM_ATTR void get_wpos_z( getter_param * item ){
    float *wpos = api::get_wpos();
    item->floatValue = wpos[axes_active.z.index];
}
IRAM_ATTR void get_wpos_u( getter_param * item ){
    float *wpos = api::get_wpos();
    item->floatValue = wpos[axes_active.u.index];
}
IRAM_ATTR void get_wpos_v( getter_param * item ){
    float *wpos = api::get_wpos();
    item->floatValue = wpos[axes_active.v.index];
}

void get_motion_state( getter_param * item ){ // called all over. Needs to be fast
    item->boolValue = ui_controller.motion_input_is_blocked() ? false : true;
}







// set home axis; int value AXIS or -1 for all
void set_home_axis( setter_param * item ){
    if( ui_controller.motion_input_is_blocked() ){
        return;
    }
    int value = char_to_int( item->second );
    if( value == -1 ){ 
        api::home_machine(); // home all axis: z is excluded if z is disabled;
    } else { api::home_axis( value ); }
}
void set_home_all( setter_param * item ){
    if( ui_controller.motion_input_is_blocked() ) return;
    api::home_machine();
}

// set move to origin
void set_move_to_origin( setter_param * item ){
    if( ui_controller.motion_input_is_blocked() ) return;
    api::move_to_origin();
}

// set toggle homing enabled for axis; int value AXIS
void set_homing_enabled( setter_param * item ){
    int axis = char_to_int( item->second );
    bool current = g_axis[ axis ]->homing_seq_enabled.get();
    g_axis[ axis ]->homing_seq_enabled.set( !current );
}

// set probe position for axis; int value AXIS
void set_probing_pos_action( setter_param * item ){
    api::auto_set_probepositions_for( char_to_int( item->second ) );
}

// set unset probing point for axis; int value AXIS
void set_probing_pos_unset( setter_param * item ){
    api::unset_probepositions_for( char_to_int( item->second ) );
}












void set_jog_axis( float mm_to_travel, uint8_t axis, bool negative = false ){

    Serial.println("Jogging motion");

    if( axis > N_AXIS ){
        debuglog("Invalid motor selection");
        return;
    }

    if( axis == axes_active.z.index && !axes_active.z.enabled ){
        debuglog( info_messages[Z_AXIS_DISABLED] );
        debuglog(" Please create a valid Z axis" );
        debuglog(" in the definitions.cpp file first" );
        return;
    }

    if( ! motor_manager.get_motor( axis )->motor_enabled() ){
        debuglog("*Motor is disabled");
        debuglog(" Invalid Pin configuration");
        #ifdef CNC_TYPE_XYUVZ
            debuglog(" XYUVZ not supported yet");
        #endif
        return;
    }

    info_codes msg_id = INFO_NONE;

    if( ui_controller.motion_input_is_blocked() ){

        msg_id = MOTION_NOT_AVAILABLE;

    } else {

        if( negative ){
            mm_to_travel = mm_to_travel*-1;
        }

        if( !api::jog_axis( mm_to_travel, get_axis_name_const( axis ), settings.get_setting_float( PARAM_ID_SPEED_RAPID ), axis ) ){
            msg_id = TARGET_OUT_OF_REACH;
        } 

    }
    if( msg_id != INFO_NONE ){
        debuglog( info_messages[msg_id] );
    }
}
void set_jog_axis_x_positive( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.x.index );
}
void set_jog_axis_x_negative( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.x.index, true );
}
void set_jog_axis_y_positive( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.y.index );
}
void set_jog_axis_y_negative( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.y.index, true );
}
void set_jog_axis_z_positive( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.z.index );
}
void set_jog_axis_z_negative( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.z.index, true );
}
void set_jog_axis_u_positive( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.u.index );
}
void set_jog_axis_u_negative( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.u.index, true );
}
void set_jog_axis_v_positive( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.v.index );
}
void set_jog_axis_v_negative( setter_param * item ){
    set_jog_axis( char_to_float( item->second ), axes_active.v.index, true );
}



void set_probing_action( setter_param * item ){
    logger_show_on_line.store( true );
    info_codes msg_id = INFO_NONE;
    if( ui_controller.motion_input_is_blocked() ){
        msg_id = MOTION_NOT_AVAILABLE;
    } else if( !api::machine_is_fully_homed() ){
        msg_id = NOT_FULLY_HOMED;
    } else {
        debuglog("");
        debuglog("", DEBUG_LOG_DELAY );
        int page = PAGE_PROBING;
        ui.emit(SHOW_PAGE,&page);
        // 0 = left back corner; 1 = back; 2 = right back corner; 3 = left; 4 = center finder; 5 = right; 6 = left front corner; 7 = front; 8 = right front corner;
        int probe_param = char_to_int( item->second );
        //bool probe_2d   = true;// enforce 2d mode ui_controller.probe_dimension==1?true:false;
        std::function<void()> callback              = nullptr;
        static const int      block_size            = 6;
        std::string           str_block[block_size] = {};
        switch( probe_param ){
            case 0:
                str_block[0] = "@Probing left back edge";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::left_back_edge_2d, &probing);
                break;
            case 1:
                str_block[0] = "@Probing back side";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::probe_y_negative, &probing);
                break;
            case 2:
                str_block[0] = "@Probing right back edge";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::right_back_edge_2d, &probing);
                break;
            case 3:
                str_block[0] = "@Probing left side";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::probe_x_positive, &probing);
                break;
            case 4:
                str_block[0] = "@Center finder";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::center_finder_2d, &probing);
                break;
            case 5:
                str_block[0] = "@Probing right side";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::probe_x_negative, &probing);
                break;
            case 6:
                str_block[0] = "@Probing left front edge";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::left_front_edge_2d, &probing);
                break;
            case 7:
                str_block[0] = "@Probing front side";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::probe_y_positive, &probing);
                break;
            case 8:
                str_block[0] = "@Probing right front edge";
                callback = std::bind(&G_EDM_PROBE_ROUTINES::right_front_edge_2d, &probing);
                break;
        }
        str_block[1] = " Display will not respond";
        str_block[2] = " until probing is finished";
        str_block[3] = " The process can be stopped";
        str_block[4] = " using the motionswitch";
        str_block[5] = " Tool diameter: ";
        str_block[5] += floatToString( settings.get_setting_float( PARAM_ID_TOOLDIAMETER ) );
        for( int i = 0; i < block_size; ++i ){
          if( str_block[i].length()<=0 ){
            continue;
          }
          debuglog(str_block[i].c_str(), DEBUG_LOG_DELAY );
        }
        if( callback != nullptr ){
            ui_controller.probe_prepare();
            callback();
            ui_controller.probe_done();
            debuglog(" Press close to return", 100 );
        }

    }
    if( msg_id != INFO_NONE ){
        debuglog( info_messages[msg_id] );
    }
    logger_show_on_line.store( false );
}





void get_probing_action( getter_param * item ){
}
void get_probe_pos( getter_param * item ){
    item->floatValue = api::probe_pos_to_mpos(item->intValue);
    item->boolValue  = sys_probed_axis[item->intValue];
}

void get_home_axis( getter_param * item ){
    item->boolValue = g_axis[ item->intValue ]->is_homed.get();
}
void get_homing_enabled( getter_param * item ){
    if( item->intValue >= N_AXIS ){ 
        item->boolValue = false;
        return;
    }
    item->boolValue = g_axis[item->intValue]->homing_seq_enabled.get();
}


// set axis steps per mm; int value
void set_motor_res( setter_param * item ){
    uint8_t axis_index = 0;
    bool axis_enabled = false;
    switch( item->first ){
        case PARAM_ID_X_RES:
            axis_enabled = axes_active.x.enabled;
            axis_index   = axes_active.x.index;
        break;
        case PARAM_ID_Y_RES:
            axis_enabled = axes_active.y.enabled;
            axis_index   = axes_active.y.index;
        break;
        case PARAM_ID_Z_RES:
            axis_enabled = axes_active.z.enabled;
            axis_index   = axes_active.z.index;
        break;
        case PARAM_ID_U_RES:
            axis_enabled = axes_active.u.enabled;
            axis_index   = axes_active.u.index;
        break;
        case PARAM_ID_V_RES:
            axis_enabled = axes_active.v.enabled;
            axis_index   = axes_active.v.index;
        break;
    }
    if( ! axis_enabled ){
        return;
    }
    g_axis[axis_index]->steps_per_mm.set( char_to_float( item->second ) );
    api::update_speeds();
}
void get_motor_res( getter_param * item ){
    uint8_t axis_index = 0;
    bool axis_enabled = false;
    switch( item->param_id ){
        case PARAM_ID_X_RES:
            axis_enabled = axes_active.x.enabled;
            axis_index   = axes_active.x.index;
        break;
        case PARAM_ID_Y_RES:
            axis_enabled = axes_active.y.enabled;
            axis_index   = axes_active.y.index;
        break;
        case PARAM_ID_Z_RES:
            axis_enabled = axes_active.z.enabled;
            axis_index   = axes_active.z.index;
        break;
        case PARAM_ID_U_RES:
            axis_enabled = axes_active.u.enabled;
            axis_index   = axes_active.u.index;
        break;
        case PARAM_ID_V_RES:
            axis_enabled = axes_active.v.enabled;
            axis_index   = axes_active.v.index;
        break;
    }
    if( !axis_enabled ){
        item->floatValue = 0.0f;
        return;
    }
    item->floatValue = g_axis[axis_index]->steps_per_mm.get();
}








// set restart esp
void set_esp_restart( setter_param * item ){
    Serial.end();
    set_machine_state( STATE_ESTOP );
    esp_task_wdt_init(1, true);
    esp_task_wdt_add(NULL);
    while(true);  // wait for watchdog timer to be triggered...
    ESP.restart();
    while( true ){ vTaskDelay(10); }
}


void set_stop_edm_process( setter_param * item ){
    int stop_reason = char_to_int( item->second );
    if( stop_reason == 0 ){
        // normal finish?
    } else {

    }
    settings.change_setting( PARAM_ID_PWM_STATE, 0 ); // turn arcgen off
}



void get_sink_dir( getter_param * item ){
    item->intValue = ui_controller.single_axis_mode_direction;
}
// set axis used for sinker mode
void set_sink_dir( setter_param * item ){ // basic incremental value. need to add incremental features to the settings class
  int direction = ui_controller.single_axis_mode_direction;
  if( ++direction > 4 ){
    direction = 0;
  }  
  ui_controller.change_sinker_axis_direction( direction );
}



void set_page_current( setter_param * item ){ // speedy access wanted.... active_page is checked heavily
    ui_controller.active_page = ( uint8_t ) char_to_int( item->second );
}






// below function require some more work due to communications; not today...
// need to load the values remotely first etc...
/*void get_mks_max_torque(  getter_param * item  ){
    item->intValue = wire_feeder.get( WIRE_TENSION_MOTOR, PARAM_ID_MKS_MAXT );
}
void set_mks_max_torque(  setter_param * item  ){
    wire_feeder.set( WIRE_TENSION_MOTOR, PARAM_ID_MKS_MAXT, char_to_int( item->second ) );
}
void get_mks_max_current(  getter_param * item  ){
    item->intValue = wire_feeder.get( WIRE_TENSION_MOTOR, PARAM_ID_MKS_MAXC );
}
void set_mks_max_current(  setter_param * item  ){
    wire_feeder.set( WIRE_TENSION_MOTOR, PARAM_ID_MKS_MAXC, char_to_int( item->second ) );
}
void set_jog_axis_mks_positive( setter_param * item ){
    wire_feeder.move( WIRE_TENSION_MOTOR, char_to_float( item->second ), false );
}
void set_jog_axis_mks_negative( setter_param * item ){
    wire_feeder.move( WIRE_TENSION_MOTOR, char_to_float( item->second ), true );
}

void set_jog_axis_wire_negative( setter_param * item ){
    wire_feeder.move( WIRE_PULL_MOTOR, char_to_float( item->second ), true );
}
void set_jog_axis_wire_positive( setter_param * item ){
    wire_feeder.move( WIRE_PULL_MOTOR, char_to_float( item->second ), false );
}*/











IRAM_ATTR void get_edm_pause( getter_param * item ){
    item->boolValue = gconf.edm_pause_motion ? true : false;
}
IRAM_ATTR void set_edm_pause( setter_param * item ){ // dirty
    bool new_state = char_to_bool( item->second );
    bool state_changed = gconf.edm_pause_motion == new_state ? false : true;
    gconf.edm_pause_motion = new_state;
    if( !state_changed ) return;
    if( gconf.edm_pause_motion ){
        int rounds = 0;
        while( !planner.get_is_paused() || ++rounds>2000 ){ // get_is_paused() returns an atomic flag but gconf.edm_pause_motion is a dirty flag used across threads without locks etc. Will make a better solution.
            gconf.edm_pause_motion = true;
            if( get_quit_motion( true ) ) break;
            vTaskDelay(10);
        }
      debuglog("@Process paused",100);
    } else {
      debuglog("@Process resumed",DEBUG_LOG_DELAY);
    }
    vTaskDelay(DEBUG_LOG_DELAY);
}










bool setting_change_notify_callback_default( setget_param_enum param_id, settings_container data ){

    switch( param_id ){

        case PARAM_ID_XYUV_JOG_COMBINED: 
            if( data.value == 1 ){
                debuglog("XU and YV jog combined");
            } else {
                debuglog("XUYV jog separated");
            }
        break;

        case PARAM_ID_RETRACT_S_F: 
        case PARAM_ID_RETRACT_H_F:
        case PARAM_ID_MAX_FEED:
        case PARAM_ID_SPEED_INITIAL:
        case PARAM_ID_SPEED_RAPID:
            api::update_speeds();
        break;

        default: break; 
    }

    return true;

}



void GUI::settings_init(){
    notify_callbacks[ SETTING_NOTIFY_DEFAULT ] = setting_change_notify_callback_default; // add default callback
    // while cleaning things up. Some of the below stuff will be put to where it belongs later
    settings.add( PARAM_ID_SINK_DIST,             SETTING_TYPE_FLOAT, 0, DEFAULT_SINKER_TRAVEL_MM,  0.0, 1000.0, SETTING_NOTIFY_NONE ); // the maximum travel into the workpiece; Note: EDM has electrode wear of x percent and setting this to 1mm doesn't produce a 1mm deep hole
    settings.add( PARAM_ID_XYUV_JOG_COMBINED,     SETTING_TYPE_BOOL, DEFAULT_XYUV_JOG_COMBINED?1:0, 0.0, 0.0, 0.0, SETTING_NOTIFY_DEFAULT );
    settings.add( PARAM_ID_TOOLDIAMETER,          SETTING_TYPE_FLOAT, 0, DEFAULT_TOOL_DIAMETER, 0.0, 0.0, SETTING_NOTIFY_NONE ); // 
    settings.add( PARAM_ID_SPEED_RAPID,           SETTING_TYPE_FLOAT, 0, DEFAULT_SPEED_RAPIDS,  0.0, 0.0, SETTING_NOTIFY_DEFAULT ); // 
    settings.add( PARAM_ID_SPEED_INITIAL,         SETTING_TYPE_FLOAT, 0, DEFAULT_SPEED_INITIAL, 0.0, 0.0, SETTING_NOTIFY_DEFAULT ); // 
    settings.add( PARAM_ID_MAX_FEED,              SETTING_TYPE_FLOAT, 0, DEFAULT_SPEED_MAXFEED, 0.0, 0.0, SETTING_NOTIFY_DEFAULT ); // 
    settings.add( PARAM_ID_RETRACT_S_F,           SETTING_TYPE_FLOAT, 0, DEFAULT_SPEED_RETR_S,  0.0, 0.0, SETTING_NOTIFY_DEFAULT ); // 
    settings.add( PARAM_ID_RETRACT_H_F,           SETTING_TYPE_FLOAT, 0, DEFAULT_SPEED_RETR_H,  0.0, 0.0, SETTING_NOTIFY_DEFAULT ); // 
    settings.add( PARAM_ID_SD_FILE_SET,           SETTING_TYPE_BOOL,  0, 0.0, 0.0, 0.0, SETTING_NOTIFY_NONE );
    //settings.add( PARAM_ID_EDM_PAUSE, SETTING_TYPE_BOOL, 0, 0.0, 0.0, 0.0, SETTING_NOTIFY_DEFAULT );
    settings.add( PARAM_ID_MODE, SETTING_TYPE_INT, DEFAULT_OPERATION_MODE, 0.0, ( float ) MODE_SINKER, ( float ) MODE_2D_WIRE, SETTING_NOTIFY_NONE ); // 
}











// Default function to get single parameters
void (*getter_table[TOTAL_PARAM_ITEMS])( getter_param *item ) = { nullptr };
// Default function to set single parameters
void (*setter_table[TOTAL_PARAM_ITEMS])( setter_param *item ) = { nullptr };

void GUI::setup_events( void ){

    debuglog( "Creating events", DEBUG_LOG_DELAY );

    //#######################################################################
    // Getter function table
    //#######################################################################
    getter_table[PARAM_ID_MPOSX]                 = get_mpos_x;
    getter_table[PARAM_ID_MPOSY]                 = get_mpos_y;
    getter_table[PARAM_ID_MPOSZ]                 = get_mpos_z;
    getter_table[PARAM_ID_MPOSU]                 = get_mpos_u;
    getter_table[PARAM_ID_MPOSV]                 = get_mpos_v;
    getter_table[PARAM_ID_X_RES]                 = get_motor_res;
    getter_table[PARAM_ID_Y_RES]                 = get_motor_res;
    getter_table[PARAM_ID_Z_RES]                 = get_motor_res;
    getter_table[PARAM_ID_U_RES]                 = get_motor_res;
    getter_table[PARAM_ID_V_RES]                 = get_motor_res;
    getter_table[PARAM_ID_SINK_DIR]              = get_sink_dir;
    getter_table[PARAM_ID_PROBE_ACTION]          = get_probing_action;
    getter_table[PARAM_ID_GET_PROBE_POS]         = get_probe_pos;
    getter_table[PARAM_ID_HOME_AXIS]             = get_home_axis;
    getter_table[PARAM_ID_HOME_ENA]              = get_homing_enabled;
    getter_table[PARAM_ID_HOME_SEQ]              = get_homing_enabled;
    getter_table[PARAM_ID_MOTION_STATE]          = get_motion_state;
    getter_table[PARAM_ID_WPOSX]                 = get_wpos_x;
    getter_table[PARAM_ID_WPOSY]                 = get_wpos_y;
    getter_table[PARAM_ID_WPOSZ]                 = get_wpos_z;
    getter_table[PARAM_ID_WPOSU]                 = get_wpos_u;
    getter_table[PARAM_ID_WPOSV]                 = get_wpos_v;
    getter_table[PARAM_ID_EDM_PAUSE]             = get_edm_pause;
    #ifdef ENABLE_SERVO42C_TORQUE_MOTOR
        getter_table[PARAM_ID_MKS_MAXT]              = get_mks_max_torque;
        getter_table[PARAM_ID_MKS_MAXC]              = get_mks_max_current;
    #endif

    //#######################################################################
    // Setter function table
    //#######################################################################
    setter_table[PARAM_ID_X_RES]                 = set_motor_res;
    setter_table[PARAM_ID_Y_RES]                 = set_motor_res;
    setter_table[PARAM_ID_Z_RES]                 = set_motor_res;
    setter_table[PARAM_ID_U_RES]                 = set_motor_res;
    setter_table[PARAM_ID_V_RES]                 = set_motor_res;
    setter_table[PARAM_ID_ESP_RESTART]           = set_esp_restart;
    setter_table[PARAM_ID_SINK_DIR]              = set_sink_dir;
    setter_table[PARAM_ID_PROBE_ACTION]          = set_probing_action;
    setter_table[PARAM_ID_SET_PROBE_POS]         = set_probing_pos_action;
    setter_table[PARAM_ID_UNSET_PROBE_POS]       = set_probing_pos_unset;
    setter_table[PARAM_ID_HOME_ALL]              = set_home_all;
    setter_table[PARAM_ID_HOME_AXIS]             = set_home_axis;
    setter_table[PARAM_ID_HOME_ENA]              = set_homing_enabled;
    setter_table[PARAM_ID_HOME_SEQ]              = set_home_axis;
    setter_table[PARAM_ID_MOVE_TO_ORIGIN]        = set_move_to_origin;
    // these jogs could be merged into a single function! #Todo!
    setter_table[PARAM_ID_XRIGHT]                = set_jog_axis_x_positive;
    setter_table[PARAM_ID_XLEFT]                 = set_jog_axis_x_negative;
    setter_table[PARAM_ID_YUP]                   = set_jog_axis_y_positive;
    setter_table[PARAM_ID_YDOWN]                 = set_jog_axis_y_negative;
    setter_table[PARAM_ID_ZUP]                   = set_jog_axis_z_positive;
    setter_table[PARAM_ID_ZDOWN]                 = set_jog_axis_z_negative;
    setter_table[PARAM_ID_URIGHT]                = set_jog_axis_u_positive;
    setter_table[PARAM_ID_ULEFT]                 = set_jog_axis_u_negative;
    setter_table[PARAM_ID_VUP]                   = set_jog_axis_v_positive;
    setter_table[PARAM_ID_VDOWN]                 = set_jog_axis_v_negative;
    //setter_table[PARAM_ID_SPINDLE_MOVE_UP]       = set_jog_axis_wire_positive;
    //setter_table[PARAM_ID_SPINDLE_MOVE_DOWN]     = set_jog_axis_wire_negative;
    setter_table[PARAM_ID_SET_PAGE]              = set_page_current;
    setter_table[PARAM_ID_SET_STOP_EDM]          = set_stop_edm_process;
    setter_table[PARAM_ID_EDM_PAUSE]             = set_edm_pause;
    #ifdef ENABLE_SERVO42C_TORQUE_MOTOR
        setter_table[PARAM_ID_MKS_MAXT]              = set_mks_max_torque;
        setter_table[PARAM_ID_MKS_MAXC]              = set_mks_max_current;
        setter_table[PARAM_ID_MKS_CW]                = set_jog_axis_mks_positive;
        setter_table[PARAM_ID_MKS_CCW]               = set_jog_axis_mks_negative;
    #endif


    //#######################################################################
    // Getter callback
    //#######################################################################
    const std::function<void(getter_param *item)> get_callback = [](getter_param *item){
        if( item->param_id < 0 || item->param_id >= TOTAL_PARAM_ITEMS ){
          return;
        }

        setget_param_enum param_id = ( setget_param_enum ) item->param_id;

        switch( settings.exists( param_id ) ){ // work in progress. new way to have all settings compact in one class object

            case SETTING_TYPE_FLOAT:
                item->floatValue = settings.get_setting_float( param_id );
                return;
            break;

            case SETTING_TYPE_BOOL:
            //case SETTING_TYPE_BOOL_FLIP:
                item->boolValue = settings.get_setting_bool( param_id );
                return;
            break;

            case SETTING_TYPE_INT:
                item->intValue = settings.get_setting_int( param_id );
                return;
            break;

            default: // no setting entry found; fallback to lookup table

                if( getter_table[item->param_id] != nullptr ){
                    getter_table[item->param_id]( item );
                }

            break;

        }

    };

    //#######################################################################
    // Setter callback
    //#######################################################################
    const std::function<void(setter_param *item)> set_callback = [](setter_param *item){

        if( item->first < 0 || item->first >= TOTAL_PARAM_ITEMS ){
          return;
        }
        arcgen.lock(); // this is called where the timer should not fill the i2s read queue if pwm is enabled; sampling will not process i2s buffers if flag is set

        setget_param_enum param_id = ( setget_param_enum ) item->first;

        switch( settings.exists( param_id ) ){ // work in progress. new way to have all settings compact in one class object

            case SETTING_TYPE_NONE: // no setting entry found; fallback to lookup table

                if( setter_table[item->first] != nullptr ){
                    setter_table[item->first]( item );
                }

            break;

            case SETTING_TYPE_BOOL: // should transmit the bool as 1/0 instead of char true/false...
                settings.change_setting( param_id, char_to_bool( item->second ) ? 1 : 0, char_to_float( item->second ) );
            break;

            default:
                settings.change_setting( param_id, char_to_int( item->second ), char_to_float( item->second ) );
            break;

        }

        arcgen.unlock();

    };

    //#######################################################################
    // Bind callbacks
    //#######################################################################
    ui_controller.on(PARAM_GET,get_callback);
    ui_controller.on(PARAM_SET,set_callback);

}


