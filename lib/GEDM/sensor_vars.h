#pragma once

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

#ifndef GSENSE_VARS_LOCK
#define GSENSE_VARS_LOCK


#include "ili9341_tft.h"
#include "gpo_scope.h"
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
#include <soc/sens_struct.h>
#include <esp_attr.h>
#include <driver/i2s.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <hal/cpu_hal.h>

#define BENCH_TIMER_INTERVALL     4096 //2048; // needs to be a power of two; don't change without changing the bits to right shift; Power of two: (2,4,8,16,32,64,128,256,512,1024,2048,4096...)
#define BENCH_TIMER_BITS_TO_SHIFT 12   //11;   // Corresponds to the power of two from the comment above: (1,2,3,4,5,6,7,8,9,10,11,12...)



/*
    const int one_volt_to_adc   = VSENSE_RESOLUTION / 90.0; // rough
    const int two_volt_to_adc   = one_volt_to_adc * 2; // rough
    const int three_volt_to_adc = one_volt_to_adc * 3; // rough
*/


enum sensor_error_type {
    S_ERROR_NONE     = 0,
    S_ERROR_ARC      = 1,
    S_ERROR_OVERLOAD = 2,
    S_ERROR_SHORT    = 3
};

enum ism_sensor_states {
    SS_SEEK,
    SS_RAMP_UP,
    SS_ACTIVE,
    SS_CORRECTING,
    SS_ERROR
};

typedef struct deep_wave_var {
    bool feed_lock               = false;
    bool wave_control_enabled    = false;
    bool has_hs_ignition         = false;
    bool has_recent_ignition     = false;
    bool has_current_flow        = false;
    bool possible_pwm_off_sample = false;
    bool current_is_discharge    = false;

    uint32_t pwm_off_sample_wait     = 0;
    uint32_t possible_vfd_touch      = 0;
    uint32_t high_cfd_load_count     = 0;
    uint32_t feed_lock_unlock_count  = 0;
    uint32_t vfd_hs_alltime_max      = 0;
    uint32_t vfd_average_ideal       = 0;
    uint32_t vfd_average_setpoint    = 0;
    uint32_t vfd_peak_reference      = 0; // highest vfd sample in i2s batch in history
    uint32_t cfd_average_ideal_max   = 0;
    uint32_t cfd_average_alltime_max = 0;
    uint32_t cfd_fast_alltime_max    = 0;
    uint32_t cfd_recent_alltime_max  = 0;
    uint32_t cfd_average_ideal       = 0;
    uint32_t cfd_average_limit       = 0;
    uint32_t cfd_last_discharge      = 0;
    uint32_t feed_locked_at_cfd_avg  = 0;
    uint32_t feed_locked_at_cfd      = 0;
    uint32_t feed_locked_since       = 0;
    uint32_t feed_locked_cfd_peak    = 0;
    uint32_t short_circuit_count     = 0;
    uint32_t arcing_count            = 0;
    uint32_t overload_count          = 0;
    uint32_t no_spark_count          = 0;
    uint32_t spark_count             = 0;
    uint32_t cfd_stairs_step         = 0;
    uint32_t cfd_stairs_up           = 0;
    uint32_t cfd_stairs_spacing      = 0;
    uint32_t cfd_increasings         = 0;
    uint32_t cfd_flatline_count      = 0;
    uint32_t cfd_peak_reference      = 0; // this builds the conbstant reference for the max cfd peak on sparks
} deep_wave_var;

typedef struct external_sensors {
    bool on_off_switch_event_detected = false;
    bool limit_switch_event_detected  = false;
    bool block_on_off_switch          = false;
} external_sensors;

typedef struct feedback_misc_data {
    int32_t plan = 0;
} feedback_misc_data;

typedef struct adc_feedback {
    // cfd = current feedback
    uint32_t cfd_recent            = 0;
    uint32_t cfd_avg_fast          = 0;
    uint32_t cfd_avg_slow          = 0;
    uint32_t cfd_recent_previous   = 0;
    uint32_t cfd_avg_fast_previous = 0;
    uint32_t cfd_avg_slow_previous = 0;
    uint32_t cfd_peak              = 0;
    uint32_t cfd_peak_bottom       = 0;
    uint32_t cfd_hs_peak           = 0;
    // vfd = voltage feedback
    uint32_t vfd_recent            = 0;
    uint32_t vfd_avg_fast          = 0;
    uint32_t vfd_avg_slow          = 0;
    uint32_t vfd_avg_slow_max      = 0;
    uint32_t vfd_hs_peak           = 0;
    uint32_t vfd_peak              = 0;
    uint32_t vfd_peak_bottom       = 0;
} adc_feedback;

std::mutex              mtx;
std::condition_variable cv;

std::atomic<bool> sennsors_running( false );       // set to true after the task and queues are started (doesn't care if the tasks are already running)
std::atomic<bool> adc_monitor_task_running(false); // set to true once the adc monitor task enters the inner loop

DRAM_ATTR std::atomic<int>        motion_plan_atomic( 0 );                          // atomic int used to store/receive the motion plan
DRAM_ATTR std::atomic<bool>       new_motion_plan( false );                         // set to true if a new plan is available
DRAM_ATTR std::atomic<bool>       restart_i2s_flag( false );                        // if this flag is set to true the adc monitor task will restart i2s
DRAM_ATTR std::atomic<bool>       motion_switch_changed( false );                   // motion on/off state changed
DRAM_ATTR std::atomic<bool>       sense_settings_changed(false);
DRAM_ATTR std::atomic<i2s_states> i2s_state( I2S_CTRL_IDLE ); 


static esp_adc_cal_characteristics_t adc1_chars;
static uint16_t* dma_buffer              = nullptr;
static size_t    dma_buffer_current_size = 0;

hw_timer_t * benchmark_timer = NULL; 
TaskHandle_t adc_monitor_task_handle;    // I2S readout task with a waitqueue
TaskHandle_t remote_control_task_handle; // Remote control task with a waitqueue

xQueueHandle adc_read_trigger_queue = NULL; // Queue used to read a batch of i2s samples; it is filled from within a timerinterrupt
xQueueHandle remote_control_queue   = NULL; // Queue used for some misc stuff like limit/estop readings, pwm on/off controls etc.

static DRAM_ATTR feedback_misc_data adc_data;
static DRAM_ATTR adc_feedback       feedback;
static DRAM_ATTR deep_wave_var      deep_wave;
static DRAM_ATTR external_sensors   sensors; // used for the limits switch and motionswitch to hold the current state and flag onchange events
static DRAM_ATTR sensor_error_type  sensor_errors = S_ERROR_NONE;
static DRAM_ATTR ism_sensor_states  sensor_state  = SS_SEEK;

// cFd = Current feedback settings
static DRAM_ATTR uint32_t cfd_setpoint_mid;
static DRAM_ATTR uint32_t cfd_setpoint_min;
static DRAM_ATTR uint32_t cfd_setpoint_max;
static DRAM_ATTR uint32_t cfd_setpoint_probing;
static DRAM_ATTR uint32_t cfd_average_slow_size;
static DRAM_ATTR uint32_t cfd_average_fast_size;   
static DRAM_ATTR uint32_t cfd_ignition_treshhold; // value is in adc resolution. 12bit = 0-4095. cFd sample peaking above this is a spark
// vFd = Voltage settings
static DRAM_ATTR int vfd_short_circuit_threshhold; // vFd below this is considered a short circuit
// Other settings
static DRAM_ATTR int  i2s_buffer_length;
static DRAM_ATTR int  i2s_num_bytes;
static DRAM_ATTR bool scope_use_high_res;           // show each sample in the i2s batch on the scope for either cfd or vfd
static DRAM_ATTR bool scope_show_vfd_channel;       // show vfd samples instead of cfd

// Stuff for kSps benchmarking
static DRAM_ATTR uint32_t benchmark_ksps          = 0;
static DRAM_ATTR uint32_t benchmark_adc_counter   = 0;

static DRAM_ATTR int64_t  micros_now       = 0;
static DRAM_ATTR int64_t  ramp_cycle_start = 0;
static DRAM_ATTR int64_t  rampup_duration  = RAMP_DURATION;
static DRAM_ATTR int64_t  ramp_up_interval = 0;//rampup_duration / cfd_setpoint_min;
static DRAM_ATTR uint32_t ramp_up_setpoint = 0; // 
// Multisamplers for current (cfd) and voltage (vfd) feedbacks. The size needs to be a power of two due to bitwise operations
static DRAM_ATTR bool     fast_added                                       = false;
static DRAM_ATTR uint32_t loop_index                                       = 0;
static DRAM_ATTR uint32_t work_index                                       = 0;
static DRAM_ATTR uint32_t multisample_counts                               = 0;
static DRAM_ATTR uint32_t multisample_buffer[ CSENSE_SAMPLER_BUFFER_SIZE ] = {0,};
static DRAM_ATTR uint32_t v_multisample_counts                             = 0;
static DRAM_ATTR uint32_t v_multisample_buffer[ V_MULTISAMPLE_SIZE ]       = {0,};

// I2S shift out specific
static DRAM_ATTR bool     i2s_vfd_begin           = false;
static DRAM_ATTR uint32_t i2s_sum_cfd             = 0;
static DRAM_ATTR uint32_t i2s_sum_vfd             = 0;
static DRAM_ATTR uint32_t i2s_count_cfd           = 0;
static DRAM_ATTR uint32_t i2s_count_vfd           = 0;
static DRAM_ATTR uint32_t i2s_channel             = 0;
static DRAM_ATTR uint32_t i2s_sample              = 0;
static DRAM_ATTR uint32_t i2s_sample_previous     = 0;
static DRAM_ATTR uint32_t i2s_cfd_sample_previous = 0;
static DRAM_ATTR uint32_t i2s_sample_skip         = 0;

static DRAM_ATTR ism_sensor_states previous_state                = sensor_state;
static DRAM_ATTR uint32_t          release_confirmations         = 0; // it is a counter, it counts things
static DRAM_ATTR int8_t            work_plan                     = MOTION_PLAN_FORWARD;
static DRAM_ATTR bool              invert_plan                   = false;    
static DRAM_ATTR bool              first_seek_done               = false;
static DRAM_ATTR bool              feed_condition_met            = false;
static DRAM_ATTR bool              retract_condition_met         = false;
static DRAM_ATTR bool              retract_release_condition_met = false;
static DRAM_ATTR bool              loop_is_locked                = false;
static DRAM_ATTR bool              high_res_show_vfd             = false;
static DRAM_ATTR bool              high_res_show_cfd             = false;
static DRAM_ATTR bool              full_short_circuit            = false;
static DRAM_ATTR bool              water_contact_established     = false;  // vfd and cfd indicate no water contact at all
static DRAM_ATTR int               data                          = 1; // data variable passed to the wait queue





// Helper functions
int percentage_to_adc( float percentage ){ // convert percentage (0-100) into an ADC value
    if( percentage <= 0.0 ) return 0;
    if( percentage >= 100.0 ) return VSENSE_RESOLUTION;
    return round( ( float( VSENSE_RESOLUTION ) / 100.0 ) * percentage );
}

float adc_to_percentage( int adc_value ){ // convert an ADC value (0-VSENSE_RESOLUTION) into percentage
    if( adc_value <= 0 ) return 0.0;
    if( adc_value >= VSENSE_RESOLUTION ) return 100.0;
    return ( float( adc_value ) / float( VSENSE_RESOLUTION ) ) * 100.0;
}

// Create and return a DMA buffer of variable size
uint16_t* create_dma_buffer( size_t variable_length ){
    if( dma_buffer != nullptr ){ // seems to be more stable to always delete the buffer and create a new one
        heap_caps_free(dma_buffer);
        dma_buffer = nullptr;
    }
    if( dma_buffer == nullptr ) {
        if( variable_length > MAX_BUFFER_SIZE ) {
            return nullptr;
        }
        dma_buffer = (uint16_t*)heap_caps_malloc(variable_length * sizeof(uint16_t), MALLOC_CAP_DMA);
        if( dma_buffer == nullptr ) {
            return nullptr;
        }
        dma_buffer_current_size = variable_length;
    } 
    memset(dma_buffer, 0, variable_length * sizeof(uint16_t));
    return dma_buffer;
}

// Some sample rates are just not working and i2s fails to start. Not all the time the error is catchable
// this function does also not 100% ensure success but it reduces possible errors by adjusting the rate
// to be within a given tolerance
bool sample_rate_valid( int rate, int master_clock = I2S_MASTER_CLOCK_SPEED, int bits_per_sample = 16, int channels = 2 ) {
    double bclkFreq      = double( rate ) * bits_per_sample * channels;
    double ratio         = double( master_clock ) / bclkFreq;
    double ratio_rounded = std::round(ratio);
    double diff          = std::abs(ratio - ratio_rounded);
    double tolerance     = 0.2;
    return diff <= tolerance ? true : false;
}

// Returns true if the sensors task started (atomic flag)
bool sensors_task_running(){
    return adc_monitor_task_running.load();
}

// Flag limit event
void IRAM_ATTR limit_switch_on_interrupt() {
    if( 
        !gconf.gedm_disable_limits &&           // limits not disabled
        !sensors.limit_switch_event_detected && // prevent multiple calls
        (  
            get_machine_state() < STATE_ALARM && // machine state below alarm eg no active alarm state, no reboot etc
            !is_machine_state( STATE_HOMING )    // skip while in homing state
        )
    ){
        sensors.limit_switch_event_detected = true;        
        int data = CMD_READ_LIMITSWITCH;
        if( remote_control_queue != NULL ){ xQueueSendFromISR( remote_control_queue, &data, NULL ); }
    }
}

// Flag estop event
void IRAM_ATTR motion_switch_on_interrupt(){
    if( sensors.block_on_off_switch ){ return; }
    sensors.on_off_switch_event_detected = true;
    int data = CMD_READ_ONOFF_SWITCH;
    if( remote_control_queue != NULL ){ xQueueSendFromISR( remote_control_queue, &data, NULL ); }
}

// Notify the sense loop to create the ksps benchmark
IRAM_ATTR void bench_on_timer() {
    int data = SENSE_BENCHMARK;
    xQueueSendFromISR( adc_read_trigger_queue, &data, NULL );
}






void number_to_console( uint32_t __number ){
    const char *msg = int_to_char( __number );
    debuglog( msg );
    delete[] msg;
}







// Push samples to the scope ( the lock does nothing currently )
static IRAM_ATTR void adc_to_scope( uint16_t sample, int8_t plan ){

    if( 
        scope_batch_ready.load() || 
        !acquire_lock_for( ATOMIC_LOCK_GSCOPE, false ) // don't wait for lock if it is already taken
    ){ return; } // full batch waiting to get processed or lock not available

        /*int test = 150;
        if( feedback.vfd_peak_max > 150 ){
            test = feedback.vfd_peak_max - 150;
        }*/

        //int voltage = round( 90.0 / VSENSE_RESOLUTION * feedback.vfd_hs_peak );



        if( gscope.add_to_scope( sample, plan ) ){ // add sample to scope buffer and if batch is full add all the meta data
            gscope.set_meta_data(
                benchmark_ksps, 
                ( uint8_t ) plan, 
                feedback.vfd_avg_slow, 
                feedback.cfd_avg_fast, 
                feedback.cfd_avg_slow,
                ( int ) deep_wave.vfd_average_setpoint
                ,( int ) deep_wave.cfd_average_ideal
                //,( int ) deep_wave.cfd_stairs_up
                //( int ) deep_wave.cfd_average_alltime_max // 286
                //,( int ) deep_wave.cfd_fast_alltime_max   // 416
            );
        }

    release_lock_for( ATOMIC_LOCK_GSCOPE );

}


IRAM_ATTR void notify_motion_plan_instant( int8_t plan, uint16_t sample = 0, bool sample_to_scope = false ){ 
    motion_plan_atomic.store( plan ); // keep the old motion plan and notify the collector
    new_motion_plan.store(true);      // set flag that a new motionplan is available
    cv.notify_all();                  // notify signal
    if( sample_to_scope ){ // even if the samples are not used we can still show them on the scope
        adc_to_scope( sample, plan ); 
    }
}


//###########################################################################
// Called from planner on core 1 to request the current motionplan
//###########################################################################
//std::unique_lock<std::mutex> get_motion_plan_dummy_lock(mtx);
IRAM_ATTR int get_calculated_motion_plan( bool enforce_fresh ){
    int peeked_plan = motion_plan_atomic.load();
    if( 
        !gconf.edm_pause_motion && ( 
            enforce_fresh || peeked_plan < MOTION_PLAN_SOFT_SHORT
            //enforce_fresh || peeked_plan < MOTION_PLAN_HARD_SHORT
        )
    ){
        std::unique_lock<std::mutex> lock(mtx); // mutex needed for the function call only, not for the logic
        if( !cv.wait_for( lock, std::chrono::microseconds( 1000000 ), [] { return new_motion_plan.load(); } ) ){
            new_motion_plan.store( false );
            return MOTION_PLAN_TIMEOUT;
        }
    } 
    new_motion_plan.store( false );
    return motion_plan_atomic.load();
}












#endif