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

#ifndef GSENSE_LOCK
#define GSENSE_LOCK


#include "definitions.h"
#include "api.h"
#include "widgets/language/en_us.h"
#include "settings_interface.h"
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/timer.h>
#include <esp32-hal-gpio.h>

// multisampler related
// vfd
#define V_VFD_FAST_SIZE       4
#define V_MULTISAMPLE_SIZE    32 // change below valud too if this is changed
#define V_MULTISAMPLE_SIZE_N1 31 // V_MULTISAMPLE_SIZE-1 precalculated for speed
// cfd
#define CSENSE_SAMPLER_BUFFER_SIZE         64 // change below value too if this is changed; buffer size needs to be power of two: 2,4,8,16,32,64,128,256...
#define CSENSE_SAMPLER_BUFFER_SIZE_N1      63 // CSENSE_SAMPLER_BUFFER_SIZE-1 precalculated for speed
#define CSENSE_PEAK_SAMPLER_BUFFER_SIZE    32 // change below value too if this is changed; buffer size needs to be power of two: 2,4,8,16,32,64,128,256...
#define CSENSE_PEAK_SAMPLER_BUFFER_SIZE_N1 31 // CSENSE_PEAK_SAMPLER_BUFFER_SIZE-1 for less in function math
#define CSENSE_PEAK_SAMPLER_BUFFER_SIZE_DIVIDER 5 // use for bitwise division on the moving average sum >> this_value 2=1 4=2 8=3 16=4 32=5 etc.. Needs to match CSENSE_PEAK_SAMPLER_BUFFER_SIZE


enum sense_queue_commands {
    SENSE_LOAD_SAMPLES = 0, // added to the sense queue by the timer interrupt for normal pulse data collection, only added if queue is empty to not polute it
    SENSE_BENCHMARK    = 1, // added to the sense queue to initiate kSps benchmarking
    SENSE_LOCK         = 2, // added to the sense queue to temporarily freeze the sampling while doing stuff like flushign retractions for example
    SENSE_UNLOCK       = 3, // added to the sense queue to return from freeze to normal operation after stuff is done
    SENSE_RESET        = 4  // reset sensors
    /*,SENSE_ENABLE_PROBING  = 5,
    SENSE_DISABLE_PROBING = 6,
    SENSE_SET_MODE_EDM    = 7,
    SENSE_SET_MODE_NORMAL = 8*/
};

enum REMOTE_CONTROL_COMMANDS {
    CMD_NONE               = 0,
    CMD_READ_ONOFF_SWITCH  = 1,
    CMD_READ_LIMITSWITCH   = 2,
    CMD_PWMOFF             = 4,
    CMD_PWMON              = 5,
    CMD_WIREFEEDER_STOP    = 6,
    CMD_WIREFEEDER_START   = 7,
    CMD_PROCESS_PAUSE      = 8,
    CMD_PROCESS_RESUME     = 9
};

enum motion_plans {
    MOTION_PLAN_ZERO       = 0,
    MOTION_PLAN_FORWARD    = 1,
    MOTION_PLAN_HOLD_SOFT  = 2,
    MOTION_PLAN_HOLD_HARD  = 3,
    MOTION_PLAN_SOFT_SHORT = 4,
    MOTION_PLAN_HARD_SHORT = 5,
    MOTION_PLAN_TIMEOUT    = 6
};

enum i2s_states {
    I2S_CTRL_IDLE = 0,
    I2S_RESTARTING,
    I2S_CTRL_NOT_AVAILABLE
};

extern DRAM_ATTR std::atomic<bool> motion_switch_changed;  // motion on/off state changed

extern xQueueHandle remote_control_queue;
extern xQueueHandle adc_read_trigger_queue; 

extern IRAM_ATTR int  get_calculated_motion_plan( bool enforce_fresh = false );
extern int   percentage_to_adc( float percentage );
extern float adc_to_percentage( int adc_value );
extern bool  sensors_task_running( void );


class G_SENSORS {

    private:

        int  sample_rate;
        int  buffer_count;

    public:

        G_SENSORS();
        ~G_SENSORS();

        IRAM_ATTR void wait_for_idle( bool aquire_lock = false );
        IRAM_ATTR void set_i2s_state( i2s_states state );
        IRAM_ATTR bool is_state( i2s_states state );

        void add_to_sense_queue( int operation );

        bool refresh_settings( void );
        void init_settings( void );
        void setup( void );
        void create_sensors( void );
        void sensor_end( void );
        void stop( void );
        void begin( void );
        void restart( void );
        void reset_sensor_global( void );
        int  set_sample_rate( int rate );

        static bool IRAM_ATTR motion_switch_read( void );
        static bool IRAM_ATTR unlock_motion_switch( void );
        static bool IRAM_ATTR limit_switch_read( void );

};

extern G_SENSORS gsense;

#endif