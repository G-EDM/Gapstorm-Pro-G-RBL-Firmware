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

/*
# Press the left DPM button for a few seconds to enter the menu
# then navigate through it and set it to this values:
#
# 5-CS = 0
# 6-bd = 115.2 // maximum baudrate the DPM supports; baudrate for ESP and DPM needs to be the same
# 4-Fd = 1
# 3-ON = 0 // output enabled at boot = 1; with 0 the output is disabled at start until it is manually turned on
# 7-ad 1
# 8 ch 1
*/


#include "definitions.h"

#include <HardwareSerial.h>
#include <esp_timer.h>
#include <map>

const int BUFFER_SIZE = 30;

enum dpm_msg {
    DPM_OK = 0,
    DPM_COM_FAIL,
    DPM_ENA_FAIL,
    DPM_ENA_FAIL_B,
    DPM_ENA_OK,
    DPM_DIS_OK,
    DPM_NO_SUPPORT,
    DPM_WAIT,
    DPM_PREPARE,
    DPM_TRY_ON,
    DPM_TRY_OFF
};

extern std::map<dpm_msg, const char*> dpm_messages;


class G_EDM_DPM_DRIVER {

    private:

        bool uart_support_enabled; // not used yet
        bool is_ready;
        char* error;
        HardwareSerial *dpm_serial;
        int64_t last_amp_reading_timestamp;
        bool IRAM_ATTR send( char* cmd, char* response, bool blocking = true );
        bool IRAM_ATTR fetch( char* response, bool blocking = true );
        int voltage_backup_mv;
        int current_backup_ma;
        float voltage_backup;
        float current_backup;

        void settings_init( void );



    public:
        G_EDM_DPM_DRIVER( bool __uart_support_enabled = DEFAULT_ENABLE_DPM );
        void end( void );
        bool get_error( void );
        bool setup( HardwareSerial &serial );
        bool init( void );
        bool power_on_off( int turn_on, int max_retry = 5 );
        int  IRAM_ATTR extract( int &value,  int multiplier, char* response );
        bool IRAM_ATTR extract_ok( char* response );
        int  set_voltage_and_current(float v, float c);
        bool set_setting_voltage(float value);
        bool set_setting_current(float value);
        int  backup_current_settings( void );
        int  restore_backup_settings( void );

        float read(char cmd);
        bool get_setting_voltage_and_current( int &voltage, int &current );
        bool get_setting_voltage( int &voltage );
        bool get_setting_current( int &current );
        bool get_power_is_on( void );//:01r12=0,

        bool change_support( bool value );
        bool support_enabled( void );
        bool initial_enable( void );
        float convert_ma_mv_to_a_v( int reading );

};

extern G_EDM_DPM_DRIVER dpm_driver;