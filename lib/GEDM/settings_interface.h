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

//############################################################################################################
/*

Not meant for high speed operation but a safe storage where settings are stored and can be distributed

*/
//############################################################################################################


#ifndef SETTINGS_INTERFACE_H
#define SETTINGS_INTERFACE_H

#include "widgets/language/en_us.h"

//#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <HardwareSerial.h>





enum setting_notify_map {
    SETTING_NOTIFY_NONE    = 0,
    SETTING_NOTIFY_ARCGEN  = 1,
    SETTING_NOTIFY_SENSOR  = 2,
    SETTING_NOTIFY_WIRE    = 3,
    SETTING_NOTIFY_PLANNER = 4,
    SETTING_NOTIFY_DPM     = 5,
    SETTING_NOTIFY_DEFAULT = 6,
    SETTING_NOTIFY_TOTAL
};

enum setting_types_enum {
    SETTING_TYPE_INT       = 0,
    SETTING_TYPE_FLOAT     = 1,
    SETTING_TYPE_BOOL      = 2,
    SETTING_TYPE_BOOL_FLIP = 3, // same as bool bit doesn't require a defined value. It just flips from 0 to 1 or 1 to 0 no matter the input
    SETTING_TYPE_NONE      = 4
};

typedef struct settings_container {
    int   value     = 0;
    float fvalue    = 0.0f;
    float range_min = 0.0f; // using floats for the range even if it is an int value 
    float range_max = 0.0f; // using floats for the range even if it is an int value 
    setting_notify_map inform_to = SETTING_NOTIFY_NONE;
    setting_notify_map load_cb   = SETTING_NOTIFY_NONE;
    setting_types_enum type      = SETTING_TYPE_INT;
} settings_container;


// not adding single callbacks for every setting
// just a gatewaay to the specific section (planner, sensor, wire module etc. and then use a switch inside those)
// this reduces the number of callbacks and normally there is not much to do. Fair tradeoff
extern bool ( * notify_callbacks[ SETTING_NOTIFY_TOTAL ] )( setget_param_enum param_id, settings_container  data ); // called after a setting changed
extern bool ( * load_callbacks[   SETTING_NOTIFY_TOTAL ] )( setget_param_enum param_id, settings_container *data ); // called before a setting is loaded


class SETTINGS_INTERFACE {

    private:

        std::mutex mtx;
        std::unordered_map< setget_param_enum, settings_container > __map;
            
        bool notify( setget_param_enum param_id, settings_container data ){

            if( data.inform_to == SETTING_NOTIFY_NONE || !notify_callbacks[ data.inform_to ] ) return true;

            return notify_callbacks[ data.inform_to ]( param_id, data );


            

        }
        
        


    public:

        SETTINGS_INTERFACE(){} // 
        ~SETTINGS_INTERFACE(){} // 

        setting_types_enum exists( setget_param_enum param_id ){
            std::lock_guard<std::mutex> lock( mtx );
            auto it = __map.find( param_id );
            return it != __map.end() ? it->second.type : SETTING_TYPE_NONE;
        }



        void add( 
            setget_param_enum param_id, 
            setting_types_enum param_type, 
            int int_value = 0, 
            float float_value = 0.0, 
            float range_min = 0.0, 
            float range_max = 0.0,
            setting_notify_map notify_to = SETTING_NOTIFY_NONE,
            setting_notify_map load_cb   = SETTING_NOTIFY_NONE
        ){
            settings_container ctr;
            ctr.type      = param_type;
            ctr.value     = int_value;
            ctr.fvalue    = float_value;
            ctr.range_min = range_min;
            ctr.range_max = range_max;
            ctr.inform_to = notify_to;
            ctr.load_cb   = load_cb;
            std::lock_guard<std::mutex> lock( mtx );
            __map[ param_id ] = ctr;
        }

        bool change_setting( setget_param_enum param_id, int int_value = 0, float float_value = 0.0 ){

            bool changed = false;
            settings_container data_copy;
            settings_container data_backup;

            {
                std::lock_guard<std::mutex> lock( mtx );
                auto it = __map.find( param_id );
                if( it == __map.end() ) return false;

                data_backup = it->second;
                                                
                switch( it->second.type ){

                    case SETTING_TYPE_BOOL:
                        if( it->second.value != int_value ){
                            it->second.value = int_value == 0 ? 0 : 1;
                            changed = true;
                        }
                    break;

                    /*case SETTING_TYPE_BOOL_FLIP:
                        it->second.value = !it->second.value;
                        changed = true;
                    break;*/

                    case SETTING_TYPE_FLOAT:
                        if( it->second.fvalue != float_value ){

                            if( it->second.range_max > 0.0 && float_value > it->second.range_max ){ // clamp to range
                                float_value = it->second.range_max;
                            } else if( it->second.range_min > 0.0 && float_value < it->second.range_min ){
                                float_value = it->second.range_min;
                            }
                            it->second.fvalue = float_value;
                            changed = true;
                        }
                    break;

                    default:
                        if( it->second.value != int_value ){
                            int min = ( int ) it->second.range_min;
                            int max = ( int ) it->second.range_max;
                            if( max > 0 && int_value > max ){ // clamp to range
                                int_value = max;
                            } else if( min > 0 && float_value < min ){
                                int_value = min;
                            }
                            it->second.value = int_value;
                            changed = true;
                        }
                    break;

                }

                data_copy = it->second;

            }
            




            if( changed ){
                if( !notify( param_id, data_copy ) ){
                    {
                        std::lock_guard<std::mutex> lock( mtx );
                        __map[ param_id ] = data_backup;
                    }
                };
                notify_change( param_id );
            }

            return true;

        }


        int get_setting_int( setget_param_enum param_id ){
            settings_container data_copy;
            {
                std::lock_guard<std::mutex> lock( mtx );
                auto it = __map.find( param_id );
                if( it == __map.end() ) return false;
                data_copy = it->second;
            }
            if( data_copy.load_cb != SETTING_NOTIFY_NONE ){
                // requires some perload action
                settings_container data;
                load_callbacks[ data_copy.load_cb ]( param_id, &data );
                if( data_copy.value != data.value ){
                    {
                        std::lock_guard<std::mutex> lock( mtx );
                        __map[ param_id ].value = data.value;
                    }
                }
                data_copy.value = data.value;
            }
            return data_copy.value;
        }



        bool get_setting_bool( setget_param_enum param_id){
            settings_container data_copy;
            {
                std::lock_guard<std::mutex> lock( mtx );
                auto it = __map.find( param_id );
                if( it == __map.end() ) return false;
                data_copy = it->second;
            }
            if( data_copy.load_cb != SETTING_NOTIFY_NONE ){
                // requires some perload action
                settings_container data;
                load_callbacks[ data_copy.load_cb ]( param_id, &data );
                if( data_copy.value != data.value ){
                    {
                        std::lock_guard<std::mutex> lock( mtx );
                        __map[ param_id ].value = data.value;
                    }
                }
                data_copy.value = data.value;
            }
            return ( data_copy.value == 0 ? false : true );
        }

        float get_setting_float( setget_param_enum param_id ){

            settings_container data_copy;
            {
                std::lock_guard<std::mutex> lock( mtx );
                auto it = __map.find( param_id );
                if( it == __map.end() ) return false;
                data_copy = it->second;
            }
            if( data_copy.load_cb != SETTING_NOTIFY_NONE ){
                // requires some perload action
                settings_container data;
                load_callbacks[ data_copy.load_cb ]( param_id, &data );
                if( data_copy.fvalue != data.fvalue ){
                    {
                        std::lock_guard<std::mutex> lock( mtx );
                        __map[ param_id ].fvalue = data.fvalue;
                    }
                }
                data_copy.fvalue = data.fvalue;
            }
            return data_copy.fvalue;

        }


        virtual void notify_change( setget_param_enum param_id ){} // sensor class overwrites... work in progress
};



extern SETTINGS_INTERFACE settings;


#endif