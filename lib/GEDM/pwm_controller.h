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

#ifndef ARC_GENERATOR_H
#define ARC_GENERATOR_H


#include "definitions.h"
#include <stdio.h>
#include <map>
#include <atomic>
#include "widgets/language/en_us.h"


//extern DRAM_ATTR std::atomic<bool> stop_sampling;

class ARC_GENERATOR
{

    private:




    public:

        ARC_GENERATOR(void);
        ~ARC_GENERATOR(void);

        void secure_on_off( bool turn_off, int delay );

        void IRAM_ATTR protection_off( void );
        void IRAM_ATTR protection_on( void );

        bool IRAM_ATTR get_pwm_is_off( void );


        void pwm_off( void );
        void pwm_on( void );
        void end( void );
        void start_ledc( void );
        void stop_ledc( void );
        void  create( void );
        bool  is_running(void);
        //int   get_freq(void);
        void  change_pwm_frequency(int freq_hz);
        //void  change_pwm_duty(int duty);
        void  change_pwm_duty( float duty_cycle_percent );
        void  disable_spark_generator(void);
        void  enable_spark_generator(void);
        void  probe_mode_on( void );
        void  probe_mode_off( void );

        void lock( void );
        void unlock( void );

};


extern ARC_GENERATOR arcgen;

#endif