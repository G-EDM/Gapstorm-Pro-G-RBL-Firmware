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


#ifndef GPROBE_LOCK
#define GPROBE_LOCK

#include "api.h"
#include <mutex>

#define MV_CMD1 "G91 G0"  // relative
#define MV_CMD2 "G90 G54" // absolute
#define MV_CMD3 "G91 G10 P1 L20"
#define MC_CMD4 "G90 G0" // absolute
#define MC_CMD5 "G91 G38.2"
#define MC_CMD6 "G91 G54 G0"
//#define MC_CMD7 "G91 G54 G21\r\n"


enum probe_var_indexes {
    PROBE_TOUCHED,
    PROBE_SUCCESS,
    PROBE_VARS_TOTAL
};


class G_EDM_PROBE_ROUTINES {

    private:

        std::mutex mtx;

        bool shared_probe_vars[ PROBE_VARS_TOTAL ];


    public:

        G_EDM_PROBE_ROUTINES( void );

        void set( probe_var_indexes var_index, bool state );
        bool get( probe_var_indexes var_index );
        bool begin( void );


        bool do_probe( int axis, bool probe_negative, bool pulloff );
        void set_probe( uint8_t axis, float pos );
        /** Default XYZ probing **/
        bool probe_x_negative( void );
        bool probe_x_positive( void );
        bool probe_y_negative( void );
        bool probe_y_positive( void );
        bool probe_z_negative( void );
        void probe_reset( void );

        bool probe_x( bool probe_negative, bool pulloff = true );
        bool probe_y( bool probe_negative, bool pulloff = true );
        bool probe_z( bool probe_negative );
        /** 3D Edge finding with Z included **/
        /*void left_back_edge_3d( void );
        void left_front_edge_3d( void );
        void right_back_edge_3d( void );
        void right_front_edge_3d( void );*/
        /** 2D edge finding without Z **/
        void left_back_edge_2d( void );
        void left_front_edge_2d( void );
        void right_back_edge_2d( void );
        void right_front_edge_2d( void );
        /** 2D center finder **/
        bool center_finder_2d( void );
        void  pulloff_axis( int axis, bool probe_negative );
        void  probe_axis( int axis, bool probe_negative );

};


extern G_EDM_PROBE_ROUTINES probing;


#endif