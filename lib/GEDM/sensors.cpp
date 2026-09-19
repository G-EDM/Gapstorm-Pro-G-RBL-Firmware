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

screen  /dev/ttyUSB1 115200
App.ActiveDocument.Sketch2284.Shape.Length


        char command_buf[50];
        sprintf(command_buf,"%d CFDa:%d CFDr:%d VFDa:%d VFDr:%d", ( int ) sensor_errors, feedback.cfd_avg_slow, feedback.cfd_recent, feedback.vfd_avg_slow, feedback.vfd_recent );  
        Serial.println( command_buf );

*/
//############################################################################################################
#include "sensors.h"
#include "sensor_vars.h"

#define I2S_CFD_CHANNEL 6
#define I2S_VFD_CHANNEL 7



G_SENSORS gsense;


IRAM_ATTR FORCE_INLINE_ATTR void parse_i2s_batch(){
    
    i2s_sum_cfd = i2s_sum_vfd = i2s_count_cfd = i2s_count_vfd = i2s_sample_previous = i2s_cfd_sample_previous = i2s_vfd_begin =
    feedback.vfd_hs_peak = 
    feedback.cfd_hs_peak = 
    deep_wave.has_hs_ignition = 0;
    
    //for( loop_index = 0; loop_index < i2s_buffer_length; ++loop_index ){
    for( loop_index = 0; loop_index < i2s_buffer_length; loop_index+=4 ){ // at the default configuration the samples contain 8 duplicates each, no need to add the overhead for duplicates I guess
        i2s_channel = ( dma_buffer[ loop_index ] >> 12 ) & 0x07;  // extract channel
        i2s_sample  =   dma_buffer[ loop_index ]         & 0xFFF; // extract sample
        
        if( i2s_channel == I2S_CFD_CHANNEL ){

            if( i2s_sample > deep_wave.cfd_peak_reference ){ // keep track of how high a non stable discharge peaks
                deep_wave.cfd_peak_reference = i2s_sample;
            }

            if( 
                i2s_sample != i2s_cfd_sample_previous &&         // Count only one without the duplicate follow up samples
                i2s_sample + 100 >= deep_wave.cfd_peak_reference // compare with a little headroom
            ){ deep_wave.has_hs_ignition = true; }

            // CFD CHANNEL
            i2s_cfd_sample_previous = i2s_sample;
            i2s_sum_cfd += i2s_sample; // add sample to the cfd sum
            ++i2s_count_cfd;
            if( high_res_show_cfd ){ // Note: this only adds the current motionplan and not the one for this recent sample
                adc_to_scope( i2s_sample, adc_data.plan ); 
            }
        
        } else if( i2s_channel == I2S_VFD_CHANNEL ){
            
            // VFD PROXIMITY CHANNEL
            i2s_sample_skip = ( i2s_sample < VFD_JITTER || !i2s_vfd_begin || i2s_sample <= i2s_sample_previous) ? 1: 0;
            i2s_vfd_begin   = true; // just skip the first and go from there, first sample can be anything
            i2s_sample_previous = i2s_sample;
            if( i2s_sample_skip ) continue;
            if( i2s_sample > feedback.vfd_hs_peak ){
                feedback.vfd_hs_peak = i2s_sample;
            }

            if( i2s_sample > deep_wave.vfd_peak_reference ){
                deep_wave.vfd_peak_reference = i2s_sample;
            }

            i2s_sum_vfd += i2s_sample; // add sample to the vfd sum
            ++i2s_count_vfd;
            if( high_res_show_vfd ){ // Note: this only adds the current motionplan and not the one for this recent sample
                adc_to_scope( i2s_sample, adc_data.plan ); 
            }
        
        }
    
    }
    
    feedback.cfd_recent = i2s_sum_cfd > 0 ? int( i2s_sum_cfd / i2s_count_cfd ) : 0; 
    feedback.vfd_recent = i2s_sum_vfd > 0 ? int( i2s_sum_vfd / i2s_count_vfd ) : feedback.vfd_recent; // if no vfd was created due to skipping fallback to previous

    deep_wave.has_recent_ignition = feedback.cfd_recent > cfd_ignition_treshhold ? true : false;                       // Flag if cfd feedback peaked above the threshold used to determine if we had a discharge
    //deep_wave.has_hs_and_recent_ignition = ( deep_wave.has_recent_ignition && deep_wave.has_hs_ignition ) ? true : false;
    //deep_wave.has_none_ignition          = ( !deep_wave.has_recent_ignition && !deep_wave.has_hs_ignition ) ? true : false;


    if( deep_wave.pwm_off_sample_wait ){
        --deep_wave.pwm_off_sample_wait;
        if( feedback.cfd_recent <= 0 && feedback.cfd_avg_fast > ADC_JITTER ){ // that alone is not so safe, zero feedback can happen for other reason
            deep_wave.possible_pwm_off_sample = true;
        } else {
            deep_wave.possible_pwm_off_sample = false;
        }
    } else {
        deep_wave.possible_pwm_off_sample = false;
    }


}





IRAM_ATTR FORCE_INLINE_ATTR void vfd_averaging(){
    work_index                                 = v_multisample_counts;
    v_multisample_buffer[v_multisample_counts] = feedback.vfd_recent;                                // store new sample
    v_multisample_counts                       = (v_multisample_counts + 1) & V_MULTISAMPLE_SIZE_N1; // advance ring index
    feedback.vfd_avg_slow                      = feedback.vfd_avg_fast = 0;
    fast_added                                 = false;
    feedback.vfd_peak_bottom                   = VSENSE_RESOLUTION;
    for( loop_index = 0; loop_index < V_MULTISAMPLE_SIZE; ++loop_index ){
        i2s_sample = v_multisample_buffer[ work_index ];
        if( i2s_sample > feedback.vfd_peak ){ feedback.vfd_peak = i2s_sample; } // track the highest valued sample in the slow average
        if( i2s_sample < feedback.vfd_peak_bottom ){ feedback.vfd_peak_bottom = i2s_sample; }
        if( loop_index < V_VFD_FAST_SIZE ){ feedback.vfd_avg_fast += i2s_sample; } 
        else {
            if( !fast_added ){
                feedback.vfd_avg_slow = feedback.vfd_avg_fast;
                fast_added            = true;
            }
            feedback.vfd_avg_slow += i2s_sample; // slow cfd average
        }
        work_index = ( work_index - 1 ) & V_MULTISAMPLE_SIZE_N1; // move reverse
    }                    
    feedback.vfd_avg_slow >>= 5; // divide by 32
    feedback.vfd_avg_fast >>= 2; // divide by 4   
}

IRAM_ATTR FORCE_INLINE_ATTR void cfd_averaging(){
    work_index                             = multisample_counts;
    feedback.cfd_peak                      = feedback.cfd_recent;
    feedback.cfd_peak_bottom               = VSENSE_RESOLUTION;
    multisample_buffer[multisample_counts] = feedback.cfd_recent;                                      // store new sample
    multisample_counts                     = (multisample_counts + 1) & CSENSE_SAMPLER_BUFFER_SIZE_N1; // advance ring index
    feedback.cfd_avg_slow = feedback.cfd_avg_fast = deep_wave.cfd_stairs_step = 0;
    fast_added            = false;
    for( loop_index = 0; loop_index < cfd_average_slow_size; ++loop_index ){
        i2s_sample = multisample_buffer[ work_index ];
        if( i2s_sample > feedback.cfd_peak ){ feedback.cfd_peak = i2s_sample; } // track the highest valued sample in the slow average
        if( i2s_sample < feedback.cfd_peak_bottom ){ feedback.cfd_peak_bottom = i2s_sample; } // track the highest valued sample in the slow average
        if( loop_index < cfd_average_fast_size ){ // build the fast average
            feedback.cfd_avg_fast += i2s_sample; 
            if( loop_index > 0 && loop_index <= 3 && i2s_sample > deep_wave.cfd_stairs_step ){
                deep_wave.cfd_stairs_step = i2s_sample; // highest of the last x samples
            }
        }
        else { // build the slow average
            if( !fast_added ){ // start the slow average by setting the value to the fast averages value to reduce a little computation
                feedback.cfd_avg_slow = feedback.cfd_avg_fast;
                fast_added            = true;
            }
            feedback.cfd_avg_slow += i2s_sample; // slow cfd average
        }
        work_index = ( work_index - 1 ) & CSENSE_SAMPLER_BUFFER_SIZE_N1; // move reverse
    }
    feedback.cfd_avg_fast /= cfd_average_fast_size;
    feedback.cfd_avg_slow /= cfd_average_slow_size;
}
















// todo
//
// arcing recognition
// setpoint adjustments a little deeper to ensure max ones are safe
// overload management.. setpoint_max usage??



IRAM_ATTR FORCE_INLINE_ATTR void deep_wave_analytics_post_error(){ // function called after the error checks are done

     
}


IRAM_ATTR FORCE_INLINE_ATTR void deep_wave_analytics(){ // this has a huge impact on how many amps it draws max, called before error checks are done


    //##########################################################################################################
    // Keep track of the absolute max feedback readings
    //##########################################################################################################
    if( feedback.cfd_avg_slow > deep_wave.cfd_average_alltime_max ){
        deep_wave.cfd_average_alltime_max = feedback.cfd_avg_slow;
    }
    if( feedback.cfd_avg_fast > deep_wave.cfd_fast_alltime_max ){
        deep_wave.cfd_fast_alltime_max = feedback.cfd_avg_fast;
    }
    if( feedback.cfd_recent > deep_wave.cfd_recent_alltime_max ){
        deep_wave.cfd_recent_alltime_max = feedback.cfd_recent;
    }
    if( feedback.vfd_hs_peak > deep_wave.vfd_hs_alltime_max ){
        deep_wave.vfd_hs_alltime_max = feedback.vfd_hs_peak;
    }

    //##########################################################################################################
    // Flag that there is/was some current flowing within the slow average time frame
    //##########################################################################################################
    deep_wave.has_current_flow = ( feedback.cfd_peak >= cfd_setpoint_min || deep_wave.has_recent_ignition || deep_wave.has_hs_ignition );

    deep_wave.current_is_discharge = ( deep_wave.has_recent_ignition || deep_wave.has_hs_ignition );


    //##########################################################################################################
    // Check if deep_wave control is active
    //##########################################################################################################
    if( deep_wave.wave_control_enabled ){

        if( ++deep_wave.feed_locked_since > 20000 || !water_contact_established ){
            // forced release
            deep_wave.high_cfd_load_count =
            deep_wave.overload_count =
            deep_wave.arcing_count =
            deep_wave.short_circuit_count =
            deep_wave.wave_control_enabled = 
            deep_wave.feed_lock = 
            deep_wave.feed_locked_since = 0;

        } else {
            // analyze the feedback and determine if the lock is released

            if( 
                !deep_wave.pwm_off_sample_wait &&
                feedback.cfd_avg_slow < deep_wave.feed_locked_at_cfd_avg && 
                deep_wave.cfd_flatline_count <= 0 && 
                deep_wave.no_spark_count > 0 && 
                feedback.cfd_recent < ( deep_wave.feed_locked_cfd_peak >> 3 ) && 
                !deep_wave.has_current_flow && 
                !deep_wave.possible_pwm_off_sample
            ){

                //##########################################################################################################
                // DeepWave release conditions met. Clean exit.                                    
                //##########################################################################################################
                if( 
                    !deep_wave.arcing_count <= 2 &&
                    //!deep_wave.arcing_count && 
                    !deep_wave.short_circuit_count 
                ){
                    // no errors, seems a good values
                    deep_wave.vfd_average_ideal = feedback.vfd_avg_slow; //deep_wave.vfd_average_setpoint; 
                    if( 
                        //!deep_wave.overload_count && 
                        feedback.cfd_avg_slow > deep_wave.cfd_average_ideal_max 
                    ){
                        deep_wave.cfd_average_ideal_max = feedback.cfd_avg_slow;
                    }
                    if( feedback.cfd_avg_slow > deep_wave.cfd_average_ideal ){
                        // rapid increase? 
                        //deep_wave.cfd_average_ideal = feedback.cfd_avg_slow;
                        ++deep_wave.cfd_average_ideal;
                    } else if( feedback.cfd_avg_slow < deep_wave.cfd_average_ideal && deep_wave.cfd_average_ideal > ADC_JITTER ){
                        --deep_wave.cfd_average_ideal;
                    }
                    if( deep_wave.cfd_average_ideal < deep_wave.cfd_average_ideal_max ){
                        deep_wave.cfd_average_ideal = deep_wave.cfd_average_ideal_max;
                    }
                }

                deep_wave.cfd_stairs_up =
                deep_wave.high_cfd_load_count =
                deep_wave.overload_count =
                deep_wave.arcing_count =
                deep_wave.short_circuit_count =
                deep_wave.wave_control_enabled = 
                deep_wave.feed_lock = 
                deep_wave.feed_locked_since = 0;

            } else {


                //##########################################################################################################
                // DeepWave active                                
                //##########################################################################################################

                // check if current keeps rising
                if( feedback.cfd_recent > deep_wave.feed_locked_cfd_peak ){
                    deep_wave.feed_locked_cfd_peak = feedback.cfd_recent; // update peak
                }

                //##########################################################################################################
                // Check if there was a very high discharge present in the average
                //##########################################################################################################
                if( feedback.cfd_peak + ADC_JITTER >= deep_wave.cfd_recent_alltime_max ){
                    ++deep_wave.high_cfd_load_count;
                } else {
                    deep_wave.high_cfd_load_count = 0;
                }

                //##########################################################################################################
                // Check if i2s batch max peak vfd is close to max history peak
                //##########################################################################################################
                if(
                    deep_wave.has_current_flow &&
                    feedback.vfd_hs_peak + ADC_JITTER >= deep_wave.vfd_hs_alltime_max
                ){
                    ++deep_wave.possible_vfd_touch;
                } else {
                    deep_wave.possible_vfd_touch = 0;
                }



                //##########################################################################################################
                //                                                                        
                //##########################################################################################################
                if( !deep_wave.current_is_discharge ){ // 
                    ++deep_wave.no_spark_count; // increase the no discharge counter
                    deep_wave.spark_count = 0;  // reset the discharge counter
                } else {
                    deep_wave.no_spark_count = 0;    // reset the no discharge counter
                    ++deep_wave.spark_count;         // increase the discharge counter
                    deep_wave.feed_locked_since = 0; // reset to prevent unlocking under load
                }


                //##########################################################################################################
                // Check if cfd flatlined
                // todo: needs some more checks.. Stabilized burn triggers this too
                //
                //              ------
                // ---     -----      ---------
                //    -----
                //##########################################################################################################
                if(
                    deep_wave.current_is_discharge
                    && ( feedback.cfd_recent < feedback.cfd_recent_previous + ADC_JITTER && feedback.cfd_recent + ADC_JITTER > feedback.cfd_recent_previous ) 
                ){ 
                    // flatlined
                    ++deep_wave.cfd_flatline_count;
                } else {
                    // not flatlined
                    deep_wave.cfd_flatline_count = 0;
                }
                
                if( feedback.cfd_recent < ( deep_wave.feed_locked_cfd_peak >> 4 ) ){ // ignore super low values for flatline
                    deep_wave.cfd_flatline_count = 0;
                }

                //##########################################################################################################
                // Check if cfd climbs up without break over multiple sparks
                // Todo: This only works for upward climbs in a row if each sample is larger then the previous 
                //
                //       | 
                //     | | 
                //   | | | 
                // | | | | 
                // | | | |  
                //##########################################################################################################
                if( deep_wave.has_current_flow && feedback.cfd_recent > feedback.cfd_recent_previous ){
                    ++deep_wave.cfd_increasings; 
                } else if( deep_wave.cfd_increasings > 0 ){
                    deep_wave.cfd_increasings = 0;
                }

                //##########################################################################################################
                // Check if cfd climbs up,
                // Stair like climp up pattern
                //
                //                           
                //           |
                //       |   |
                //   |   | | |
                // | | | | | |    
                //##########################################################################################################
                if( deep_wave.current_is_discharge ){
                    // store the discharge value if it is a valid spark for later
                    deep_wave.cfd_last_discharge = feedback.cfd_recent;
                    
                    if( feedback.cfd_recent > deep_wave.cfd_stairs_step + ADC_JITTER ){ // discharge cfd is higher then the last set stair step
                    //if( feedback.cfd_recent > deep_wave.cfd_stairs_step ){ // discharge cfd is higher then the last set stair step
                        ++deep_wave.cfd_stairs_up;   
                        deep_wave.cfd_stairs_spacing = 0;
                    } else if( ++deep_wave.cfd_stairs_spacing > 3 ){
                        deep_wave.cfd_stairs_up = 0;
                    }
                
                } else if( !deep_wave.possible_pwm_off_sample ) { 
                    deep_wave.cfd_stairs_up = 0;
                }

                //##########################################################################################################
                // deep_wave based feed lock and unlock                                                                 
                //##########################################################################################################
                /*if( // allow feed if below conditions are met while deep_wave is active
                    !deep_wave.pwm_off_sample_wait &&
                    !deep_wave.high_cfd_load_count &&
                    !deep_wave.possible_vfd_touch &&
                    !deep_wave.cfd_stairs_up &&
                    !deep_wave.has_hs_ignition &&
                    deep_wave.cfd_flatline_count <= 0 &&
                    ( 
                        ( deep_wave.no_spark_count > 4 && sensor_state == SS_ACTIVE ) ||
                        (
                            !deep_wave.cfd_increasings
                            && deep_wave.no_spark_count > 2                                                                                                 // some discharged missed
                            && feedback.cfd_avg_slow <= deep_wave.cfd_average_ideal + ADC_JITTER                                                            // average slow cfd feedback below or close to ideal average
                            && ( feedback.vfd_avg_slow < deep_wave.vfd_average_setpoint + 4 && feedback.vfd_avg_slow + 4 > deep_wave.vfd_average_setpoint ) // voltage very close to ideal
                        ) 

                    )

                ){
                    if( ++deep_wave.feed_lock_unlock_count > 1 ){
                        deep_wave.feed_lock = false;
                        deep_wave.feed_lock_unlock_count = 0;
                    } else {
                        deep_wave.feed_lock = true;
                    }
                } else {
                    deep_wave.feed_lock = true;
                }*/






            }

        }

    } else {

        // feed not locked yet. If there is a discharge let's lock
        if( deep_wave.has_current_flow ){
            // no further feed allowed until wave recovered
            deep_wave.wave_control_enabled = deep_wave.feed_lock = true; 
            // set recent current cfd feedback for reference
            deep_wave.feed_locked_at_cfd = deep_wave.feed_locked_cfd_peak = feedback.cfd_recent;
            // set current slow cfd average for reference
            deep_wave.feed_locked_at_cfd_avg = feedback.cfd_avg_slow;
            // set current slow vfd average for reference
            //deep_wave.vfd_average_setpoint = deep_wave.vfd_average_ideal;
            deep_wave.vfd_average_setpoint = deep_wave.vfd_average_ideal > vfd_short_circuit_threshhold ? deep_wave.vfd_average_ideal : feedback.vfd_avg_slow;
            // reset
            deep_wave.cfd_stairs_up =
            deep_wave.high_cfd_load_count =
            deep_wave.overload_count =
            deep_wave.arcing_count =
            deep_wave.short_circuit_count =
            deep_wave.cfd_flatline_count =
            deep_wave.no_spark_count = 
            deep_wave.feed_locked_since = 
            deep_wave.feed_lock_unlock_count =
            deep_wave.cfd_increasings = 0;
        }

    }







}





IRAM_ATTR FORCE_INLINE_ATTR void state_decision(){
            
    retract_condition_met = (       // Conditions to enter a retraction
        sensor_errors > S_ERROR_ARC // arcs don't get retractions; only some pwm off required to shut off arcs
    ) ? true : false;
    
    retract_release_condition_met = ( // Conditions to release from a retaction state 
        sensor_errors == S_ERROR_NONE &&
        deep_wave.cfd_flatline_count <= 0 &&

        //feedback.vfd_peak_bottom > vfd_short_circuit_threshhold &&

        (
            feedback.cfd_avg_slow <= ADC_JITTER || (
                // cfd conditions
                feedback.cfd_avg_fast <= feedback.cfd_avg_slow
                && feedback.cfd_avg_slow <= feedback.cfd_avg_slow_previous
                && feedback.cfd_avg_slow <= cfd_setpoint_min
                // vfd conditions
            )
        )

        && (
            deep_wave.wave_control_enabled && (
                feedback.vfd_avg_slow + 50 >= deep_wave.vfd_average_setpoint
                && deep_wave.cfd_flatline_count <= 0
            )
        )

        && !deep_wave.possible_pwm_off_sample // pwm off can produce 0 cfd values while the short is still not gone. So if cfd is zero ensure it is a real 0 load


    ) ? true : false;
        



    if( !retract_release_condition_met ){
        release_confirmations = 0; // reset release counter
    }


    feed_condition_met = ( // Conditions to allow feed aka forward motions
        !water_contact_established || (

            !sensor_errors              // only if no errors
            && !deep_wave.possible_pwm_off_sample // only if sample is not a pwm off sample
            && !deep_wave.feed_lock     // only if deep wave is not locking the feed

            && feedback.cfd_avg_slow <= cfd_setpoint_min                            // slow average below min setpoint
            && feedback.cfd_avg_slow <= feedback.cfd_avg_slow_previous + ADC_JITTER // slow average decreasing compared to previous slow average
            && feedback.cfd_avg_fast <= feedback.cfd_avg_fast_previous + ADC_JITTER // fast average decreasing compared to previous fast average
            && feedback.cfd_avg_fast <= feedback.cfd_avg_slow          + ADC_JITTER // fast average trending to current decrease compared against slow average
            && feedback.cfd_recent   <= feedback.cfd_recent_previous   + ADC_JITTER
            && feedback.cfd_peak_bottom <= cfd_setpoint_min
            //&& feedback.cfd_peak <= cfd_setpoint_min

        )
    ) ? true : false;


}




IRAM_ATTR FORCE_INLINE_ATTR void error_determination(){

    sensor_errors = S_ERROR_NONE; // set default
    
    if( deep_wave.has_current_flow ){ // at least some recent cfd load in the slow average to prevent false positives due to electrode distance
    //if( feedback.cfd_peak >= deep_wave.cfd_average_ideal_max || deep_wave.has_recent_ignition ){ // at least some recent cfd load in the slow average to prevent false positives due to electrode distance
        //##########################################################################################################
        // _  _ ____ ____    ____ ____ ___ _ _  _ ____    ____ _  _ ____ ____ ____ _  _ ___    ____ _    ____ _ _ _ 
        // |__| |__| [__     |__| |     |  | |  | |___    |    |  | |__/ |__/ |___ |\ |  |     |___ |    |  | | | | 
        // |  | |  | ___]    |  | |___  |  |  \/  |___    |___ |__| |  \ |  \ |___ | \|  |     |    |___ |__| |_|_| 
        //                                                                          
        //##########################################################################################################
        if( 
            //##########################################################################################################
            // ____ _  _ ____ ____ ___    ____ _ ____ ____ _  _ _ ___    ____ ____ _  _ ___  _ ___ _ ____ _  _ ____ 
            // [__  |__| |  | |__/  |     |    | |__/ |    |  | |  |     |    |  | |\ | |  \ |  |  | |  | |\ | [__  
            // ___] |  | |__| |  \  |     |___ | |  \ |___ |__| |  |     |___ |__| | \| |__/ |  |  | |__| | \| ___] 
            //
            // Full short circuits require pwm off and fast retraction
            //##########################################################################################################
            full_short_circuit 

        ){

            ++deep_wave.short_circuit_count;
            deep_wave.pwm_off_sample_wait = 100;
            sensor_errors = S_ERROR_SHORT;
            arcgen.protection_off();
            sensor_state       = SS_ERROR; // switch into error state instantly on hard short circuits
            motion_plan_atomic.store( MOTION_PLAN_HARD_SHORT ); // 
            new_motion_plan.store(true);
            cv.notify_all();
            delayMicroseconds( ARC_OFF_DEALY_HARD_SHORT );
            arcgen.protection_on();
            
        } else if( 
            //##########################################################################################################
            // ____ ____ ____ _ _  _ ____    ____ ____ _  _ ___  _ ___ _ ____ _  _ ____ 
            // |__| |__/ |    | |\ | | __    |    |  | |\ | |  \ |  |  | |  | |\ | [__  
            // |  | |  \ |___ | | \| |__]    |___ |__| | \| |__/ |  |  | |__| | \| ___] 
            //
            // Arcs require a short pwm off. They can happen all the time and are really bad. Not only for 
            // electrode wear and surface finish but also for the wire. They create an uninterrupted arc on a tiny spot
            // on the wire with high current flowing from a tiny spot. It gets hot really fast and the wire will either 
            // deform or snap. Needs to be captured and shut off. A few microseconds with pwm off will break the arc.
            //##########################################################################################################
            deep_wave.cfd_flatline_count > 6 || 
            deep_wave.cfd_stairs_up >= 3
        ){
            ++deep_wave.arcing_count;
            deep_wave.pwm_off_sample_wait = 50;
            sensor_errors = S_ERROR_ARC;
            arcgen.protection_off(); // always shut down the pulse on error
            delayMicroseconds( ARC_OFF_DEALY );
            arcgen.protection_on();
            //number_to_console( deep_wave.cfd_flatline_count );
        }

        if(
            //##########################################################################################################
            // ____ _  _ ____ ____ _    ____ ____ ___     ____ ____ _  _ ___  _ ___ _ ____ _  _ ____ 
            // |  | |  | |___ |__/ |    |  | |__| |  \    |    |  | |\ | |  \ |  |  | |  | |\ | [__  
            // |__|  \/  |___ |  \ |___ |__| |  | |__/    |___ |__| | \| |__/ |  |  | |__| | \| ___]     
            //                                               
            // Overload requires a soft retraction without turning pwm off.Just a little step back.
            //##########################################################################################################
            sensor_errors != S_ERROR_SHORT // not needed if already flagged as hard error
            && (
                deep_wave.cfd_increasings >= 4 ||
                deep_wave.cfd_stairs_up >= 3 ||
                deep_wave.possible_vfd_touch > 1 ||
                feedback.vfd_peak_bottom < vfd_short_circuit_threshhold ||
                (
                    deep_wave.cfd_average_ideal > ADC_JITTER &&
                    (
                        ( // if vfd is stable not going into overload
                            //feedback.vfd_recent   + 200 < deep_wave.vfd_average_setpoint || 
                            feedback.vfd_avg_fast + 200 < deep_wave.vfd_average_setpoint || 
                            feedback.vfd_avg_slow + 200 < deep_wave.vfd_average_setpoint 
                        ) && (
                            //feedback.cfd_avg_slow > ( deep_wave.cfd_average_ideal << 2 ) ||
                            feedback.cfd_avg_slow + 50 > deep_wave.cfd_average_alltime_max ||
                            feedback.cfd_avg_fast + 50 > deep_wave.cfd_fast_alltime_max
                        )
                    )
                )
                //|| feedback.cfd_avg_slow + ADC_JITTER >= deep_wave.cfd_average_alltime_max
                //feedback.cfd_avg_slow > ( deep_wave.cfd_average_ideal << 1 )
            )


        ){

            ++deep_wave.overload_count;
            deep_wave.pwm_off_sample_wait = 10;
            
            sensor_errors = MAX( sensor_errors, S_ERROR_OVERLOAD );


        } 
    
    } else { 
        //##########################################################################################################
        // _  _ ____    ____ ____ ____ _       ____ _  _ ____ ____ ____ _  _ ___    ____ _    ____ _ _ _ 
        // |\ | |  |    |__/ |___ |__| |       |    |  | |__/ |__/ |___ |\ |  |     |___ |    |  | | | | 
        // | \| |__|    |  \ |___ |  | |___    |___ |__| |  \ |  \ |___ | \|  |     |    |___ |__| |_|_| 
        //                                                                          
        //##########################################################################################################
    
    }

}












IRAM_ATTR void adc_monitor_task(void *parameter){ // everything used in the loops should be placed in IRAM or DRAM. Calls to external functions should be avoided and best is to have everything in this loop.

    G_SENSORS *__this = (G_SENSORS *)parameter; // pointer to the sensor object

    __this->init_settings();
    
    ramp_up_interval = rampup_duration / cfd_setpoint_min;

    size_t bytes_read;

    __this->begin();         // start with the default settings
    __this->wait_for_idle(); // idle state is set after it is ready

    adc_monitor_task_running.store( true ); // set flag that task is running
    restart_i2s_flag.store( true );         // flag for i2s restart on initial run


    for(;;){

        // Note about vfd: it is roughly 90v max range until the adc peaks out at VSENSE_RESOLUTION (4095)
        // 10 ADC equals very roughly 0.22v. Not exact science here going on and jitter can be higher etc.
        // 100 ADC 2.2v
        // 70V at pure off time sampling without any switching: 3185 ADC. Based on math with 90V range. Roughly matches what I see. I get around 3015 for 70v
        // The ideal sparking ADC i watch on the scope at 70V is around 2850 or something
        // difference of 165 which is roughly 3.6v
        // i assume 3v would be ideal: 136.5 ADC could be around 3v
        // off time sample - 136.5 (or 150..) 3020 - 150 = 2870... Pretty close to what i see on the adjustable setpoint
        //
        // create a setpoint based on percentage from the highest slow average...?
        // 3015 = 70v
        // 5% from 3015 is around 150
        // 3015-150=2865 which is very close to the theory above...
        // The firmware does not 100% know the current voltage the DPH is set to
        // users can adjust it on the DPH panel bypassing the firmware and to really know the voltage it would require a constant polling via the DPH interface
        // so a method that does not require to know the current set voltage is prefered
        

        xQueueReceive( adc_read_trigger_queue, &data, portMAX_DELAY );


        switch ( data ){

            // could use different types for probing, cutting, normal etc.... Would reduce a tiny bit of load but increase switch complexity.. Maybe worth testing.
            case SENSE_LOAD_SAMPLES: break; // early exit for the normal operation

            case SENSE_BENCHMARK: // this is a seperate timer running, skip the readout for this one
                // the reason this is part of the sensor loop and not done in the timer isr is threadsafety and speed
                // i2s is super picky in regards of threadsafety. If this loop increments the benchmark counter while another taks calculates with it
                // big issue and i2s can get bricked. Using an atomic counter is slower and therefore it is part of this loop and the timer isr just adds a different data value
                // to detect the type of task to perform here. Since the timer does not have any pause functions going on it needs to continue to prevent
                // any further processing while pwm is disabled. 
                if( benchmark_adc_counter > 0 ){ // just a little kSps benchmarking
                    benchmark_ksps        = ( uint32_t )( ( benchmark_adc_counter * i2s_buffer_length ) >> BENCH_TIMER_BITS_TO_SHIFT );
                    benchmark_adc_counter = 0;
                }
                continue;
                break;

            case SENSE_LOCK: // just set the flag; used for flushing moves to prevent entering error states while retracting etc.
                if( sensor_state > SS_ACTIVE ){
                    // if in error state reset to active to prevent locks after returning to normal operation
                    sensor_state = SS_ACTIVE;
                }
                loop_is_locked = true;
                continue;
                break;

            case SENSE_UNLOCK: // unset the flag and reset some things for smooth transitioning
                loop_is_locked           = false;
                deep_wave.no_spark_count = 0;
                continue;
                break;

            case SENSE_RESET: // least expected
                motion_plan_atomic.store( MOTION_PLAN_FORWARD );
                new_motion_plan.store( false );
                __this->refresh_settings();
                memset( &feedback,  0, sizeof( feedback ) );
                memset( &adc_data,  0, sizeof( adc_data ) );

                // reset some stuff on the deep_wave object

                //deep_wave.vfd_average_ideal = vfd_short_circuit_threshhold

                loop_is_locked            = false;
                previous_state            = SS_SEEK;      // default to seek
                sensor_state              = SS_SEEK;      // default to seek
                sensor_errors             = S_ERROR_NONE; // unset errors
                first_seek_done           = false;        // unset initial seek flag
                release_confirmations     = 0;
                ramp_up_interval          = rampup_duration / cfd_setpoint_min;
                invert_plan               = false;
                benchmark_adc_counter     = 0;
                high_res_show_vfd         = ( scope_use_high_res &&  scope_show_vfd_channel ) ? true : false;
                high_res_show_cfd         = ( scope_use_high_res && !scope_show_vfd_channel ) ? true : false;
                if( cfd_average_fast_size > cfd_average_slow_size ){ // just to be sure 
                    cfd_average_fast_size = cfd_average_slow_size; 
                }
                arcgen.protection_on();
                continue;
                break;

            default: break;

        }

        if( loop_is_locked ){ // just in case unlock the motion
            adc_data.plan = MOTION_PLAN_FORWARD;
            notify_motion_plan_instant( adc_data.plan );
            continue;
        }



        //##########################################################################################################
        //
        // ____  ____  ___  ____  _  _       ____  ___   ____       ____  ____  _  _  ___   _     ____  ____ 
        // |___  |___   |   |     |__|       |__|  |  \  |          [__   |__|  |\/|  |__]  |     |___  [__  
        // |     |___   |   |___  |  |       |  |  |__/  |___       ___]  |  |  |  |  |     |___  |___  ___] 
        // 
        // Get the i2s buffer with the precious samples
        //############################################################################################################
        //size_t bytes_read;
        bytes_read = 0;
        if( ESP_OK == i2s_read( I2S_NUM_0, dma_buffer, i2s_num_bytes, &bytes_read, I2S_TIMEOUT_TICKS ) && ( bytes_read >> 1 ) == i2s_buffer_length ){

                ++benchmark_adc_counter; // Increment the batch counter for the ksps benchmark
                //#################################################################################################
                // Process the I2S BUFFER and extract voltage / current feedback
                //#################################################################################################
                parse_i2s_batch();






                #ifdef TEST_PEAK_AVG
                
                    // Slow moving average for peak current size over a window of 32 samples
                    i2s_sample = cpeak_sampler[ cpeak_sampler_counts ]; // oldest sample to be removed
                    // Remove oldest sample from sum
                    if( feedback.cfd_avg_slow_peak_sum >= i2s_sample ) {
                        feedback.cfd_avg_slow_peak_sum -= i2s_sample;
                    } else {
                        feedback.cfd_avg_slow_peak_sum = 0; // Prevent underflow
                    }
                    // Add new sample
                    feedback.cfd_avg_slow_peak_sum += feedback.cfd_hs_peak;
                    // Compute average as sum divided by 32
                    feedback.cfd_avg_slow_peak = feedback.cfd_avg_slow_peak_sum >> CSENSE_PEAK_SAMPLER_BUFFER_SIZE_DIVIDER; // bitwise division
                    // Store new sample in buffer
                    cpeak_sampler[ cpeak_sampler_counts ] = feedback.cfd_hs_peak;
                    // Advance ring buffer index
                    cpeak_sampler_counts = ( cpeak_sampler_counts + 1 ) & CSENSE_PEAK_SAMPLER_BUFFER_SIZE_N1;
                    
                #endif





                //##########################################################################################################
                // I2S sample acquisition and parsing finished
                // Creating the motion plan ( Probing uses different logic )
                //##########################################################################################################
                if( is_machine_state( STATE_PROBING ) ){ // could use an internal flag created by submitting a given value to queue and remove the function call..

                    //##########################################################################################################
                    // ___   ____  ____  ___   _  _  _  ____ 
                    // |__]  |__/  |  |  |__]  |  |\ |  | __ 
                    // |     |  \  |__|  |__]  |  | \|  |__] 
                    // 
                    //############################################################################################################
                    arcgen.protection_on(); // just in case
                    work_plan = MOTION_PLAN_HOLD_SOFT; 
                    if( probing.get( PROBE_TOUCHED ) ){ // already triggered
                        __this->reset_sensor_global();
                        work_plan = MOTION_PLAN_SOFT_SHORT; 
                    } else {
                        if( feedback.cfd_recent >= cfd_setpoint_probing ){
                            work_plan = MOTION_PLAN_SOFT_SHORT; // probe confirmed
                            probing.set( PROBE_TOUCHED, true );
                            __this->reset_sensor_global();
                            if( feedback.cfd_recent >= cfd_setpoint_probing ){
                                debuglog("@Probe triggered by CFD Feedback");
                            }
                        } else {
                            probing.set( PROBE_TOUCHED, false );
                            work_plan = MOTION_PLAN_FORWARD;
                        }
                    }

                } else {

                    //##########################################################################################################
                    // ____  ___   _  _       ___   ____  ____  ____  ____  ____  ____ 
                    // |___  |  \  |\/|       |__]  |__/  |  |  |     |___  [__   [__  
                    // |___  |__/  |  |       |     |  \  |__|  |___  |___  ___]  ___]                                                                                                                         
                    // 
                    //############################################################################################################
                    work_plan = MOTION_PLAN_FORWARD; // Set default motion plan ( forward )

                    //##########################################################################################################
                    // ____  _  _  ____  ____  ___       ___   ____  ___  ____  ____  _  _  _  _  _  ____  ___  _  ____  _  _ 
                    // [__   |__|  |  |  |__/   |        |  \  |___   |   |___  |__/  |\/|  |  |\ |  |__|   |   |  |  |  |\ | 
                    // ___]  |  |  |__|  |  \   |        |__/  |___   |   |___  |  \  |  |  |  | \|  |  |   |   |  |__|  | \|                                                                                 
                    // 
                    // Initial real short circuit detection
                    // Note that there can still be false positives here that are not catchable at this position
                    // If the electrode is further away the voltage may drop. Still acceptable at this point
                    // but to finally enter into error state it will require a closer look at the current multisampler later
                    // Also after protection turned PWM off vfd will rise quick making this unreliable at this state
                    // But good enough to determine the first short and go from there
                    //############################################################################################################
                    water_contact_established = ( // this only checks if there is water contact at all, all feedbacks will be zero with maybe a tiny jitter only if no water contact is given
                        ( feedback.vfd_recent == 0 && feedback.cfd_recent == 0 ) // totally no load indicated here and 100% not even water contact
                        || ( 
                            feedback.vfd_avg_fast           < ADC_JITTER   // some vfd going on but very very low. Either electrode miles away or just jitter. Normally indicates no water contact. 
                            && feedback.cfd_recent          < ADC_JITTER   // after a discharge it is also possible that this is a zero reading, needs harder proof
                            && feedback.cfd_recent_previous < ADC_JITTER   // previous cfd above jitter 
                            && feedback.cfd_avg_fast        < ADC_JITTER ) // fast average above jitter
                    ) ? false : true;

                    full_short_circuit = ( // initial check for a true hard short circuit based on a voltage drop from the DPH switching after exceeding max current setting
                        water_contact_established && // water contact given
                        ( 
                            feedback.vfd_recent < vfd_short_circuit_threshhold 
                            //feedback.vfd_avg_fast < vfd_short_circuit_threshhold 
                        )
                    );


                    if( !deep_wave.possible_pwm_off_sample ){

                        //##########################################################################################################
                        // _  _  ____  ___      ____  _  _  ____  ____  ____  ____  _  _  _  ____ 
                        // |  |  |___  |  \     |__|  |  |  |___  |__/  |__|  | __  |  |\ |  | __ 
                        //  \/   |     |__/     |  |   \/   |___  |  \  |  |  |__]  |  | \|  |__] 
                        //     
                        //##########################################################################################################   
                        vfd_averaging();
                        //##########################################################################################################
                        // ____  ____  ___      ____  _  _  ____  ____  ____  ____  _  _  _  ____
                        // |     |___  |  \     |__|  |  |  |___  |__/  |__|  | __  |  |\ |  | __ 
                        // |___  |     |__/     |  |   \/   |___  |  \  |  |  |__]  |  | \|  |__]                                                                            
                        //                                                            
                        //##########################################################################################################
                        cfd_averaging();

                    }


                    //##########################################################################################################
                    // ____ _  _ ____ _    _   _ ____ _ ____ 
                    // |__| |\ | |__| |     \_/  [__  | [__  
                    // |  | | \| |  | |___   |   ___] | ___] 
                    //
                    // Not perfect. Could need some review. Especially for arcing detection. 
                    //##########################################################################################################
                    deep_wave_analytics();
                    error_determination();
                    deep_wave_analytics_post_error(); 
                    //##########################################################################################################
                    // ____  ___  ____  ___  ____       ___   ____  ____  _  ____  _  ____  _  _  ____ 
                    // [__    |   |__|   |   |___       |  \  |___  |     |  [__   |  |  |  |\ |  [__  
                    // ___]   |   |  |   |   |___       |__/  |___  |___  |  ___]  |  |__|  | \|  ___] 
                    //                                                            
                    // Feed/Hold/Retract/Release conditions
                    //##########################################################################################################
                    state_decision();


                    //###########################################################################
                    // Store recent as previous for next iteration
                    //###########################################################################
                    feedback.cfd_avg_slow_previous = feedback.cfd_avg_slow;
                    feedback.cfd_avg_fast_previous = feedback.cfd_avg_fast;
                    feedback.cfd_recent_previous   = feedback.cfd_recent;


                    //##########################################################################################################
                    // ____  ____  _  _  ____  ____  ____       ____  ___  ____  ___  ____  ____ 
                    // [__   |___  |\ |  [__   |  |  |__/       [__    |   |__|   |   |___  [__  
                    // ___]  |___  | \|  ___]  |__|  |  \       ___]   |   |  |   |   |___  ___] 
                    //
                    // State specific decisions
                    //##########################################################################################################
                    switch ( sensor_state ){ // after a pause state is set to idle again in the reset section

                        //##########################################################################################################
                        // Seeking contact, less forward blocking, faster speed etc. Once contact is made it will switch to ramping
                        //##########################################################################################################
                        case SS_SEEK: // only happens until the first contact is etablished; seek until first spark is detected the switch to ramp up; it should have a least one tiny discharge now
                            
                            previous_state = SS_SEEK;

                            if( 
                                deep_wave.has_hs_ignition ||
                                feedback.cfd_peak     > cfd_setpoint_min || 
                                feedback.cfd_recent   > cfd_setpoint_min || 
                                feedback.cfd_avg_fast > cfd_setpoint_min || 
                                feedback.cfd_avg_slow > cfd_setpoint_min 
                            ){
                                deep_wave.pwm_off_sample_wait = 10;
                                first_seek_done  = true;
                                ramp_cycle_start = esp_timer_get_time(); //
                                ramp_up_setpoint = 0;
                                sensor_state     = SS_RAMP_UP; // change state to ramp up
                                work_plan        = MOTION_PLAN_HOLD_HARD; // use hold hard to indicate touch for the planner; hold soft does the same motionwise but does not trigger the initial touch flag
                                for( int i = CSENSE_SAMPLER_BUFFER_SIZE; i >= 0; --i ){ // on contact forcefill the multisampler with a little above setpoint to get the averages up fast
                                    if( multisample_buffer[i] < cfd_setpoint_min + ADC_JITTER ){
                                        multisample_buffer[i] = cfd_setpoint_min + ADC_JITTER;
                                    }
                                }
                                multisample_buffer[ multisample_counts ] = feedback.cfd_recent;
                                invert_plan = true;
                            } 

                            break;

                        //##########################################################################################################
                        // Ramp up phase
                        //##########################################################################################################
                        case SS_RAMP_UP:

                            previous_state = SS_RAMP_UP;

                            if( retract_condition_met ){ 
                                deep_wave.pwm_off_sample_wait = 20;
                                work_plan    = MOTION_PLAN_SOFT_SHORT; // 
                                sensor_state = SS_CORRECTING;          // enter soft correction state
                            } else if( 
                                !feed_condition_met || 
                                feedback.cfd_peak     > ramp_up_setpoint ||
                                feedback.cfd_avg_fast > ramp_up_setpoint || 
                                feedback.cfd_avg_slow > ramp_up_setpoint ||
                                feedback.cfd_recent   > ramp_up_setpoint
                            ){ 
                                work_plan = MOTION_PLAN_HOLD_HARD; 
                                deep_wave.pwm_off_sample_wait = 10;
                            } 

                            //##########################################################################################################
                            // Ramp up adjustments and exit
                            //##########################################################################################################
                            micros_now = esp_timer_get_time(); // 
                            if( ramp_up_setpoint >= cfd_setpoint_min ){
                                // ramping done; change state and reduce ramping time for following ramps after short circuits
                                previous_state   = SS_ACTIVE;
                                sensor_state     = SS_ACTIVE;
                                ramp_up_interval = RAMP_DURATION_ACTIVE / cfd_setpoint_min; // reduce ramp time after the initial cold start ramp; it will reset to the intial ramp after pauses etc.
                            } else if( micros_now - ramp_cycle_start > ramp_up_interval ){
                                // ramp up setpoint
                                ++ramp_up_setpoint;
                                ramp_cycle_start = micros_now;
                            }

                            break;

                        //##########################################################################################################
                        // Active phase in normal operation
                        //##########################################################################################################
                        case SS_ACTIVE: // normal cutting phase; either move forward, hold or change state

                            previous_state = SS_ACTIVE;

                            if( retract_condition_met ){ 

                                sensor_state = SS_CORRECTING;          // enter soft correction state
                                work_plan    = MOTION_PLAN_SOFT_SHORT; // 

                            } else if( !feed_condition_met ){

                                work_plan = ( 
                                    feedback.cfd_avg_slow >= cfd_setpoint_mid || feedback.cfd_avg_fast > cfd_setpoint_mid 
                                ) ? MOTION_PLAN_HOLD_HARD : MOTION_PLAN_HOLD_SOFT;

                            } else if( deep_wave.no_spark_count > 400 ){

                                // switch to seek for faster moves
                                sensor_state = SS_SEEK;

                            }

                            break;


                        //###########################################################
                        // Retractions are evil and should be avoided at all cost if
                        // possible. Use them with care and exit retractions early
                        // They can make problems worse easily and will mess with
                        // the surface finish too
                        // release conditions are the same
                        //###########################################################
                        case SS_CORRECTING: // soft correction, non extensive single steps only without strict release conditions?
                            sensor_state                   = previous_state; 
                            deep_wave.no_spark_count       = 0;
                            work_plan = MOTION_PLAN_SOFT_SHORT; // 
                            break;
                        case SS_ERROR: // hard short circuit correction


                            if( retract_release_condition_met && ++release_confirmations > HARD_SHORT_RELEASE_CONFIMATIONS ){ 
                                sensor_state = first_seek_done ? SS_RAMP_UP : previous_state; 
                                deep_wave.no_spark_count       = 0;
                                deep_wave.pwm_off_sample_wait += 20;
                            }

                            work_plan = MOTION_PLAN_HARD_SHORT; // 
                            
                            break;

                        default:
                            break;

                    }
 
                }

                if( is_system_mode_edm() ){
                    
                    if( work_plan == MOTION_PLAN_FORWARD && adc_data.plan == MOTION_PLAN_FORWARD && sensor_state > SS_SEEK ){
                        work_plan = MOTION_PLAN_HOLD_SOFT; // skip every second forward
                    }
                    
                    if( adc_data.plan > work_plan && new_motion_plan.load() ){
                        // old plan was higher but not collected yet from the planner
                        // ensure at least one collection?
                        work_plan = adc_data.plan;
                    }

                }
            

                adc_data.plan = work_plan; // adc_data.plan is not allowed to be negative

                //###########################################################################
                // Distribute the motion plan
                //###########################################################################
                if( invert_plan ){                                                  // inverted plan is only for the planner to flag first contact
                    work_plan *= -1;                                                // only work_plan is allowed to be negative
                    if( motion_plan_atomic.load() < 0 && !new_motion_plan.load() ){ // previous plan was negative and already collected by the planner
                        invert_plan = false;                                        // unflag, planner is notified of contact
                    } 
                }
    
                motion_plan_atomic.store( work_plan );
                new_motion_plan.store(true);
                cv.notify_all();


                if( !scope_use_high_res ){
                    adc_to_scope( scope_show_vfd_channel ? feedback.vfd_recent : feedback.cfd_recent, adc_data.plan ); 
                }

            
        } 



    }
    vTaskDelete(NULL);
}
























G_SENSORS::~G_SENSORS(){}

// Constructor
G_SENSORS::G_SENSORS(){

    pinMode( ON_OFF_SWITCH_PIN,       INPUT_PULLDOWN ); // motion switch
    pinMode( STEPPERS_LIMIT_ALL_PIN,  INPUT_PULLDOWN ); // limit switches (one pin for all)
    adc1_config_width( ADC_WIDTH_12Bit );
    adc1_config_channel_atten( CURRENT_SENSE_CHANNEL, ADC_ATTEN_11db );
    adc1_config_channel_atten( VOLTAGE_SENSE_CHANNEL, ADC_ATTEN_11db ) ;
    adc1_get_raw( VOLTAGE_SENSE_CHANNEL ); 
    adc1_get_raw( CURRENT_SENSE_CHANNEL ); 
    esp_adc_cal_characterize( ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, &adc1_chars );
    pinMode( CURRENT_SENSE_PIN, INPUT );
    pinMode( VOLTAGE_SENSE_PIN, INPUT );

}

void G_SENSORS::setup(){
    vTaskDelay(DEBUG_LOG_DELAY);
    detachInterrupt( digitalPinToInterrupt( ON_OFF_SWITCH_PIN ) );
    detachInterrupt( digitalPinToInterrupt( STEPPERS_LIMIT_ALL_PIN ) );
    attachInterrupt( ON_OFF_SWITCH_PIN,      motion_switch_on_interrupt, CHANGE );
    attachInterrupt( STEPPERS_LIMIT_ALL_PIN, limit_switch_on_interrupt,  CHANGE );
}

void G_SENSORS::add_to_sense_queue( int operation ) {
    xQueueSendFromISR( adc_read_trigger_queue, &operation, NULL );
}

void G_SENSORS::sensor_end(){
    debuglog("Stopping sensors");
    if( ! sennsors_running.load() ){
        debuglog("Sensors not running",DEBUG_LOG_DELAY);
        return;
    }
    vTaskDelete( remote_control_task_handle );
    vTaskDelete( adc_monitor_task_handle );
    if( adc_read_trigger_queue != NULL ){ vQueueDelete( adc_read_trigger_queue ); adc_read_trigger_queue = NULL; }
    if( remote_control_queue != NULL ){ vQueueDelete( remote_control_queue ); remote_control_queue = NULL; }    
    vTaskDelay(DEBUG_LOG_DELAY);

    arcgen.lock();
    gsense.stop();
    arcgen.unlock();
}


// Set the sample rate in Hz
int G_SENSORS::set_sample_rate( int rate ){
    while( !sample_rate_valid( rate, I2S_MASTER_CLOCK_SPEED ) ){ ++rate; }
    if( rate > 1000000 ){ while( !sample_rate_valid( rate, I2S_MASTER_CLOCK_SPEED ) ){ --rate; } }
    return rate;
}

// Compare the current state against a wanted state
IRAM_ATTR bool G_SENSORS::is_state( i2s_states state ){
    return ( i2s_state.load() == state );
}

// Set the state (idle, budy etc)
IRAM_ATTR void G_SENSORS::set_i2s_state( i2s_states state ){
    i2s_state.store( state );
}

// Wait for I2S to finish what it does
IRAM_ATTR void G_SENSORS::wait_for_idle( bool aquire_lock ){
    while( !is_state( I2S_CTRL_IDLE ) ){
        vTaskDelay(1);
    }
}

// Restarts the sampling service
void G_SENSORS::restart(){
    // Restart only if i2s is running
    if( is_state( I2S_CTRL_NOT_AVAILABLE ) ){
        return;
    }
    // Set state to restart
    set_i2s_state( I2S_RESTARTING );
    //while( !gscope.scope_is_running() ){ vTaskDelay(200); }
    acquire_lock_for( ATOMIC_LOCK_GSCOPE );
    // Stop I2S and uninstall driver
    stop();
    // Install i2s driver and start it
    begin(); // state is now set to I2S_CTRL_IDLE by the reset function called inside begin()
    release_lock_for( ATOMIC_LOCK_GSCOPE );
}

// Stop I2S and uninstall driver
void G_SENSORS::stop(){
    if( is_state( I2S_CTRL_NOT_AVAILABLE ) ){ // wasn't ready
        return;
    }
    i2s_stop( I2S_NUM_0 );
    i2s_adc_disable(I2S_NUM_0);
    i2s_driver_uninstall( I2S_NUM_0 );
    set_i2s_state( I2S_CTRL_NOT_AVAILABLE );
}

// Starts the sampling service
void G_SENSORS::begin(){

    // Create a new DMA buffer if needed
    while( create_dma_buffer( i2s_buffer_length ) == nullptr ){
        vTaskDelay(DEBUG_LOG_DELAY);
    }

    periph_module_reset(PERIPH_I2S0_MODULE);
    // Some math can be done in advance
    i2s_num_bytes = sizeof(uint16_t) * i2s_buffer_length;
    // I2S core configuration
    i2s_config_t i2s_config; 
    i2s_config.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
    i2s_config.sample_rate          = set_sample_rate( sample_rate );  
    i2s_config.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT; 
    i2s_config.channel_format       = I2S_CHANNEL_FMT_ALL_LEFT;//I2S_CHANNEL_FMT_RIGHT_LEFT;//I2S_CHANNEL_FMT_ALL_LEFT; //I2S_CHANNEL_FMT_ONLY_LEFT,
    i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB);
    i2s_config.intr_alloc_flags     = (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_INTRDISABLED); //ESP_INTR_FLAG_LEVEL1,
    i2s_config.dma_buf_count        = buffer_count;
    i2s_config.dma_buf_len          = i2s_buffer_length;
    i2s_config.use_apll             = true;
    i2s_config.tx_desc_auto_clear   = true;
    i2s_config.fixed_mclk           = I2S_MASTER_CLOCK_SPEED;

    // install i2s driver
    while( ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) ){ 
        vTaskDelay(5); 
    }

    i2s_zero_dma_buffer( I2S_NUM_0 );
    // Setup some things and enable second channel
    i2s_set_adc_mode(ADC_UNIT_1, CURRENT_SENSE_CHANNEL);
    i2s_adc_enable(I2S_NUM_0);

    //SENS.sar_read_ctrl.sar1_sample_cycle = 2;
    //SENS.sar_read_ctrl.sar1_sample_num   = 2;

    
    // Scan multiple channels.
    SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 1, SYSCON_SARADC_SAR1_PATT_LEN_S);

    // This 32 bit register has 4 bytes for the first set of channels to scan.
    // Each byte consists of:
    // [7:4] Channel
    // [3:2] Bit Width; 3=12bit, 2=11bit, 1=10bit, 0=9bit
    // [1:0] Attenuation; 3=11dB, 2=6dB, 1=2.5dB, 0=0dB
    WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG, 0x7F6F0000);

    //SYSCON.saradc_ctrl.work_mode = 1;
    //SYSCON.saradc_ctrl.sar1_patt_len = 1; // 2 items 0,1 = 1,2
    //SYSCON.saradc_sar1_patt_tab[0]   = 0x7F6F0000;
    //SYSCON.saradc_ctrl2.meas_num_limit = 0;
    //SYSCON.saradc_ctrl.work_mode = 2;

    //delay( I2S_INIT_DELAY );
    vTaskDelay( I2S_INIT_DELAY );

    i2s_start(I2S_NUM_0);
    set_i2s_state( I2S_CTRL_IDLE );

    reset_sensor_global();

}

// Check the limit switch
bool IRAM_ATTR G_SENSORS::limit_switch_read(){
    vTaskDelay( 32 / portTICK_PERIOD_MS ); 
    bool state  = false;
    if ( !is_machine_state( STATE_HOMING ) && GRBL_LIMITS::limits_get_state() ){
        if( !gconf.gedm_disable_limits ){
            if( !is_machine_state( STATE_ESTOP ) ){ 
                set_alarm( ERROR_HARDLIMIT );
                set_machine_state( STATE_ESTOP );
                vTaskDelay(DEBUG_LOG_DELAY);
            }
        }
        state = true;
    } else {
        state = false;
    }
    new_motion_plan.store( true ); // early exit on waits
    sensors.limit_switch_event_detected = false;
    return state;
}

bool IRAM_ATTR G_SENSORS::unlock_motion_switch(){
    sensors.block_on_off_switch = false;
    motion_switch_read();
    motion_switch_changed.store( true );
    return true;
}

// Check the estop switch. Need to change this
// currently toggling the switch faster can create errors ( wrote this long ago. Not sure if it still is that way..)
// todo: Confirm the above comment...
IRAM_ATTR bool G_SENSORS::motion_switch_read(){
    vTaskDelay( 32 / portTICK_PERIOD_MS ); 
    bool state  = false;
    //int is_high = ( GPIO_REG_READ( GPIO_IN1_REG ) >> ( ON_OFF_SWITCH_PIN - 32 ) ) & 0x1;;
    if( digitalRead(ON_OFF_SWITCH_PIN) ){
        if( system_block_motion ){
            enforce_redraw.store( true );
            motion_switch_changed.store( true );
        }
        system_block_motion = false;
        state = true;
    } else {
        if( is_system_mode_edm() ){
            sensors.block_on_off_switch = true; // get some control over the shutdown and block reenabling motion until all is done
        }
        if( !system_block_motion ){
            enforce_redraw.store( true );
            motion_switch_changed.store( true );
        }
        system_block_motion = true;
        state = false;
    }
    new_motion_plan.store( true );
    sensors.on_off_switch_event_detected = false;
    return state;
}

// Set the flag for a sensor object reset
// Before processing the next i2s batch it will reset the sensor objects used to make decision
void G_SENSORS::reset_sensor_global(){ // called from planner on core1
    //reset_sensor_state.store( true );
    gsense.add_to_sense_queue( SENSE_RESET ); // just in case
    gsense.add_to_sense_queue( SENSE_UNLOCK ); // just in case
}

// Default event queue loop / remote control
IRAM_ATTR void remote_control_task(void *parameter){
    while( remote_control_queue == NULL ) vTaskDelay(10);

    // Timer interrupt for benchmarking the kSps
    benchmark_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(benchmark_timer, &bench_on_timer, true);
    timerAlarmWrite(benchmark_timer, BENCH_TIMER_INTERVALL*1000, true);
    timerAlarmEnable(benchmark_timer);

    int data = 0;
    for (;;){
        xQueueReceive( remote_control_queue, &data, portMAX_DELAY ); 
        arcgen.lock(); // feedback task starves other stuff if not blocked
        switch ( data ){
            case 1:  G_SENSORS::motion_switch_read(); break;
            case 2:  G_SENSORS::limit_switch_read();  break;
            case 4:  arcgen.pwm_off();                break; // this is just temporary pwm on/off toggling without changing the state
            case 5:  arcgen.pwm_on();                 break; // happens on core1 for pausing etc. Does not require to run a settings change
            case 6:  settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 0 ); break;
            case 7:  settings.change_setting( PARAM_ID_SPINDLE_ONOFF, 1 ); break;
            default: break;
        }
        data = 0;
        arcgen.unlock();
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}
















bool setting_change_notify_callback_sense( setget_param_enum param_id, settings_container data ){


    switch ( param_id ){ // i2s requires a restart to apply those changes
        case PARAM_ID_I2S_RATE:
        case PARAM_ID_I2S_BUFFER_L:
        case PARAM_ID_I2S_BUFFER_C:
            restart_i2s_flag.store( true ); // flag to restart i2s
        break;
        default: break;
    }

    // Not very efficient right now but if a settings changed
    // we just fully refresh all settings and perform all required calculations etc
    // a little overweight but secure
    sense_settings_changed.store( true ); // flag that a setting changed

    gsense.reset_sensor_global();



    //reset_sensor_state.store( true ); // set flag to enter deep check inside the sensor loop

    // the above flags will not force the sensor loop to enter deep check
    // in order for them to take effect the reset_sensor_state flag needs to be set to true
    // there are situations where this function is called multiple times in a row
    // to prevent heavy resetting overloads it uses a delayed timer to set the flag
    // if the timer is already running it will stop it and reset the delay 
    // so if for example a loop updates settings 10 times rapidly
    // the timer will reset 9 times and finally execute after the last one
    // in theory....
    // Seems using this timer interferes with the NVC writing. Can't tell for sure yet
    // but sometimes NVS gets bricked and I think it happens when this time interrupt fires 
    // while working with NVS. Error is not easy to repeat and I haven't seen it since 
    //trigger_deep_check_timer( 1000 ); // timeout in microseconds (us) 1000us = 1ms (millis)

    return true;



}



//###########################################################################
// Create the settings when starting the sensor class
//###########################################################################
void G_SENSORS::init_settings(){
    
    notify_callbacks[ SETTING_NOTIFY_SENSOR ] = setting_change_notify_callback_sense;

    float vsense_max  = ( float ) VSENSE_RESOLUTION;
    float cfd_avg_max = ( float ) CSENSE_SAMPLER_BUFFER_SIZE_N1;
    float s_rate      = ( float ) I2S_SAMPLE_RATE / 1000.0;

    settings.add( PARAM_ID_USE_HIGH_RES_SCOPE, SETTING_TYPE_BOOL,  DEFAULT_SCOPE_USE_HIGH_RES?1:0, 0.0, 0.0, 0.0,          SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_SCOPE_SHOW_VFD,     SETTING_TYPE_BOOL,  0,                              0.0, 0.0, 0.0,          SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_FAST_CFD_AVG_SIZE,  SETTING_TYPE_INT,   DEFAULT_CFD_AVERAGE_FAST,       0.0, 2.0, cfd_avg_max,  SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_SLOW_CFD_AVG_SIZE,  SETTING_TYPE_INT,   DEFAULT_CFD_AVERAGE_SLOW,       0.0, 4.0, cfd_avg_max,  SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_VDROP_THRESH,       SETTING_TYPE_INT,   DEFAULT_VFD_SHORT_THRESH,       0.0, 0.0, vsense_max,   SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_EDGE_THRESH,        SETTING_TYPE_INT,   DEFAULT_EDGE_THRESHOLD,         0.0, 0.0, vsense_max,   SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_I2S_BUFFER_L,       SETTING_TYPE_INT,   I2S_NUM_SAMPLES,                0.0, 8.0, 1000.0,       SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_I2S_BUFFER_C,       SETTING_TYPE_INT,   I2S_BUFF_COUNT,                 0.0, 2.0, 128.0,        SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_I2S_RATE,           SETTING_TYPE_FLOAT, 0, s_rate,                      1.0, 1000.0,            SETTING_NOTIFY_SENSOR );
    settings.add( PARAM_ID_PROBE_TR_C,         SETTING_TYPE_FLOAT, 0, DEFAULT_CFD_SETPOINT_PROBE,  0.2, 50.0,              SETTING_NOTIFY_SENSOR ); // percentage, needs to be corrected to adc only
    settings.add( PARAM_ID_SETMIN,             SETTING_TYPE_FLOAT, 0, DEFAULT_CFD_SETPOINT_MIN,    1.0, 100.0,             SETTING_NOTIFY_SENSOR ); // percentage, needs to be corrected to adc only
    settings.add( PARAM_ID_SETMAX,             SETTING_TYPE_FLOAT, 0, DEFAULT_CFD_SETPOINT_MAX,    3.0, 100.0,             SETTING_NOTIFY_SENSOR ); // percentage, needs to be corrected to adc only

    sense_settings_changed.store( true );
    restart_i2s_flag.store( false ); // i2s not started yet
    refresh_settings(); // copy the initial settings 

}


//###########################################################################
// Reload all settings on runtime and if needed restart i2s 
//###########################################################################
bool G_SENSORS::refresh_settings(){

    if( sense_settings_changed.load() ){

        sense_settings_changed.store( false ); // unset flag 
        // load the settings from the settings container


        // reset those deep_wave things only if settings changed
        // no need to calibrate things again if nothing changed
        deep_wave.cfd_average_ideal_max   = ADC_JITTER; 
        deep_wave.cfd_average_alltime_max = ( ADC_JITTER << 1 );
        deep_wave.cfd_fast_alltime_max    = ( ADC_JITTER << 1 );
        deep_wave.cfd_recent_alltime_max  = ( ADC_JITTER << 1 );
        deep_wave.vfd_peak_reference      = 0;
        deep_wave.cfd_peak_reference      = 0;
        deep_wave.vfd_hs_alltime_max      = vfd_short_circuit_threshhold;
        deep_wave.pwm_off_sample_wait     = 100;
        deep_wave.possible_pwm_off_sample = false;


        scope_use_high_res           = settings.get_setting_bool( PARAM_ID_USE_HIGH_RES_SCOPE );
        scope_show_vfd_channel       = settings.get_setting_bool( PARAM_ID_SCOPE_SHOW_VFD );
        cfd_setpoint_min             = percentage_to_adc( settings.get_setting_float( PARAM_ID_SETMIN ) );
        cfd_setpoint_max             = percentage_to_adc( settings.get_setting_float( PARAM_ID_SETMAX ) );
        cfd_setpoint_probing         = percentage_to_adc( settings.get_setting_float( PARAM_ID_PROBE_TR_C ) );
        cfd_average_fast_size        = settings.get_setting_int( PARAM_ID_FAST_CFD_AVG_SIZE );
        cfd_average_slow_size        = settings.get_setting_int( PARAM_ID_SLOW_CFD_AVG_SIZE );
        vfd_short_circuit_threshhold = settings.get_setting_int( PARAM_ID_VDROP_THRESH );
        cfd_ignition_treshhold       = settings.get_setting_int( PARAM_ID_EDGE_THRESH );
        i2s_buffer_length            = settings.get_setting_int( PARAM_ID_I2S_BUFFER_L );
        buffer_count                 = settings.get_setting_int( PARAM_ID_I2S_BUFFER_C );
        sample_rate                  = round( 1000.0 * settings.get_setting_float( PARAM_ID_I2S_RATE ) );

        // postprocessing some stuff
        i2s_num_bytes = sizeof(uint16_t) * i2s_buffer_length;


        if( cfd_setpoint_max <= cfd_setpoint_min ){
            cfd_setpoint_max = cfd_setpoint_min+50; // adc
        }

        settings.change_setting( PARAM_ID_SETMAX, 0, adc_to_percentage( cfd_setpoint_max ) );

        cfd_setpoint_mid = cfd_setpoint_min + ( ( cfd_setpoint_max - cfd_setpoint_min ) >> 1 );


        // data validation for some stuff that would be hard to make in the settings change function as some things depend on others

        if( cfd_average_slow_size < 4 ){
            cfd_average_slow_size = 4;
        }
        if( cfd_average_fast_size >= cfd_average_slow_size ){
            cfd_average_fast_size = cfd_average_slow_size-1; // there are bottom limits set on the range clamp and slow-1 is safe
        }


    }

    if( restart_i2s_flag.load() ){ // settings are fresh now; check for i2s restart requirenments
        restart_i2s_flag.store( false );
        vTaskDelay(40);
        restart();
        vTaskDelay(40); 
        return true;
    }

    return false;
}


void G_SENSORS::create_sensors(){
    debuglog("Creating sensors", DEBUG_LOG_DELAY );
    unlock_motion_switch();
    debuglog("Starting sensor queues", DEBUG_LOG_DELAY ); //create_queues();
    int max_rounds = 10;
    while( adc_read_trigger_queue == NULL && --max_rounds > 0 ){ adc_read_trigger_queue = xQueueCreate( 10,  sizeof(int) ); vTaskDelay(5); }
    max_rounds = 10;
    while( remote_control_queue == NULL && --max_rounds > 0 ){ remote_control_queue  = xQueueCreate( 20,  sizeof(int) ); vTaskDelay(5); }
    debuglog("Starting sensor tasks", DEBUG_LOG_DELAY );//create_tasks();
    xTaskCreatePinnedToCore( remote_control_task, "remote_control_task_handle", STACK_SIZE_A, this, 1,                             &remote_control_task_handle, I2S_TASK_A_CORE);
    xTaskCreatePinnedToCore( adc_monitor_task,    "adc_monitor_task",           STACK_SIZE_B, this, TASK_VSENSE_RECEIVER_PRIORITY, &adc_monitor_task_handle,    I2S_TASK_B_CORE); 
    while( !adc_monitor_task_running.load() ) vTaskDelay(10);


    sennsors_running.store( true );
}





