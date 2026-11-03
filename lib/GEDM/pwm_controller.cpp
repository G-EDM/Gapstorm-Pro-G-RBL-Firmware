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



    GPIO_REG_READ(GPIO_IN_REG)
    Reads the current value of the GPIO input register. This register contains the logic levels of all GPIO pins.

    GEDM_PWM_PIN & 0x1F
    Masks the GEDM_PWM_PIN with 0x1F (which is 31 in decimal). This effectively extracts the lower 5 bits of GEDM_PWM_PIN, assuming GPIO pin numbers are within 0-31.

    (gpio_num_t)(GEDM_PWM_PIN & 0x1F)
    Casts the masked value to the gpio_num_t type, which is typically an enum or typedef representing GPIO pin numbers.

    GPIO_REG_READ(GPIO_IN_REG) >> (gpio_num_t)(GEDM_PWM_PIN & 0x1F)
    Shifts the register value right by the pin number, moving the target GPIO pin's bit to the least significant bit position.

    & 1U
    Masks out all bits except the least significant bit, effectively reading the state (0 or 1) of that specific GPIO pin.


*/
//############################################################################################################
#include "pwm_controller.h"
#include "sensors.h"
#include "gedm_dpm_driver.h"
#include "ili9341_tft.h"
#include <freertos/portmacro.h>
#include <esp32-hal-gpio.h>
#include <driver/ledc.h>
#include <soc/ledc_periph.h>
#include <soc/ledc_reg.h>
#include <soc/ledc_struct.h>


enum arc_gen {
    ARC_OK = 0,
    ARC_CREATE
};

std::map<arc_gen, const char*> arc_messages = {
    { ARC_OK,     "" },
    { ARC_CREATE, "Creating arc generator" }
};

static DRAM_ATTR std::mutex arc_mtx; // the protection function is called inside the i2s loop and needs to execute as fast as possible. Only reason to put the mutex into ram     


constexpr ledc_timer_t     timer_sel              = LEDC_TIMER_0;
constexpr ledc_timer_bit_t timer_bit_width        = LEDC_TIMER_8_BIT;
constexpr int              max_duty_int           = ( ( 1 << timer_bit_width ) - 1 ); // 8Bit range: 0-255


std::atomic<bool> spark_generator_is_running_flag(false);

DRAM_ATTR std::atomic<bool> arc_generator_pwm_off_flag(false);

ARC_GENERATOR arcgen;

hw_timer_t *adc_timer = nullptr;
constexpr int MIN_INTERVAL_US = 40; // 50=20khz 100=10khz

// Timer interrupt that will unlock the I2S sampling task
IRAM_ATTR void adc_on_timer(){
    static DRAM_ATTR bool timer_triggered = false;
    static DRAM_ATTR int data             = SENSE_LOAD_SAMPLES;
    if( 
        timer_triggered || // already running
        xQueueIsQueueEmptyFromISR( adc_read_trigger_queue ) == pdFALSE // queue already contains an item
    ){ return; }
    timer_triggered = true;
    xQueueSendFromISR( adc_read_trigger_queue, &data, NULL );
    timer_triggered = false;
}
IRAM_ATTR void timer_set_speed( int frequency = 0 ){
    static int timer_speed_current = 0;
    frequency = ( frequency > 0 ) ? ( 1000000 / ( frequency ) ) - 1 : MIN_INTERVAL_US;
    if( frequency < MIN_INTERVAL_US ){ frequency = MIN_INTERVAL_US; }
    if( timer_speed_current == frequency ) return; // nothing to do
    timer_speed_current = frequency;
    timerAlarmWrite(adc_timer, frequency, true);
}
void end_adc_timer(){
    if( adc_timer ){
        timerDetachInterrupt( adc_timer );
        timerEnd( adc_timer );
        adc_timer = nullptr;
    }
}
void create_adc_timer( int frequency = 0 ){
    if( adc_timer ) return;
    frequency = ( frequency > 0 ) ? ( 1000000 / ( frequency ) ) - 1 : MIN_INTERVAL_US;
    if( frequency < MIN_INTERVAL_US ){
        frequency = MIN_INTERVAL_US;
    }
    adc_timer = timerBegin(3, 80, true);
    timerAttachInterrupt(adc_timer, &adc_on_timer, true);
    timer_set_speed( frequency );
    timerAlarmDisable( adc_timer );
    //timerDetachInterrupt(adc_timer);
}
void resume_adc_timer( int frequency = 0 ){
    if( timerAlarmEnabled(adc_timer) ) return;
    timer_set_speed( frequency );
    timerAlarmEnable(adc_timer);
    vTaskDelay(10);
}
void pause_adc_timer(){
    if( !timerAlarmEnabled(adc_timer) ) return;
    timerAlarmDisable( adc_timer );
    vTaskDelay(10);
}



void log_number( int number ){

    const char *msg = int_to_char( number ); 
    debuglog( msg, 3000 ); 
    delete[] msg;


}



/**/

int khz_to_hz( float khz ){
    return round( khz * 1000.0 );
}



bool setting_change_notify_callback( setget_param_enum param_id, settings_container data ){

    switch( param_id ){

        case PARAM_ID_FREQ: 
            arcgen.change_pwm_frequency( khz_to_hz( data.fvalue ) ); 
        break;

        case PARAM_ID_DUTY: 
            arcgen.change_pwm_duty( data.fvalue );        
        break;

        case PARAM_ID_PWM_STATE: 
            ( data.value == 1 ? arcgen.enable_spark_generator() : arcgen.disable_spark_generator() ); 
            enforce_redraw.store( true ); 
        break;

        default: break; 
    }
    vTaskDelay(50);
    return true;

}






ARC_GENERATOR::ARC_GENERATOR(){}
ARC_GENERATOR::~ARC_GENERATOR(){}

void ARC_GENERATOR::end(){
    pwm_off();
    end_adc_timer();
    stop_ledc();
}


void ARC_GENERATOR::create(){
    debuglog( arc_messages[ARC_CREATE] );

    pinMode( GEDM_PWM_PIN, OUTPUT );
    digitalWrite( GEDM_PWM_PIN, LOW );

    // add the settings
    notify_callbacks[ SETTING_NOTIFY_ARCGEN ] = setting_change_notify_callback;
    settings.add( PARAM_ID_PWM_STATE,  SETTING_TYPE_BOOL,  0, 0.0,                           0.0,                   0.0,                   SETTING_NOTIFY_ARCGEN );
    settings.add( PARAM_ID_DUTY,       SETTING_TYPE_FLOAT, 0, DEFAULT_PWM_DUTY,              0.0,                   PWM_DUTY_MAX,          SETTING_NOTIFY_ARCGEN );
    settings.add( PARAM_ID_PROBE_DUTY, SETTING_TYPE_FLOAT, 0, DEFAULT_PROBING_DUTY,          0.0,                   PWM_DUTY_MAX,          SETTING_NOTIFY_ARCGEN );
    settings.add( PARAM_ID_FREQ,       SETTING_TYPE_FLOAT, 0, DEFAULT_PWM_FREQUENCY_KHZ,     PWM_FREQUENCY_KHZ_MIN, PWM_FREQUENCY_KHZ_MAX, SETTING_NOTIFY_ARCGEN );
    settings.add( PARAM_ID_PROBE_FREQ, SETTING_TYPE_FLOAT, 0, DEFAULT_PROBING_FREQUENCY_KHZ, PWM_FREQUENCY_KHZ_MIN, PWM_FREQUENCY_KHZ_MAX, SETTING_NOTIFY_ARCGEN );

    int freq_hz = khz_to_hz( settings.get_setting_float( PARAM_ID_FREQ ) );

    start_ledc();
    change_pwm_frequency( freq_hz );
    change_pwm_duty( settings.get_setting_float( PARAM_ID_DUTY ) );
    create_adc_timer( freq_hz );
    disable_spark_generator();

}


void ARC_GENERATOR::start_ledc( void ){


    //periph_module_reset(PERIPH_LEDC_MODULE);



    float duty_percent = settings.get_setting_float( PARAM_ID_DUTY ); // has own lock inside
    int   duty         = MAX( 1, duty_percent > 0.0 ? round( ( duty_percent * max_duty_int ) / 100 ) : 0 ); // convert to int range 0-255
    int   freq_hz      = khz_to_hz( settings.get_setting_float( PARAM_ID_FREQ ) );

    ledc_timer_config_t   ledc_timer;        
    ledc_channel_config_t ledc_conf;

    ledc_timer.speed_mode      = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num       = timer_sel;
    ledc_timer.duty_resolution = timer_bit_width; // = LEDC_TIMER_8_BIT
    ledc_timer.freq_hz         = freq_hz;
    ledc_timer.clk_cfg         = LEDC_AUTO_CLK;

    ledc_conf.channel          = LEDC_CHANNEL_0;
    ledc_conf.duty             = duty;
    ledc_conf.gpio_num         = GEDM_PWM_PIN;
    ledc_conf.hpoint           = 0;
    ledc_conf.intr_type        = LEDC_INTR_DISABLE;
    ledc_conf.speed_mode       = LEDC_HIGH_SPEED_MODE;
    ledc_conf.timer_sel        = timer_sel;

    std::lock_guard<std::mutex> lock( arc_mtx ); vTaskDelay(10);
    ledc_timer_config( &ledc_timer );
    ledc_channel_config( &ledc_conf );

    arc_generator_pwm_off_flag.store( false );

}



void ARC_GENERATOR::stop_ledc( void ){
    std::lock_guard<std::mutex> lock( arc_mtx );
    ledc_stop( LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0 );
    digitalWrite( GEDM_PWM_PIN, LOW );
}

































// it will stop populating the i2s readouts queue
// to free resources for other stuff if needed
std::atomic<uint16_t> locks_taken_count(0);
IRAM_ATTR void ARC_GENERATOR::lock() { // just pauses the adc timer
    uint16_t previous_count = locks_taken_count.fetch_add(1, std::memory_order_acquire);
    if( previous_count == 0 ){ // First lock acquired
        {
            std::lock_guard<std::mutex> lock( arc_mtx );
            pause_adc_timer();
        }
    }
    // add a little delay to ensure sensor loop iteration is done
    
}
IRAM_ATTR void ARC_GENERATOR::unlock() { // reenables adc timer with some condition checks
    uint16_t previous_count = locks_taken_count.fetch_sub(1, std::memory_order_release);
    if( previous_count == 1 ){ // Last lock released
        {
            std::lock_guard<std::mutex> lock( arc_mtx );
            if( get_pwm_is_off() ) return; // if pwm was turned off in the meantime don't reenable
            resume_adc_timer( khz_to_hz( settings.get_setting_float( PARAM_ID_FREQ ) ) );
        }
    }
}







// Fastly turn PWM off without disabling the arc generator
// if he arcgen is flagged as not running it will exit the process
// and this function is used to turn pwm off without flagging the arcgen as disabled
void ARC_GENERATOR::pwm_off(){
    change_pwm_duty( 0.0 );
    std::lock_guard<std::mutex> lock( arc_mtx ); 
    pause_adc_timer();
    LEDC.channel_group[ LEDC_HIGH_SPEED_MODE ].channel[ LEDC_CHANNEL_0 ].conf0.sig_out_en = 0;
    arc_generator_pwm_off_flag.store( true );
}

// Fastly turn PWM back on without affecting the arc generator state
void ARC_GENERATOR::pwm_on(){
    int freq_hz = khz_to_hz( settings.get_setting_float( PARAM_ID_FREQ ) );
    arc_generator_pwm_off_flag.store( false );
    change_pwm_frequency( freq_hz );
    change_pwm_duty( settings.get_setting_float( PARAM_ID_DUTY ) );
    std::lock_guard<std::mutex> lock( arc_mtx );
    resume_adc_timer( freq_hz ); // 20000 / 1000 = 20 10000 / 1000 = 10
    LEDC.channel_group[ LEDC_HIGH_SPEED_MODE ].channel[ LEDC_CHANNEL_0 ].conf0.sig_out_en = 1;
}






void ARC_GENERATOR::secure_on_off( bool turn_off, int delay = 5 ){ // turn pwm on off. Hopefully threadsafe. does not flag the arcgen as off. Just used in the planner for pausings where pwm needs to be off
    if( turn_off == get_pwm_is_off() ){ delayMicroseconds( delay ); return; }
    remote_control( turn_off ? CMD_PWMOFF : CMD_PWMON );
    while( turn_off != get_pwm_is_off() ){
        if( get_quit_motion( true ) ) break;
        delayMicroseconds(10); 
    }
}




































void IRAM_ATTR ARC_GENERATOR::protection_off(){ // shut arc off for some time quickly, time will still run and populate the adc queue
    std::lock_guard<std::mutex> lock( arc_mtx );
    if( arc_generator_pwm_off_flag.load() ) return;
    LEDC.channel_group[ LEDC_HIGH_SPEED_MODE ].channel[ LEDC_CHANNEL_0 ].conf0.sig_out_en = 0;
}
void IRAM_ATTR ARC_GENERATOR::protection_on(){ // 
    std::lock_guard<std::mutex> lock( arc_mtx ); // needed, pwm off flag is set after pwm is actually turned off. Parallel call to this would reenable without mutex
    if( arc_generator_pwm_off_flag.load() ) return;
    LEDC.channel_group[ LEDC_HIGH_SPEED_MODE ].channel[ LEDC_CHANNEL_0 ].conf0.sig_out_en = 1;
}







// Turn the arc generator OFF
void ARC_GENERATOR::disable_spark_generator(){
    pwm_off();
    spark_generator_is_running_flag.store(false);
}

// Turn the arc generator ON
void ARC_GENERATOR::enable_spark_generator(){
    spark_generator_is_running_flag.store(true); // needs to be set first else duty will be set to zero
    pwm_on();
    spark_generator_is_running_flag.store(true); // ???
}








// Check if spark generator is running
bool ARC_GENERATOR::is_running(){
    return spark_generator_is_running_flag.load();
}

// Check if PWM is on or off. Doesn't check if the arc generator is 
// running. Just if the PWM is currently running.
bool IRAM_ATTR ARC_GENERATOR::get_pwm_is_off(){
    return arc_generator_pwm_off_flag.load();
}

// Get the PWM frequency in Hz
/*int ARC_GENERATOR::get_freq(){
    std::lock_guard<std::mutex> lock( arc_mtx );
    return pwm_frequency_hz;
}*/







// Change the duty etc. This is only called from within this 
// class to finally apply the new duty. Don't call it directly.
// duty is converted to 0-255 (8Bit)
void ARC_GENERATOR::change_pwm_duty( float duty_cycle_percent ){
    int duty = duty_cycle_percent > 0.0 ? round( ( duty_cycle_percent * max_duty_int ) / 100 ) : 0;
    if( is_system_mode_edm() && get_pwm_is_off() ){
        // gets here if value is changed in the process screen
        // Prevents the new duty from getting set after updating required to keep PWM turned off until process is resumed
        if( duty == 0 ){
            digitalWrite( GEDM_PWM_PIN, LOW );
        }
        return;
    }

    std::lock_guard<std::mutex> lock( arc_mtx );
    // if( !spark_generator_is_running_flag.load() ){ 
    if( get_pwm_is_off() ){ 
        duty = 0; // force LEDC to go low
    }
    ledc_set_duty( LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty );
    ledc_update_duty( LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0 );
    if( duty == 0 ){
        digitalWrite( GEDM_PWM_PIN, LOW );
    }
}



// Enable probe mode. It will adjust PWM frequency and duty
// and also change the DPM voltage and current and turn it on if
// DPM support is enabled. The current settings are saved
// and restored after probing
void ARC_GENERATOR::probe_mode_on(){
    pwm_on();                                      // ledc lock inside
    change_pwm_frequency( khz_to_hz( settings.get_setting_float( PARAM_ID_PROBE_FREQ ) ) ); // ledc lock inside
    change_pwm_duty( settings.get_setting_float( PARAM_ID_PROBE_DUTY ) );               // ledc lock inside
    vTaskDelay(300);
}
// Restore the backup settings after probing
// restores PWM frequency, duty
// then turns pwm_off without disabling the arc gen
void ARC_GENERATOR::probe_mode_off(){
    change_pwm_frequency( khz_to_hz( settings.get_setting_float( PARAM_ID_FREQ ) ) ); 
    change_pwm_duty( settings.get_setting_float( PARAM_ID_DUTY ) );
    pwm_off();
}








// Change the PWM frequency (Hz)
void ARC_GENERATOR::change_pwm_frequency( int freq_hz ){

    std::lock_guard<std::mutex> lock( arc_mtx );
    if( adc_timer ){
        timer_set_speed( freq_hz );
    }

    if( is_system_mode_edm() && get_pwm_is_off() ){
        // gets here if value is changed in the process screen
        // don't apply until process is resumed
        return;
    }

    ledc_set_freq( LEDC_HIGH_SPEED_MODE, timer_sel, freq_hz );

}







