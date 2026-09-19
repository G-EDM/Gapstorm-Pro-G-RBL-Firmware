//############################################################################################################
//  ______     _______ ______  _______
// |  ____ ___ |______ |     \ |  |  |
// |_____|     |______ |_____/ |  |  |
// ___  _  _ ____ __ _ ___ ____ _  _
// |--' |--| |--| | \|  |  [__] |\/|
//
// Sinker and Wire EDM firmware 
//
// How to install:
//
// First Install:
//     1. Install visual studio code and open it. Go to extensions and install platform.io
//     2. On windows install the USB to UART driver if needed
//     3. Press open folder and load the project folder. It may take some time until everything is setup. 
//        Restart vstudio once it is finished.
//     4. Connect the ESP via USB. On Linux it may be required to use "chmod 777 /dev/ttyUSB0" 
//        to gain write access to the ESP. ttyUSB0 could be another address too. 
//     5. Press the upload button on the top right and wait until it is finished.
//
// Updating:
//     1. Press the build button. It is at the same location as the upload button. Upload and Build can 
//        can be toggled with the small drop down menu.
//        vStudio will generate a firmware.bin file that should be located within the project folder at
//        /.pio/build/esp32dev/firmware.bin
//     2. Copy the firmware.bin file on the sd card at the top directory. Not into any folder.
//     3. Insert the SD card. Once the sd card is detected it will search for an update. Wait until the update
//        is done and the ESP will restart.
//
//
//
// Arch linux using "screen" to monitor serial:
// Install screen:
// sudo pacman -S screen
//
// find port first:
// ls -l /dev/ttyUSB* /dev/ttyACM*
//
// connect:
// screen /dev/ttyUSB1 115200
// ttyUSB can be anything from ttyUSB0 or ttyUSB2 etc..
//############################################################################################################
#include <driver/adc.h>
#include "gedm_main.h"
#include "definitions.h"
#include "Grbl.h"
#include "widgets/ui_interface.h"

TaskHandle_t protocol_task = NULL;

void setup() {
    Serial.begin( DPM_RXTX_SERIAL_COMMUNICATION_BAUD_RATE ); // baud set to 115200
    while(!Serial);
    disableCore0WDT();
    disableCore1WDT();
    grbl.init();
    gsense.setup();
    ui_controller.start_ui_task();
    grbl.reset_variables();
    xTaskCreatePinnedToCore( gcode_consumer_task, "protocol_task", 8000, &gproto, TASK_PROTOCOL_PRIORITY, &protocol_task, 1 );
}

void loop(){ 
    vTaskDelete(NULL); 
}