#include <stdio.h>
#include "pico/stdlib.h"

// self headers
#include "settings.hpp"
#include "global_vars.hpp"
#include "sender.hpp"
#include "motor.hpp"
#include "lidar.hpp"
#include "encoder.hpp"
#include "imu.hpp"
#include "led.hpp"

// UART debug
// screen /dev/tty.usbserial-FTU7C2WR 115200
// screen /dev/tty.usbserial-FTU7C2WR 460800

// USB debug
// screen /dev/tty.usbmodem11101
// screen /dev/tty.usbmodem1101

// global extern variables
bool is_motor_fine = true;
TCP_Buffer tcp_buffer;  // init TCP buffer here since it is global

int main() {
    LED led;  // init LED first for indicator
    led.set_indicator(LED_INDICATOR::POWER_ON);

    bool has_wifi = false;
    init_data_sending(has_wifi);
    sleep_ms(1000);  // allow time for connection
    DBG("Rover Starting...\n");

    // init i2c
    // init_i2c1();

    MotorControl motor_control;
    // Lidar lidar;
    // IMU imu;

    /*
    things that have seperate update loops:
    - spinning lidar
    - motors (encoders included)
    - imu (xl + gyro)
    - tcp parser / buffer
    */
    DBG("Starting loop...\n");
    led.set_indicator(has_wifi ? LED_INDICATOR::LOOP_WITH_WIFI
                               : LED_INDICATOR::LOOP_NO_WIFI);
    while (true) {
        tcp_buffer.parse_tcp_buffer(motor_control);
        // imu.update();
        // lidar.update_lidar();
        // motor_control.update_motors();
        led.update();
        sleep_ms(10);
    }
}
