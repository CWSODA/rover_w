#pragma once

#include "pico/stdlib.h"

// toggles for optimization possible
// used for tracking
#define SHOW_OPT false
#if SHOW_OPT
#define OPT(...) \
    fail:)
#else
#define OPT
#endif

#define TODO assert(false)

/* ------------------------------------------------------ */
/*                        SETTINGS                        */
/* ------------------------------------------------------ */

/* ---------------------- debugging --------------------- */
#define DEBUG_LOG true
#define UART_DEBUG false
#define USB_DEBUG true  // set true for easy flashing
#define USB_SEND false  // send byte data to USB, disable to only use printf/DBG
#define USB_WAIT false  // configures whether to wait for usb connection
#define NET_DEBUG true

constexpr int DEBUG_TX_PIN = 0;
constexpr int DEBUG_RX_PIN = 1;
constexpr uint UART_DEBUG_BAUDRATE = 115200 * 8;  // 921600

/* ------------------------- TCP ------------------------ */
constexpr uint16_t TCP_PORT = 4242;
constexpr uint8_t TCP_POLL_TIME_S = 1;
constexpr uint TCP_BUFFER_SIZE = 2048;

// #define WIFI_SSID "VM24B898"
// #define WIFI_PASSWORD "xVqtbjdqwyy4"
#define WIFI_SSID "Nada"
#define WIFI_PASSWORD "cvmf8452"

#define DEBUG_TCP_WRITE false

/* ------------------------- I2C ------------------------ */
constexpr uint I2C1_BAUDRATE = 400e3;  // max 1M for encoder, 400k for IMU
constexpr uint I2C1_SDA_PIN = 2;
constexpr uint I2C1_SCL_PIN = 3;

/* ------------------------- IMU ------------------------ */
#define DEBUG_IMU false

#define SEND_IMU_DATA true
constexpr float IMU_SEND_CD_MS = 1000.0f / 100.0f;  // 100 Hz

/* ------------------------ LiDAR ----------------------- */
constexpr uint LIDAR_RX_PIN = 5;
#define SEND_LIDAR_DATA true
#define TIMESTAMP_FRAME false
#define DEBUG_ROT_SPEED false
#define DEBUG_SAMPLE_COUNT false
#define DEBUG_LIDAR_STATE false
#define DEBUG_LENGTH false
#define DEBUG_ANGLE false
#define DEBUG_OFFSET_ANGLE false

// one lidar sample per throttle count
constexpr int DATA_THROTTLE_COUNT = 1;

/* ----------------------- Motors ----------------------- */
constexpr uint MOTOR_FL_PWM_PIN = 6;
constexpr uint MOTOR_FL_DIR_PIN = 7;
constexpr uint MOTOR_FR_PWM_PIN = 8;
constexpr uint MOTOR_FR_DIR_PIN = 9;
constexpr uint MOTOR_BL_PWM_PIN = 10;
constexpr uint MOTOR_BL_DIR_PIN = 11;
constexpr uint MOTOR_BR_PWM_PIN = 12;
constexpr uint MOTOR_BR_DIR_PIN = 13;

// PID
constexpr float MOTOR_P = 1.0f;
constexpr float MOTOR_I = 1.0f;
constexpr float MOTOR_D = 1.0f;

constexpr float MANUAL_DRIVE_TIMEOUT_MS = 1000.0f;

#define DEBUG_PWM false

/* ----------------------- Encoder ---------------------- */
constexpr uint ENCODER_FL_PIN = 18;
constexpr uint ENCODER_FR_PIN = 19;
constexpr uint ENCODER_BL_PIN = 20;
constexpr uint ENCODER_BR_PIN = 21;

#define DEBUG_ENCODER false

/* ---------------------- Algorithm --------------------- */
constexpr float DIST_THRESHOLD = 1.0f;     // ignore if above in meters
constexpr uint8_t SIG_STR_THRESHOLD = 10;  // ignore if below in (0 to 255)
constexpr float FOV = 180.0f;              // ignore if above in degrees

constexpr float TURN_MULTIPLIER = 10.0f;
constexpr float SPEED_MULTIPLIER = 10.0f;

/* ------------------------ LEDS ------------------------ */
constexpr uint LED_R_PIN = 27;
constexpr uint LED_G_PIN = 26;
constexpr uint LED_B_PIN = 22;

#include "sender.hpp"