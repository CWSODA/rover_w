#pragma once

// toggle for debug printf
#define DEBUG_LOG true
#if DEBUG_LOG
#include <stdio.h>
#define DBG(...) printf(__VA_ARGS__)
#else
#define DBG(...)
#endif

#define todo assert(false)

/* ------------------------------------------------------ */
/*                        SETTINGS                        */
/* ------------------------------------------------------ */

/* ---------------------- debugging --------------------- */
#define UART_SERIAL false
#define USB_SERIAL true
#define NET_DATA true

constexpr int DEBUG_TX_PIN = 0;
constexpr int DEBUG_RX_PIN = 1;
constexpr uint UART_DEBUG_BAUDRATE = 115200 * 8;  // 921600

/* ------------------------- I2C ------------------------ */
constexpr uint I2C1_BAUDRATE = 100e3;  // max 1M for encoder
constexpr uint I2C1_SDA_PIN = 2;
constexpr uint I2C1_SCL_PIN = 3;

/* ------------------------- IMU ------------------------ */

/* ------------------------ Lidar ----------------------- */
constexpr uint LIDAR_RX_PIN = 5;
#define SEND_LIDAR_DATA true
#define TIMESTAMP_FRAME true
#define DEBUG_ROT_SPEED true
#define DEBUG_SAMPLE_COUNT false
#define DEBUG_STATES false
#define DEBUG_LENGTH false
#define DEBUG_ANGLE false
#define DEBUG_OFFSET_ANGLE false

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

/* ----------------------- Encoder ---------------------- */
constexpr uint ENCODER_FL_PIN = 18;
constexpr uint ENCODER_FR_PIN = 19;
constexpr uint ENCODER_BL_PIN = 20;
constexpr uint ENCODER_BR_PIN = 21;
