#pragma once

#include <Arduino.h>


// ==================================================
// L298N MOTOR DRIVER
// ==================================================

constexpr uint8_t ENA = 10;
constexpr uint8_t ENB = 11;

// Confirmed left-motor direction pins.
constexpr uint8_t IN1 = 7;
constexpr uint8_t IN2 = 6;

// Confirmed right-motor direction pins.
constexpr uint8_t IN3 = 8;
constexpr uint8_t IN4 = 9;


// ==================================================
// 16-CHANNEL ANALOG MUX
// ==================================================

constexpr uint8_t MUX_A = A1;
constexpr uint8_t MUX_B = A2;
constexpr uint8_t MUX_C = A3;
constexpr uint8_t MUX_D = A0;

constexpr uint8_t MUX_SIG = A7;


// ==================================================
// THREE MENU BUTTONS
// ==================================================

constexpr uint8_t BUTTON_UP_PIN = 2;
constexpr uint8_t BUTTON_DOWN_PIN = 3;
constexpr uint8_t BUTTON_ACTION_PIN = 4;


// ==================================================
// SENSOR GEOMETRY
// ==================================================

constexpr uint8_t SENSOR_COUNT = 14;

// Each sensor is separated by 100 position units.
//
// S0  = 0
// S1  = 100
// S2  = 200
// ...
// S13 = 1300
constexpr uint16_t SENSOR_POSITION_SCALE = 100;

// The center between S6 and S7 is 650.
constexpr int16_t LINE_CENTER =
    ((SENSOR_COUNT - 1) *
     SENSOR_POSITION_SCALE) / 2;
