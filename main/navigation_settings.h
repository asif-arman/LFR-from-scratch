#pragma once

#include <Arduino.h>


// One fresh 14-sensor frame is acquired per 3.5 ms navigation tick.
constexpr uint16_t CONTROL_PERIOD_US = 3500;

// Calibrated analog strength below 2% is ignored as sensor-floor noise.
constexpr uint8_t ANALOG_NOISE_FLOOR = 20;


// ================================================================
// FRAME-COUNTED CONFIRMATIONS AND DURATIONS
// ================================================================

constexpr uint16_t GAP_ALLOWANCE_MS = 200;
constexpr uint8_t SIDE_CONFIRM_TICKS = 2;            // ~7 ms
constexpr uint8_t JUNCTION_CLEAR_TICKS = 2;          // ~7 ms
constexpr uint8_t REACQUIRE_CONFIRM_TICKS = 3;       // ~10.5 ms
constexpr uint8_t TURN_CENTER_LOST_TICKS = 2;        // ~7 ms
constexpr uint8_t TURN_CAPTURE_MAX_ACTIVE = 10;
constexpr uint8_t START_EXIT_CONFIRM_TICKS = 4;      // ~14 ms
constexpr uint8_t FINISH_ARM_TICKS = 12;             // ~42 ms
constexpr uint8_t FINISH_CONFIRM_TICKS = 8;          // ~28 ms


// ================================================================
// SENSOR-PATTERN CLASSIFICATION
// ================================================================

constexpr uint8_t USABLE_LINE_MAX_ACTIVE = 7;
constexpr uint8_t USABLE_LINE_MAX_SPAN = 7;
constexpr uint8_t SIDE_CLUSTER_MIN_ACTIVE = 2;
constexpr uint8_t JUNCTION_SIDE_MIN_SPAN = 7;
constexpr uint8_t BOX_MIN_ACTIVE = 12;
constexpr uint8_t BOX_MIN_SPAN = 12;

// A disappearing line is a directed sharp corner only after its calibrated
// position reached an outer sensor region. Centered loss remains a gap.
constexpr int16_t RIGHT_EDGE_MAX_POSITION = 450; // near S0..S4
constexpr int16_t LEFT_EDGE_MIN_POSITION = 850;  // near S9..S13
constexpr int16_t MODERATE_ERROR_THRESHOLD = 200;
constexpr int16_t OUTER_ERROR_THRESHOLD = 400;


// ================================================================
// MOTOR COMMANDS
// ================================================================

constexpr uint8_t GAP_MAX_PWM = 75;
constexpr uint8_t JUNCTION_CRAWL_PWM = 80;
constexpr uint8_t REACQUIRE_PWM = 115;
constexpr uint8_t MODERATE_ERROR_PWM = 150;
constexpr uint8_t OUTER_ERROR_PWM = 115;
constexpr uint8_t START_BOX_EXIT_PWM = 95;
constexpr uint8_t TURN_PWM = 100;
constexpr uint8_t UTURN_PWM = 110;
