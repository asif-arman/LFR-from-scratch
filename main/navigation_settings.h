#pragma once

#include <Arduino.h>


// One fresh 14-sensor frame is acquired per 3.5 ms navigation tick.
constexpr uint16_t CONTROL_PERIOD_US = 3500;

// Limits a one-frame derivative kick without adding a history filter.
constexpr int16_t DERIVATIVE_ERROR_DELTA_LIMIT = 300;


// ================================================================
// FRAME-COUNTED CONFIRMATION AND TIMEOUTS
// ================================================================

constexpr uint8_t LOSS_PROBE_TICKS = 14;           // ~49 ms
constexpr uint8_t LINE_CONFIRM_TICKS = 3;          // ~10.5 ms
constexpr uint8_t PROBE_BRAKE_TICKS = 3;           // ~10.5 ms
constexpr uint8_t JUNCTION_CONFIRM_TICKS = 3;      // ~10.5 ms
constexpr uint8_t ROUTE_MIN_VOTES = 2;             // ~7 ms evidence
constexpr uint8_t START_EXIT_CONFIRM_TICKS = 4;    // ~14 ms
constexpr uint8_t FINISH_ARM_TICKS = 12;           // ~42 ms narrow travel
constexpr uint8_t FINISH_CONFIRM_TICKS = 8;        // ~28 ms
constexpr uint8_t TURN_CENTER_CONFIRM_TICKS = 2;   // ~7 ms
constexpr uint8_t JUNCTION_EXIT_CONFIRM_TICKS = 4; // ~14 ms
constexpr uint8_t TURN_MIN_TICKS = 10;             // ~35 ms
constexpr uint16_t TURN_TIMEOUT_TICKS = 400;        // ~1.4 s
constexpr uint8_t UTURN_MIN_TICKS = 72;             // ~252 ms
constexpr uint16_t UTURN_TIMEOUT_TICKS = 515;       // ~1.8 s
constexpr uint8_t INVERSE_CONFIRM_TICKS = 2;        // ~7 ms
constexpr uint8_t INVERSE_COOLDOWN_TICKS = 143;     // ~500 ms


// ================================================================
// SENSOR-PATTERN FILTERS
// ================================================================

constexpr uint8_t NARROW_LINE_MAX_ACTIVE = 5;
constexpr uint8_t NARROW_LINE_MAX_SPAN = 6;
constexpr uint8_t WIDE_FEATURE_MIN_ACTIVE = 8;
constexpr uint8_t SIDE_CLUSTER_MIN_ACTIVE = 2;
constexpr uint8_t BOX_MIN_ACTIVE = 12;
constexpr uint8_t BOX_MIN_SPAN = 12;

// Narrow exits outside this central error are branches, not straight exits.
constexpr uint16_t STRAIGHT_CENTER_ERROR_LIMIT = 180;
constexpr uint16_t CONTINUITY_CENTER_ERROR_LIMIT = 120;

constexpr uint16_t TURN_CENTER_ERROR_LIMIT = 150;
constexpr uint16_t TURN_TREND_MIN_STEP = 40;
constexpr uint8_t TURN_OUTWARD_CONFIRM = 2;
constexpr uint16_t TURN_LOSS_ERROR_LIMIT = 350;


// ================================================================
// MOTOR COMMANDS
// ================================================================

constexpr uint8_t FORWARD_PROBE_MAX_PWM = 200;
constexpr uint8_t GAP_PROBE_MIN_PWM = 100;
constexpr uint8_t GAP_STEERING_MAX_DELTA = 60;
constexpr uint8_t WIDE_AREA_CRAWL_PWM = 100;
constexpr uint8_t START_BOX_EXIT_PWM = 105;
constexpr uint8_t REACQUIRE_FORWARD_PWM = 115;

// Signed pivot command used for explicit left/right route turns.
constexpr uint8_t TURN_PWM = 120;

// Slightly stronger signed pivot used for 180-degree loss recovery.
constexpr uint8_t UTURN_PWM = 130;

constexpr uint8_t TURN_REACQUIRE_PWM = 100;
constexpr uint8_t TURN_CROSS_CORRECTION_PWM = 90;

constexpr uint8_t LEFT_MOTOR_EFFECTIVE_MIN_PWM = 90;
constexpr uint8_t RIGHT_MOTOR_EFFECTIVE_MIN_PWM = 90;
constexpr uint8_t FOLLOW_PWM_SLEW_STEP = 36;


// ================================================================
// ADAPTIVE FOLLOW SPEED
// ================================================================

// Ordinary PD begins slowing above this absolute position error.
constexpr uint16_t ADAPTIVE_SPEED_START_ERROR = 100;
constexpr uint8_t ADAPTIVE_SPEED_REDUCTION_DIVISOR = 5;
constexpr uint8_t ADAPTIVE_SPEED_MAX_REDUCTION = 70;

// A consistently outward-moving approach receives extra slowdown.
constexpr uint16_t TURN_SLOW_START_ERROR = 250;
constexpr uint8_t TURN_APPROACH_MIN_PWM = 120;
constexpr uint8_t TURN_APPROACH_REDUCTION_DIVISOR = 4;
