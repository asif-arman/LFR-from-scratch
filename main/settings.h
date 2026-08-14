#pragma once

#include <Arduino.h>

#include "config.h"


// Route bits are deliberately powers of two so available and attempted
// directions fit in one byte.
enum RouteDirection : uint8_t
{
  ROUTE_NONE = 0,
  ROUTE_LEFT = 1 << 0,
  ROUTE_STRAIGHT = 1 << 1,
  ROUTE_RIGHT = 1 << 2,
  ROUTE_U_TURN = 1 << 3
};


enum RoutePriorityOrder : uint8_t
{
  PRIORITY_STRAIGHT_LEFT_RIGHT,
  PRIORITY_STRAIGHT_RIGHT_LEFT,
  PRIORITY_LEFT_STRAIGHT_RIGHT,
  PRIORITY_LEFT_RIGHT_STRAIGHT,
  PRIORITY_RIGHT_STRAIGHT_LEFT,
  PRIORITY_RIGHT_LEFT_STRAIGHT,
  ROUTE_PRIORITY_COUNT
};


// KP default: 0.30. Controls present line-error response. Increase if steering
// is weak; decrease if the robot oscillates.
// KD default: 0.08. Controls damping from error change. Increase to suppress
// overshoot; decrease if motor commands become noisy or twitchy.
// Both use fixed-point hundredths: 30 means 0.30.
extern uint8_t kpX100;
extern uint8_t kdX100;

// Default: 160 PWM. Controls normal PD speed. Increase for more speed after
// the robot is stable; decrease if it overshoots or loses the line.
extern uint8_t baseSpeed;

// Default: 400 ADC. Used until per-sensor calibration succeeds. Increase if
// white is being read as black; decrease if black is being missed.
extern uint16_t sensorThreshold;

// Default: Straight > Left > Right. Controls which detected, untried branch
// is selected first. Change it to match the required course strategy.
extern uint8_t routePriority;

extern bool sensorCalibrationValid;
extern uint16_t sensorThresholds[SENSOR_COUNT];


// Load a validated EEPROM record or restore safe defaults.
void settingsLoad();

// EEPROM.update() is used byte-by-byte, so unchanged bytes are not rewritten.
void settingsSaveIfChanged();

// Use the global threshold again after the user edits THRESH.
void settingsUseGlobalThreshold();

// Install and persist new per-sensor midpoint thresholds.
void settingsApplyCalibration(const uint16_t thresholds[]);

// Return the threshold currently used for one sensor.
uint16_t settingsThresholdForSensor(uint8_t sensorIndex);

// Read one of the three directions in the selected priority order.
RouteDirection settingsPriorityAt(uint8_t index);
