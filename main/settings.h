#pragma once

#include <Arduino.h>

#include "config.h"


enum RouteDirection : uint8_t
{
  ROUTE_NONE = 0,
  ROUTE_LEFT,
  ROUTE_STRAIGHT,
  ROUTE_RIGHT,
  ROUTE_U_TURN
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


// KP default: 0.20. Controls present line-error response. Increase if steering
// is weak; decrease if the robot oscillates.
// KD default: 0.50. Controls damping from error change. Increase to suppress
// overshoot; decrease if motor commands become noisy or twitchy.
// Both use fixed-point hundredths: 20 means 0.20.
extern uint8_t kpX100;
extern uint8_t kdX100;

// Default: 100 PWM. Controls normal PD speed. Increase for more speed after
// the robot is stable; decrease if it overshoots or loses the line.
extern uint8_t baseSpeed;

// Default: 400 ADC. Used until per-sensor calibration succeeds. Increase if
// white is being read as black; decrease if black is being missed.
extern uint16_t sensorThreshold;

// Default: Straight > Left > Right. The first detected branch in this strict
// order is selected and remains locked until the junction has been exited.
extern uint8_t routePriority;

// Default: OFF. When enabled, RUN treats broad black regions as start/finish
// boxes for the entire run.
extern bool boxMode;

extern bool sensorCalibrationValid;
extern uint16_t sensorMinimums[SENSOR_COUNT];
extern uint16_t sensorMaximums[SENSOR_COUNT];


// Load a validated EEPROM record or restore safe defaults.
void settingsLoad();

// EEPROM.update() is used byte-by-byte, so unchanged bytes are not rewritten.
void settingsSaveIfChanged();

// Use the global threshold again after the user edits THRESH.
void settingsUseGlobalThreshold();

// Install and persist measured per-sensor calibration endpoints.
void settingsApplyCalibration(const uint16_t minimums[],
                              const uint16_t maximums[]);

// Return the threshold currently used for one sensor.
uint16_t settingsThresholdForSensor(uint8_t sensorIndex);

// Normalize one raw reading to black strength in the range 0..1000.
uint16_t settingsBlackStrength(uint8_t sensorIndex, uint16_t value);

// Read one of the three directions in the selected priority order.
RouteDirection settingsPriorityAt(uint8_t index);
