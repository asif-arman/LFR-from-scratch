#pragma once

#include <Arduino.h>


enum CalibrationResult : uint8_t
{
  CALIBRATION_RUNNING,
  CALIBRATION_SUCCEEDED,
  CALIBRATION_FAILED
};


void calibrationStart();
void calibrationCancel();
CalibrationResult calibrationTick(uint16_t sensorValues[]);
uint8_t calibrationFailedSensor();
