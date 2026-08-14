#pragma once

#include <Arduino.h>

#include "buttons.h"


enum UiAction : uint8_t
{
  UI_NO_ACTION,
  UI_START_RUN,
  UI_START_CALIBRATION,
  UI_START_MOTOR_TEST
};


void uiInit();
UiAction uiHandleButton(ButtonEvent event);

bool uiIsSensorScreen();
bool uiWantsDigitalValues();
void uiDrawSensorValues(const uint16_t values[]);
void uiDrawSensorPattern(uint16_t pattern);

void uiShowPlacement();
void uiShowRunning();
void uiShowCalibration();
void uiShowCalibrationResult(bool succeeded);
void uiShowMotorTest();
void uiShowMotorTestComplete();
void uiReturnToMain();
