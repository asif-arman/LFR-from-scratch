#include "buttons.h"
#include "calibration.h"
#include "config.h"
#include "hardware.h"
#include "navigation.h"
#include "motor_test.h"
#include "oled_ui.h"
#include "robot.h"
#include "settings.h"


static uint16_t sensorValues[SENSOR_COUNT];


enum AppMode : uint8_t
{
  APP_MENU,
  APP_RUN_PLACEMENT,
  APP_RUNNING,
  APP_CALIBRATING,
  APP_MOTOR_TEST,
  APP_RESULT
};


static AppMode appMode = APP_MENU;
static uint32_t modeStartedAt = 0;
static uint32_t lastSensorDisplayUpdate = 0;


// Default: 2000 ms. Existing placement delay before any run motor command.
// Increase if positioning takes longer; decrease only if placement is reliable.
constexpr uint16_t RUN_PLACEMENT_DELAY_MS = 2000;

// Default: 120 ms. OLED diagnostic refresh. Increase to reduce display traffic;
// decrease for a faster-looking sensor view (at the cost of more I2C work).
constexpr uint8_t SENSOR_DISPLAY_INTERVAL_MS = 120;

// Default: 1400 ms. Time a calibration/test result stays visible.
constexpr uint16_t RESULT_DISPLAY_MS = 1400;


static void returnToMenu()
{
  navigationStop();
  calibrationCancel();
  motorTestCancel();
  stopMotors();
  appMode = APP_MENU;
  uiReturnToMain();
}


static void startRunPlacement()
{
  stopMotors();
  navigationStop();
  if (!sensorCalibrationValid)
  {
    uiShowCalibrationResult(false);
    appMode = APP_RESULT;
    modeStartedAt = millis();
    return;
  }
  appMode = APP_RUN_PLACEMENT;
  modeStartedAt = millis();
  uiShowPlacement();
}


static void startCalibration()
{
  stopMotors();
  navigationStop();
  calibrationStart();
  appMode = APP_CALIBRATING;
  uiShowCalibration();
}


static void startMotorTest()
{
  stopMotors();
  navigationStop();
  motorTestStart();
  appMode = APP_MOTOR_TEST;
  uiShowMotorTest();
}


void setup()
{
  motorInit();
  sensorInit();
  buttonsInit();
  settingsLoad();
  stopMotors();
  uiInit();
}


void loop()
{
  const ButtonEvent buttonEvent = readButtonEvent();
  const uint32_t now = millis();

  if (appMode == APP_RUN_PLACEMENT)
  {
    if (buttonEvent == BUTTON_ACTION_LONG_PRESS)
    {
      returnToMenu();
      return;
    }

    stopMotors();
    if (now - modeStartedAt >= RUN_PLACEMENT_DELAY_MS)
    {
      navigationStart();
      appMode = APP_RUNNING;
      uiShowRunning();
    }
    return;
  }

  if (appMode == APP_RUNNING)
  {
    if (buttonEvent == BUTTON_ACTION_LONG_PRESS)
    {
      returnToMenu();
      return;
    }

    const NavigationResult result = navigationTick(sensorValues);
    if (result == NAVIGATION_FINISHED || result == NAVIGATION_LOST)
    {
      stopMotors();
      if (result == NAVIGATION_FINISHED) uiShowRunFinished();
      else uiShowLineLost();
      appMode = APP_RESULT;
      modeStartedAt = now;
    }
    return;
  }

  if (appMode == APP_CALIBRATING)
  {
    if (buttonEvent == BUTTON_ACTION_LONG_PRESS)
    {
      returnToMenu();
      return;
    }

    const CalibrationResult result = calibrationTick(sensorValues);
    if (result != CALIBRATION_RUNNING)
    {
      stopMotors();
      uiShowCalibrationResult(result == CALIBRATION_SUCCEEDED,
                              calibrationFailedSensor());
      appMode = APP_RESULT;
      modeStartedAt = now;
    }
    return;
  }

  if (appMode == APP_MOTOR_TEST)
  {
    if (buttonEvent == BUTTON_ACTION_LONG_PRESS)
    {
      returnToMenu();
      return;
    }

    if (motorTestTick())
    {
      uiShowMotorTestComplete();
      appMode = APP_RESULT;
      modeStartedAt = now;
    }
    return;
  }

  if (appMode == APP_RESULT)
  {
    stopMotors();
    if (now - modeStartedAt >= RESULT_DISPLAY_MS ||
        buttonEvent == BUTTON_ACTION_LONG_PRESS ||
        buttonEvent == BUTTON_ACTION_CLICK)
    {
      returnToMenu();
    }
    return;
  }

  const UiAction action = uiHandleButton(buttonEvent);
  if (action == UI_START_RUN)
  {
    startRunPlacement();
    return;
  }
  if (action == UI_START_CALIBRATION)
  {
    startCalibration();
    return;
  }
  if (action == UI_START_MOTOR_TEST)
  {
    startMotorTest();
    return;
  }

  if (uiIsSensorScreen() &&
      now - lastSensorDisplayUpdate >= SENSOR_DISPLAY_INTERVAL_MS)
  {
    lastSensorDisplayUpdate = now;
    readSensors(sensorValues);
    if (uiWantsDigitalValues())
    {
      uiDrawSensorPattern(makeSensorPattern(sensorValues));
    }
    else
    {
      uiDrawSensorValues(sensorValues);
    }
  }
}
