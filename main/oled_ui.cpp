#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>

#include "oled_ui.h"
#include "settings.h"


// U8x8 writes text directly to the authoritative SH1106 and avoids a 1 KiB
// framebuffer on the Nano.
static U8X8_SH1106_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);


enum UiScreen : uint8_t
{
  SCREEN_MAIN,
  SCREEN_TUNING,
  SCREEN_EDIT,
  SCREEN_SENSORS,
  SCREEN_STATUS
};


enum MainItem : uint8_t
{
  MAIN_START_RUN,
  MAIN_TUNING,
  MAIN_SENSOR_VALUES,
  MAIN_ITEM_COUNT
};


enum TuningItem : uint8_t
{
  TUNE_KP,
  TUNE_KD,
  TUNE_SPEED,
  TUNE_THRESHOLD,
  TUNE_ROUTE_PRIORITY,
  TUNE_CALIBRATE,
  TUNE_MOTOR_TEST,
  TUNING_ITEM_COUNT
};


static UiScreen currentScreen = SCREEN_MAIN;
static uint8_t mainItem = MAIN_START_RUN;
static uint8_t tuningItem = TUNE_KP;
static uint16_t originalValue = 0;
static bool digitalSensorView = false;


static void drawMarker(uint8_t row, bool selected)
{
  oled.setCursor(0, row);
  oled.print(selected ? '>' : ' ');
}


static void printGain(uint8_t valueX100)
{
  oled.print(valueX100 / 100);
  oled.print('.');
  const uint8_t decimals = valueX100 % 100;
  if (decimals < 10) oled.print('0');
  oled.print(decimals);
}


static void printPriority(uint8_t priority)
{
  switch (priority)
  {
    case PRIORITY_STRAIGHT_LEFT_RIGHT: oled.print(F("S>L>R")); break;
    case PRIORITY_STRAIGHT_RIGHT_LEFT: oled.print(F("S>R>L")); break;
    case PRIORITY_LEFT_STRAIGHT_RIGHT: oled.print(F("L>S>R")); break;
    case PRIORITY_LEFT_RIGHT_STRAIGHT: oled.print(F("L>R>S")); break;
    case PRIORITY_RIGHT_STRAIGHT_LEFT: oled.print(F("R>S>L")); break;
    default: oled.print(F("R>L>S")); break;
  }
}


static void printRouteName(RouteDirection route)
{
  if (route == ROUTE_STRAIGHT) oled.print(F("STRAIGHT"));
  else if (route == ROUTE_LEFT) oled.print(F("LEFT"));
  else oled.print(F("RIGHT"));
}


static void drawMainMenu()
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print(F("MAIN MENU"));

  drawMarker(2, mainItem == MAIN_START_RUN);
  oled.setCursor(2, 2);
  oled.print(F("START RUN"));

  drawMarker(3, mainItem == MAIN_TUNING);
  oled.setCursor(2, 3);
  oled.print(F("TUNING"));

  drawMarker(4, mainItem == MAIN_SENSOR_VALUES);
  oled.setCursor(2, 4);
  oled.print(F("SENSOR VALUES"));

  oled.setCursor(0, 7);
  oled.print(F("R2 ENTER"));
}


static void drawTuningLabel(uint8_t item, uint8_t row)
{
  drawMarker(row, tuningItem == item);
  oled.setCursor(2, row);

  switch (item)
  {
    case TUNE_KP:
      oled.print(F("KP"));
      oled.setCursor(12, row);
      printGain(kpX100);
      break;
    case TUNE_KD:
      oled.print(F("KD"));
      oled.setCursor(12, row);
      printGain(kdX100);
      break;
    case TUNE_SPEED:
      oled.print(F("SPEED"));
      oled.setCursor(12, row);
      oled.print(baseSpeed);
      break;
    case TUNE_THRESHOLD:
      oled.print(F("THRESH"));
      oled.setCursor(12, row);
      oled.print(sensorThreshold);
      break;
    case TUNE_ROUTE_PRIORITY:
      oled.print(F("ROUTE"));
      oled.setCursor(10, row);
      printPriority(routePriority);
      break;
    case TUNE_CALIBRATE:
      oled.print(F("CALIBRATE"));
      oled.setCursor(12, row);
      oled.print(sensorCalibrationValid ? F("YES") : F("NO"));
      break;
    default:
      oled.print(F("MOTOR TEST"));
      break;
  }
}


static void drawTuningMenu()
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print(F("TUNING"));

  const uint8_t first = tuningItem > 4 ? tuningItem - 4 : 0;
  for (uint8_t row = 1; row <= 5; row++)
  {
    drawTuningLabel(first + row - 1, row);
  }

  oled.setCursor(0, 7);
  oled.print(F("R2 EDIT L2 BACK"));
}


static uint16_t getSelectedValue()
{
  switch (tuningItem)
  {
    case TUNE_KP: return kpX100;
    case TUNE_KD: return kdX100;
    case TUNE_SPEED: return baseSpeed;
    case TUNE_THRESHOLD: return sensorThreshold;
    case TUNE_ROUTE_PRIORITY: return routePriority;
    default: return 0;
  }
}


static void setSelectedValue(uint16_t value)
{
  switch (tuningItem)
  {
    case TUNE_KP: kpX100 = value; break;
    case TUNE_KD: kdX100 = value; break;
    case TUNE_SPEED: baseSpeed = value; break;
    case TUNE_THRESHOLD: sensorThreshold = value; break;
    case TUNE_ROUTE_PRIORITY: routePriority = value; break;
  }
}


static void drawEditScreen()
{
  oled.clearDisplay();
  oled.setCursor(0, 0);

  if (tuningItem == TUNE_ROUTE_PRIORITY)
  {
    oled.print(F("ROUTE PRIORITY"));
    oled.setCursor(0, 1);
    oled.print(F("ORDER "));
    oled.print(routePriority + 1);
    oled.print(F(" OF 6"));
    for (uint8_t rank = 0; rank < 3; rank++)
    {
      oled.setCursor(0, 3 + rank);
      oled.print(rank + 1);
      oled.print(F(" "));
      printRouteName(settingsPriorityAt(rank));
    }
  }
  else
  {
    oled.print(F("EDIT "));
    if (tuningItem == TUNE_KP) oled.print(F("KP"));
    else if (tuningItem == TUNE_KD) oled.print(F("KD"));
    else if (tuningItem == TUNE_SPEED) oled.print(F("SPEED"));
    else oled.print(F("THRESHOLD"));

    oled.setCursor(0, 2);
    oled.print(F("VALUE: "));
    if (tuningItem == TUNE_KP) printGain(kpX100);
    else if (tuningItem == TUNE_KD) printGain(kdX100);
    else oled.print(getSelectedValue());
  }

  oled.setCursor(0, 6);
  oled.print(F("UP/DOWN CHANGE"));
  oled.setCursor(0, 7);
  oled.print(F("R2 OK L2 CANCEL"));
}


static void increaseSelectedValue()
{
  if (tuningItem == TUNE_KP && kpX100 < 200) kpX100++;
  else if (tuningItem == TUNE_KD && kdX100 < 200) kdX100++;
  else if (tuningItem == TUNE_SPEED)
    baseSpeed = baseSpeed <= 250 ? baseSpeed + 5 : 255;
  else if (tuningItem == TUNE_THRESHOLD)
    sensorThreshold = sensorThreshold <= 1013 ? sensorThreshold + 10 : 1023;
  else if (tuningItem == TUNE_ROUTE_PRIORITY)
    routePriority = (routePriority + 1) % ROUTE_PRIORITY_COUNT;
}


static void decreaseSelectedValue()
{
  if (tuningItem == TUNE_KP && kpX100 > 0) kpX100--;
  else if (tuningItem == TUNE_KD && kdX100 > 0) kdX100--;
  else if (tuningItem == TUNE_SPEED)
    baseSpeed = baseSpeed >= 5 ? baseSpeed - 5 : 0;
  else if (tuningItem == TUNE_THRESHOLD)
    sensorThreshold = sensorThreshold >= 10 ? sensorThreshold - 10 : 0;
  else if (tuningItem == TUNE_ROUTE_PRIORITY)
    routePriority = routePriority == 0 ? ROUTE_PRIORITY_COUNT - 1 :
                                        routePriority - 1;
}


static void drawSensorLabel(uint8_t column, uint8_t row, uint8_t index)
{
  oled.setCursor(column, row);
  if (index < 10) oled.print(' ');
  oled.print(index);
  oled.print(':');
}


static void drawSensorFrame()
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print(digitalSensorView ? F("DIGITAL U TOGGLE") :
                                 F("ANALOG  U TOGGLE"));

  for (uint8_t row = 1; row <= 7; row++)
  {
    drawSensorLabel(0, row, 14 - row);
    drawSensorLabel(8, row, 7 - row);
  }
}


static void printAnalogValue(uint16_t value)
{
  if (value < 1000) oled.print(' ');
  if (value < 100) oled.print(' ');
  if (value < 10) oled.print(' ');
  oled.print(value);
}


void uiInit()
{
  oled.setI2CAddress(0x3C * 2);
  oled.setBusClock(400000UL);
  oled.begin();
  oled.setFont(u8x8_font_chroma48medium8_r);
  uiReturnToMain();
}


UiAction uiHandleButton(ButtonEvent event)
{
  if (event == BUTTON_NONE) return UI_NO_ACTION;

  if (currentScreen == SCREEN_MAIN)
  {
    if (event == BUTTON_UP_CLICK)
    {
      mainItem = mainItem == 0 ? MAIN_ITEM_COUNT - 1 : mainItem - 1;
      drawMainMenu();
    }
    else if (event == BUTTON_DOWN_CLICK)
    {
      mainItem = (mainItem + 1) % MAIN_ITEM_COUNT;
      drawMainMenu();
    }
    else if (event == BUTTON_RIGHT_DOUBLE_CLICK)
    {
      if (mainItem == MAIN_START_RUN) return UI_START_RUN;
      if (mainItem == MAIN_TUNING)
      {
        currentScreen = SCREEN_TUNING;
        tuningItem = TUNE_KP;
        drawTuningMenu();
      }
      else
      {
        currentScreen = SCREEN_SENSORS;
        digitalSensorView = false;
        drawSensorFrame();
      }
    }
  }
  else if (currentScreen == SCREEN_TUNING)
  {
    if (event == BUTTON_UP_CLICK)
    {
      tuningItem = tuningItem == 0 ? TUNING_ITEM_COUNT - 1 : tuningItem - 1;
      drawTuningMenu();
    }
    else if (event == BUTTON_DOWN_CLICK)
    {
      tuningItem = (tuningItem + 1) % TUNING_ITEM_COUNT;
      drawTuningMenu();
    }
    else if (event == BUTTON_LEFT_DOUBLE_CLICK)
    {
      uiReturnToMain();
    }
    else if (event == BUTTON_RIGHT_DOUBLE_CLICK)
    {
      if (tuningItem == TUNE_CALIBRATE)
      {
        return UI_START_CALIBRATION;
      }
      if (tuningItem == TUNE_MOTOR_TEST)
      {
        return UI_START_MOTOR_TEST;
      }
      originalValue = getSelectedValue();
      currentScreen = SCREEN_EDIT;
      drawEditScreen();
    }
  }
  else if (currentScreen == SCREEN_EDIT)
  {
    if (event == BUTTON_UP_CLICK)
    {
      increaseSelectedValue();
      drawEditScreen();
    }
    else if (event == BUTTON_DOWN_CLICK)
    {
      decreaseSelectedValue();
      drawEditScreen();
    }
    else if (event == BUTTON_LEFT_DOUBLE_CLICK)
    {
      setSelectedValue(originalValue);
      currentScreen = SCREEN_TUNING;
      drawTuningMenu();
    }
    else if (event == BUTTON_RIGHT_DOUBLE_CLICK)
    {
      if (tuningItem == TUNE_THRESHOLD)
      {
        settingsUseGlobalThreshold();
      }
      settingsSaveIfChanged();
      currentScreen = SCREEN_TUNING;
      drawTuningMenu();
    }
  }
  else if (currentScreen == SCREEN_SENSORS)
  {
    if (event == BUTTON_UP_CLICK || event == BUTTON_DOWN_CLICK)
    {
      digitalSensorView = !digitalSensorView;
      drawSensorFrame();
    }
    else if (event == BUTTON_LEFT_DOUBLE_CLICK)
    {
      uiReturnToMain();
    }
  }

  return UI_NO_ACTION;
}


bool uiIsSensorScreen()
{
  return currentScreen == SCREEN_SENSORS;
}


bool uiWantsDigitalValues()
{
  return digitalSensorView;
}


void uiDrawSensorValues(const uint16_t values[])
{
  for (uint8_t row = 1; row <= 7; row++)
  {
    oled.setCursor(4, row);
    printAnalogValue(values[14 - row]);
    oled.setCursor(12, row);
    printAnalogValue(values[7 - row]);
  }
}


void uiDrawSensorPattern(uint16_t pattern)
{
  for (uint8_t row = 1; row <= 7; row++)
  {
    const uint8_t left = 14 - row;
    const uint8_t right = 7 - row;
    oled.setCursor(4, row);
    oled.print((pattern >> left) & 1);
    oled.print(F("   "));
    oled.setCursor(12, row);
    oled.print((pattern >> right) & 1);
    oled.print(F("   "));
  }
}


static void showStatus(const __FlashStringHelper *line1,
                       const __FlashStringHelper *line2)
{
  currentScreen = SCREEN_STATUS;
  oled.clearDisplay();
  oled.setCursor(0, 2);
  oled.print(line1);
  oled.setCursor(0, 4);
  oled.print(line2);
}


void uiShowPlacement()
{
  showStatus(F("PLACE ROBOT"), F("START IN 2 SEC"));
  oled.setCursor(0, 7);
  oled.print(F("L2 CANCEL"));
}


void uiShowRunning()
{
  showStatus(F("RUNNING"), F("SENSOR MONITORED"));
  oled.setCursor(0, 7);
  oled.print(F("L2 STOP"));
}


void uiShowCalibration()
{
  showStatus(F("CALIBRATION"), F("PLACE: 2 SEC"));
  oled.setCursor(0, 7);
  oled.print(F("L2 CANCEL"));
}


void uiShowCalibrationResult(bool succeeded)
{
  showStatus(succeeded ? F("CAL SAVED") : F("CAL FAILED"),
             succeeded ? F("EEPROM UPDATED") : F("CHECK ALL SENS"));
}


void uiShowMotorTest()
{
  showStatus(F("MOTOR TEST"), F("LIFT WHEELS 2S"));
  oled.setCursor(0, 7);
  oled.print(F("L2 CANCEL"));
}


void uiShowMotorTestComplete()
{
  showStatus(F("MOTOR TEST DONE"), F("CHECK DIRECTIONS"));
}


void uiReturnToMain()
{
  currentScreen = SCREEN_MAIN;
  mainItem = MAIN_START_RUN;
  drawMainMenu();
}
