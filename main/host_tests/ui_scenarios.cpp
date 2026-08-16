#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>

#include "../settings.cpp"
#include "../oled_ui.cpp"


uint32_t millis() { return 0; }
uint32_t micros() { return 0; }
void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return HIGH; }


static std::string rowText(uint8_t row)
{
  std::string text(HostOled::rows[row], 16);
  while (!text.empty() && text.back() == ' ') text.pop_back();
  return text;
}


static void initializeSettings()
{
  EEPROM.reset();
  kpX100 = 20;
  kdX100 = 50;
  baseSpeed = 200;
  sensorThreshold = 400;
  routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
  boxMode = false;
  sensorCalibrationValid = true;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    sensorMinimums[sensor] = 100;
    sensorMaximums[sensor] = 850;
  }
  settingsSaveIfChanged();
}


int main()
{
  initializeSettings();
  uiInit();
  assert(rowText(7) == "CLICK=OK HOLD=<");
  assert(uiHandleButton(BUTTON_ACTION_CLICK) == UI_START_RUN);

  // Open TUNING, then KP edit. A long ACTION restores the original value and
  // does not touch EEPROM.
  assert(uiHandleButton(BUTTON_DOWN_CLICK) == UI_NO_ACTION);
  assert(uiHandleButton(BUTTON_ACTION_CLICK) == UI_NO_ACTION);
  assert(currentScreen == SCREEN_TUNING);
  assert(rowText(7) == "CLICK=OK HOLD=<");
  assert(uiHandleButton(BUTTON_ACTION_CLICK) == UI_NO_ACTION);
  assert(currentScreen == SCREEN_EDIT);
  uiHandleButton(BUTTON_UP_CLICK);
  assert(kpX100 == 21);
  uiHandleButton(BUTTON_ACTION_LONG_PRESS);
  assert(currentScreen == SCREEN_TUNING && kpX100 == 20);
  kpX100 = 99;
  settingsLoad();
  assert(kpX100 == 20);

  // Short ACTION confirms the edit and persists it.
  uiHandleButton(BUTTON_ACTION_CLICK);
  uiHandleButton(BUTTON_UP_CLICK);
  assert(kpX100 == 21);
  uiHandleButton(BUTTON_ACTION_CLICK);
  assert(currentScreen == SCREEN_TUNING);
  kpX100 = 0;
  settingsLoad();
  assert(kpX100 == 21);

  // All six compact OLED labels are rendered from the priority table itself.
  const char *expectedLabels[ROUTE_PRIORITY_COUNT] =
  {
    "S>L>R", "S>R>L", "L>S>R",
    "L>R>S", "R>S>L", "R>L>S"
  };
  for (uint8_t priority = 0; priority < ROUTE_PRIORITY_COUNT; priority++)
  {
    routePriority = priority;
    HostOled::clear();
    printPriority();
    assert(rowText(0) == expectedLabels[priority]);
  }

  uiShowRunning();
  assert(rowText(7) == "HOLD TO STOP");

  std::cout << "UI scenarios passed: ACTION save/cancel and 6 priorities\n";
  return 0;
}
