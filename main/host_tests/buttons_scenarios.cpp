#include <cassert>
#include <cstdint>
#include <iostream>

#include "../buttons.cpp"


static uint32_t simulatedMillis = 0;
static uint8_t pinLevels[32] = {};
static uint16_t pinModeCalls[32] = {};
static uint16_t digitalReadCalls[32] = {};


uint32_t millis() { return simulatedMillis; }
uint32_t micros() { return simulatedMillis * 1000UL; }


void pinMode(uint8_t pin, uint8_t)
{
  pinModeCalls[pin]++;
}


int digitalRead(uint8_t pin)
{
  digitalReadCalls[pin]++;
  return pinLevels[pin];
}


static ButtonEvent sampleAt(uint32_t timeMillis)
{
  simulatedMillis = timeMillis;
  return readButtonEvent();
}


static void setAction(uint8_t level)
{
  pinLevels[BUTTON_ACTION_PIN] = level;
}


int main()
{
  for (uint8_t pin = 0; pin < 32; pin++) pinLevels[pin] = HIGH;
  buttonsInit();

  assert(pinModeCalls[BUTTON_UP_PIN] == 1);
  assert(pinModeCalls[BUTTON_DOWN_PIN] == 1);
  assert(pinModeCalls[BUTTON_ACTION_PIN] == 1);
  assert(pinModeCalls[12] == 0 && digitalReadCalls[12] == 0);

  // Press and release bounce must settle independently. ACTION deliberately
  // emits its short-click event only after a stable release.
  setAction(LOW);
  assert(sampleAt(1) == BUTTON_NONE);
  setAction(HIGH);
  assert(sampleAt(5) == BUTTON_NONE);
  setAction(LOW);
  assert(sampleAt(9) == BUTTON_NONE);
  assert(sampleAt(28) == BUTTON_NONE);
  assert(sampleAt(29) == BUTTON_NONE);
  assert(sampleAt(90) == BUTTON_NONE);

  setAction(HIGH);
  assert(sampleAt(100) == BUTTON_NONE);
  setAction(LOW);
  assert(sampleAt(105) == BUTTON_NONE);
  setAction(HIGH);
  assert(sampleAt(110) == BUTTON_NONE);
  assert(sampleAt(129) == BUTTON_NONE);
  assert(sampleAt(130) == BUTTON_ACTION_CLICK);
  assert(sampleAt(180) == BUTTON_NONE);

  // A held ACTION emits one long press at about 600 ms, never repeats, and
  // its eventual release must not become a short Enter/OK click.
  setAction(LOW);
  assert(sampleAt(200) == BUTTON_NONE);
  assert(sampleAt(220) == BUTTON_NONE);
  for (uint32_t time = 225; time < 820; time += 5)
    assert(sampleAt(time) == BUTTON_NONE);
  assert(sampleAt(820) == BUTTON_ACTION_LONG_PRESS);
  for (uint32_t time = 821; time <= 1000; time += 3)
    assert(sampleAt(time) == BUTTON_NONE);

  setAction(HIGH);
  assert(sampleAt(1001) == BUTTON_NONE);
  assert(sampleAt(1020) == BUTTON_NONE);
  assert(sampleAt(1021) == BUTTON_NONE);
  assert(sampleAt(1200) == BUTTON_NONE);

  // UP and DOWN still emit once on a debounced press and never repeat held.
  pinLevels[BUTTON_UP_PIN] = LOW;
  assert(sampleAt(1300) == BUTTON_NONE);
  assert(sampleAt(1320) == BUTTON_UP_CLICK);
  assert(sampleAt(1400) == BUTTON_NONE);
  pinLevels[BUTTON_UP_PIN] = HIGH;
  assert(sampleAt(1420) == BUTTON_NONE);
  assert(sampleAt(1440) == BUTTON_NONE);

  pinLevels[BUTTON_DOWN_PIN] = LOW;
  assert(sampleAt(1500) == BUTTON_NONE);
  assert(sampleAt(1520) == BUTTON_DOWN_CLICK);
  assert(sampleAt(1600) == BUTTON_NONE);

  assert(digitalReadCalls[12] == 0);
  std::cout << "button scenarios passed: ACTION click/hold/bounce, no D12\n";
  return 0;
}
