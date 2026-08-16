#include <cassert>
#include <cstdint>
#include <iostream>

#include "../buttons.cpp"


static uint32_t simulatedMillis = 0;
static uint8_t pinLevels[32] = {};


uint32_t millis() { return simulatedMillis; }
uint32_t micros() { return simulatedMillis * 1000UL; }


void pinMode(uint8_t, uint8_t) {}


int digitalRead(uint8_t pin)
{
  return pinLevels[pin];
}


static ButtonEvent sampleAt(uint32_t timeMillis)
{
  simulatedMillis = timeMillis;
  return readButtonEvent();
}


static void setRight(uint8_t level)
{
  pinLevels[BUTTON_RIGHT_PIN] = level;
}


int main()
{
  for (uint8_t pin = 0; pin < 32; pin++) pinLevels[pin] = HIGH;
  buttonsInit();

  // A real switch can chatter LOW/HIGH several times. Only the final LOW
  // that remains unchanged for 25 ms is one physical RIGHT press.
  setRight(LOW);
  assert(sampleAt(1) == BUTTON_NONE);
  setRight(HIGH);
  assert(sampleAt(5) == BUTTON_NONE);
  setRight(LOW);
  assert(sampleAt(9) == BUTTON_NONE);
  assert(sampleAt(33) == BUTTON_NONE);
  assert(sampleAt(34) == BUTTON_RIGHT_CLICK);

  // Keeping RIGHT held cannot create repeats because no new press edge occurs.
  uint8_t heldEvents = 0;
  for (uint32_t time = 35; time <= 1000; time += 5)
  {
    if (sampleAt(time) != BUTTON_NONE) heldEvents++;
  }
  assert(heldEvents == 0);

  // A debounced release creates no event, but it arms one later press.
  setRight(HIGH);
  assert(sampleAt(1001) == BUTTON_NONE);
  setRight(LOW);
  assert(sampleAt(1005) == BUTTON_NONE);
  setRight(HIGH);
  assert(sampleAt(1009) == BUTTON_NONE);
  assert(sampleAt(1034) == BUTTON_NONE);

  setRight(LOW);
  assert(sampleAt(1100) == BUTTON_NONE);
  assert(sampleAt(1124) == BUTTON_NONE);
  assert(sampleAt(1125) == BUTTON_RIGHT_CLICK);
  assert(sampleAt(1500) == BUTTON_NONE);

  std::cout << "button scenarios passed: one RIGHT event per debounced press\n";
  return 0;
}
