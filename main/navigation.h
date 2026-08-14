#pragma once

#include <Arduino.h>


enum NavigationResult : uint8_t
{
  NAVIGATION_ACTIVE,
  NAVIGATION_FINISHED,
  NAVIGATION_LOST
};


void navigationStart();
void navigationStop();

// One non-blocking control tick. It always takes a fresh 14-sensor frame.
NavigationResult navigationTick(uint16_t sensorValues[]);

uint16_t navigationWorstSensorFrameMicros();
uint16_t navigationWorstControlTickMicros();
