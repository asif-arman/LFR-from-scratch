#include "robot.h"
#include "config.h"
#include "settings.h"


// Convert the 14 analog readings into a 14-bit pattern.
//
// Bit 0  represents S0.
// Bit 13 represents S13.
//
// A bit becomes 1 when its sensor reading is above
// the adjustable threshold.
uint16_t makeSensorPattern(
    const uint16_t values[]
)
{
  uint16_t pattern = 0;

  for (
      uint8_t i = 0;
      i < SENSOR_COUNT;
      i++
  )
  {
    if (values[i] > settingsThresholdForSensor(i))
    {
      pattern |= (uint16_t)1 << i;
    }
  }

  return pattern;
}


// Find the average position of all sensors that
// currently detect the black line.
//
// Returns -1 when no sensor detects the line.
int16_t calculateLinePosition(
    uint16_t pattern
)
{
  uint16_t positionSum = 0;

  uint8_t activeCount = 0;

  for (
      uint8_t i = 0;
      i < SENSOR_COUNT;
      i++
  )
  {
    if (pattern & ((uint16_t)1 << i))
    {
      positionSum +=
          i * SENSOR_POSITION_SCALE;

      activeCount++;
    }
  }

  if (activeCount == 0)
  {
    return -1;
  }

  return positionSum / activeCount;
}


// A centered line produces an error near zero.
int16_t calculateLineError(
    int16_t linePosition
)
{
  return LINE_CENTER - linePosition;
}


// Fixed-point proportional calculation.
//
// Example:
//
// kpX100 = 30
//
// error * 30 / 100
//
// This acts like:
//
// error * 0.30
int16_t calculateProportionalCorrection(
    int16_t error
)
{
  return (
      (int32_t)error *
      kpX100
  ) / 100;
}


// Fixed-point derivative calculation.
int16_t calculateDerivativeCorrection(
    int16_t currentError,
    int16_t previousError
)
{
  const int16_t errorChange =
      currentError - previousError;

  return (
      (int32_t)errorChange *
      kdX100
  ) / 100;
}
