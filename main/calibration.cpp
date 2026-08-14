#include "calibration.h"

#include <avr/pgmspace.h>

#include "config.h"
#include "hardware.h"
#include "settings.h"


// Default: 2000 ms. Placement time before calibration motors move. Increase
// if more setup time is needed; decrease only when the robot is already held.
constexpr uint16_t CALIBRATION_PLACEMENT_MS = 2000;

// Default: 110 PWM. Pivot speed during calibration. Increase if motors stall;
// decrease if the array sweeps across the line too quickly.
constexpr uint8_t CALIBRATION_PWM = 110;

// Default: 650 ms per sweep. Controls the angular coverage of each pass.
// Increase if edge sensors never cross both surfaces; decrease if cables bind.
constexpr uint16_t CALIBRATION_SWEEP_MS = 650;

// Default: 120 ms. Pause between direction changes. Increase to let the chassis
// settle; decrease to finish calibration sooner.
constexpr uint8_t CALIBRATION_PAUSE_MS = 120;

// Default: 80 ADC. Minimum observed black/white range for every sensor. Increase
// to reject weak sensors; decrease only for a verified low-contrast surface.
constexpr uint8_t CALIBRATION_MIN_RANGE = 80;


enum CalibrationPhase : uint8_t
{
  CAL_PHASE_IDLE,
  CAL_PHASE_PLACEMENT,
  CAL_PHASE_SWEEP,
  CAL_PHASE_PAUSE,
  CAL_PHASE_DONE
};


static const int8_t SWEEP_DIRECTIONS[4] PROGMEM = { -1, 1, 1, -1 };
static uint16_t minimumValues[SENSOR_COUNT];
static uint16_t maximumValues[SENSOR_COUNT];
static uint16_t newThresholds[SENSOR_COUNT];
static CalibrationPhase phase = CAL_PHASE_IDLE;
static uint32_t phaseStartedAt = 0;
static uint8_t sweepIndex = 0;


void calibrationStart()
{
  stopMotors();
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    minimumValues[i] = 1023;
    maximumValues[i] = 0;
  }

  sweepIndex = 0;
  phase = CAL_PHASE_PLACEMENT;
  phaseStartedAt = millis();
}


void calibrationCancel()
{
  stopMotors();
  phase = CAL_PHASE_IDLE;
}


CalibrationResult calibrationTick(uint16_t sensorValues[])
{
  const uint32_t now = millis();

  if (phase == CAL_PHASE_PLACEMENT)
  {
    stopMotors();
    if (now - phaseStartedAt >= CALIBRATION_PLACEMENT_MS)
    {
      phase = CAL_PHASE_SWEEP;
      phaseStartedAt = now;
    }
    return CALIBRATION_RUNNING;
  }

  if (phase == CAL_PHASE_SWEEP)
  {
    readSensors(sensorValues);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
      minimumValues[i] = min(minimumValues[i], sensorValues[i]);
      maximumValues[i] = max(maximumValues[i], sensorValues[i]);
    }

    const int8_t direction =
        (int8_t)pgm_read_byte(&SWEEP_DIRECTIONS[sweepIndex]);
    moveLFR(direction * CALIBRATION_PWM,
            -direction * CALIBRATION_PWM);

    if (now - phaseStartedAt >= CALIBRATION_SWEEP_MS)
    {
      stopMotors();
      phase = CAL_PHASE_PAUSE;
      phaseStartedAt = now;
    }
    return CALIBRATION_RUNNING;
  }

  if (phase == CAL_PHASE_PAUSE)
  {
    stopMotors();
    if (now - phaseStartedAt < CALIBRATION_PAUSE_MS)
    {
      return CALIBRATION_RUNNING;
    }

    sweepIndex++;
    if (sweepIndex < 4)
    {
      phase = CAL_PHASE_SWEEP;
      phaseStartedAt = now;
      return CALIBRATION_RUNNING;
    }

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
      if (maximumValues[i] - minimumValues[i] < CALIBRATION_MIN_RANGE)
      {
        phase = CAL_PHASE_DONE;
        return CALIBRATION_FAILED;
      }
      newThresholds[i] =
          (maximumValues[i] + minimumValues[i]) / 2;
    }

    settingsApplyCalibration(newThresholds);
    phase = CAL_PHASE_DONE;
    return CALIBRATION_SUCCEEDED;
  }

  return CALIBRATION_FAILED;
}
