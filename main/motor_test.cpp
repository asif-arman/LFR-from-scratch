#include "motor_test.h"

#include "hardware.h"


// Default: 2000 ms. Gives time to lift the wheels before the test. Increase if
// setup takes longer; decrease only when the chassis is already secured.
constexpr uint16_t MOTOR_TEST_PLACEMENT_MS = 2000;

// Default: 90 PWM. Tests that each motor starts in each direction. Increase if
// a motor cannot overcome static friction; decrease for a delicate drive train.
constexpr uint8_t MOTOR_TEST_PWM = 90;

// Default: 300 ms. Duration of each single-motor direction test. Increase to
// make direction easier to observe; decrease to reduce mechanical motion.
constexpr uint16_t MOTOR_TEST_STEP_MS = 300;

// Default: 120 ms. Motor-off pause between test directions. Increase to make
// steps more distinct; decrease to complete the test sooner.
constexpr uint8_t MOTOR_TEST_PAUSE_MS = 120;


static uint32_t phaseStartedAt = 0;
static uint8_t phase = 0;
static bool active = false;


void motorTestStart()
{
  stopMotors();
  phase = 0;
  phaseStartedAt = millis();
  active = true;
}


void motorTestCancel()
{
  stopMotors();
  active = false;
}


bool motorTestTick()
{
  if (!active)
  {
    stopMotors();
    return true;
  }

  const uint32_t now = millis();
  if (phase == 0)
  {
    stopMotors();
    if (now - phaseStartedAt >= MOTOR_TEST_PLACEMENT_MS)
    {
      phase = 1;
      phaseStartedAt = now;
    }
    return false;
  }

  const bool pausePhase = (phase & 1) == 0;
  if (pausePhase)
  {
    stopMotors();
    if (now - phaseStartedAt >= MOTOR_TEST_PAUSE_MS)
    {
      phase++;
      phaseStartedAt = now;
    }
    return false;
  }

  // Only one motor is energized. The diagnostic never issues a blind
  // both-motors-forward command, so it cannot bypass the navigation probe.
  switch (phase)
  {
    case 1: moveLFR(MOTOR_TEST_PWM, 0); break;
    case 3: moveLFR(-MOTOR_TEST_PWM, 0); break;
    case 5: moveLFR(0, MOTOR_TEST_PWM); break;
    case 7: moveLFR(0, -MOTOR_TEST_PWM); break;
    default:
      stopMotors();
      active = false;
      return true;
  }

  if (now - phaseStartedAt >= MOTOR_TEST_STEP_MS)
  {
    stopMotors();
    if (phase == 7)
    {
      active = false;
      return true;
    }
    phase++;
    phaseStartedAt = now;
  }
  return false;
}
