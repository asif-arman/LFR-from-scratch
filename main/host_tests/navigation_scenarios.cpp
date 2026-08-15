#include <cassert>
#include <cstdint>
#include <iostream>

#include "../motor_guard.h"
#include "../navigation.cpp"


constexpr uint16_t HOST_SENSOR_FRAME_US = 1550;
static uint32_t simulatedMicros = 0;
static uint16_t sensorFrame[SENSOR_COUNT] = {};
static int16_t commandedLeft = 0;
static int16_t commandedRight = 0;
static bool brakeWasCalled = false;
static uint16_t stopCallCount = 0;
static MotorDirectionGuardState hostLeftGuard = { 0, true, 0 };
static MotorDirectionGuardState hostRightGuard = { 0, true, 0 };

uint8_t kpX100 = 30;
uint8_t kdX100 = 8;
uint8_t baseSpeed = 160;
uint16_t sensorThreshold = 400;
uint8_t routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
bool boxMode = false;
bool sensorCalibrationValid = true;
uint16_t sensorMinimums[SENSOR_COUNT] = {};
uint16_t sensorMaximums[SENSOR_COUNT] = {};


uint32_t millis() { return simulatedMicros / 1000UL; }
uint32_t micros() { return simulatedMicros; }


void moveLFR(int16_t left, int16_t right)
{
  commandedLeft = motorDirectionGuardCommand(
      hostLeftGuard, left, simulatedMicros);
  commandedRight = motorDirectionGuardCommand(
      hostRightGuard, right, simulatedMicros);
}


void stopMotors()
{
  stopCallCount++;
  moveLFR(0, 0);
}


void brakeMotors()
{
  motorDirectionGuardReset(hostLeftGuard);
  motorDirectionGuardReset(hostRightGuard);
  commandedLeft = 0;
  commandedRight = 0;
  brakeWasCalled = true;
}


void motorInit() {}
void sensorInit() {}


void readSensors(uint16_t values[])
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) values[i] = sensorFrame[i];
  simulatedMicros += HOST_SENSOR_FRAME_US;
}


uint16_t settingsBlackStrength(uint8_t sensor, uint16_t value)
{
  if (!sensorCalibrationValid)
    return value > sensorThreshold ? 1000 : 0;
  const uint16_t minimum = sensorMinimums[sensor];
  const uint16_t maximum = sensorMaximums[sensor];
  if (value <= minimum) return 0;
  if (value >= maximum) return 1000;
  return ((uint32_t)(value - minimum) * 1000UL) / (maximum - minimum);
}


uint16_t makeSensorPattern(const uint16_t values[])
{
  uint16_t pattern = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (settingsBlackStrength(i, values[i]) >= 500)
      pattern |= (uint16_t)1 << i;
  }
  return pattern;
}


int16_t calculateLinePosition(uint16_t pattern)
{
  uint16_t sum = 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (pattern & ((uint16_t)1 << i))
    {
      sum += i * SENSOR_POSITION_SCALE;
      count++;
    }
  }
  return count == 0 ? -1 : sum / count;
}


int16_t calculateLineError(int16_t position)
{
  return LINE_CENTER - position;
}


int16_t calculateProportionalCorrection(int16_t error)
{
  return ((int32_t)error * kpX100) / 100;
}


int16_t calculateDerivativeCorrection(int16_t current, int16_t previous)
{
  return ((int32_t)(current - previous) * kdX100) / 100;
}


RouteDirection settingsPriorityAt(uint8_t index)
{
  static const RouteDirection orders[ROUTE_PRIORITY_COUNT][3] =
  {
    { ROUTE_STRAIGHT, ROUTE_LEFT, ROUTE_RIGHT },
    { ROUTE_STRAIGHT, ROUTE_RIGHT, ROUTE_LEFT },
    { ROUTE_LEFT, ROUTE_STRAIGHT, ROUTE_RIGHT },
    { ROUTE_LEFT, ROUTE_RIGHT, ROUTE_STRAIGHT },
    { ROUTE_RIGHT, ROUTE_STRAIGHT, ROUTE_LEFT },
    { ROUTE_RIGHT, ROUTE_LEFT, ROUTE_STRAIGHT }
  };
  return index < 3 ? orders[routePriority][index] : ROUTE_NONE;
}


void settingsLoad() {}
void settingsSaveIfChanged() {}
void settingsUseGlobalThreshold() {}
void settingsApplyCalibration(const uint16_t[], const uint16_t[]) {}
uint16_t settingsThresholdForSensor(uint8_t sensor)
{
  return (sensorMinimums[sensor] + sensorMaximums[sensor]) / 2;
}


static NavigationResult tickFrame(const uint16_t frame[])
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) sensorFrame[i] = frame[i];
  uint16_t values[SENSOR_COUNT] = {};
  const NavigationResult result = navigationTick(values);
  if (CONTROL_PERIOD_US > HOST_SENSOR_FRAME_US)
    simulatedMicros += CONTROL_PERIOD_US - HOST_SENSOR_FRAME_US;
  return result;
}


static NavigationResult tick(uint16_t pattern)
{
  uint16_t frame[SENSOR_COUNT];
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    frame[i] = (pattern & ((uint16_t)1 << i)) ? 850 : 100;
  return tickFrame(frame);
}


static void repeat(uint16_t pattern, uint16_t count)
{
  while (count-- != 0) tick(pattern);
}


static uint16_t oneSensor(uint8_t sensor)
{
  return (uint16_t)1 << sensor;
}


static void fillFrame(uint16_t frame[], uint16_t value)
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) frame[i] = value;
}


static void resetScenario(bool enableBoxMode = false)
{
  simulatedMicros = 0;
  fillFrame(sensorFrame, 100);
  commandedLeft = 0;
  commandedRight = 0;
  brakeWasCalled = false;
  stopCallCount = 0;
  kpX100 = 30;
  kdX100 = 8;
  baseSpeed = 160;
  routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
  boxMode = enableBoxMode;
  sensorCalibrationValid = true;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorMinimums[i] = 100;
    sensorMaximums[i] = 850;
  }
  motorDirectionGuardReset(hostLeftGuard);
  motorDirectionGuardReset(hostRightGuard);
  navigationStart();
  stopCallCount = 0;
}


static void clearStartAndArmFinish(uint16_t centered)
{
  repeat(ALL_SENSOR_MASK, 3);
  repeat(centered, START_EXIT_CONFIRM_TICKS);
  assert(startCleared && state == STATE_FOLLOW && !finishArmed);
  repeat(centered, FINISH_ARM_TICKS);
  assert(finishArmed);
}


static void driveToUTurn(uint16_t centered)
{
  tick(centered);
  tick(0);
  repeat(0, LOSS_PROBE_TICKS);
  assert(state == STATE_U_TURN);
}


int main()
{
  const uint16_t centered = oneSensor(6) | oneSensor(7);
  const uint16_t leftLine = oneSensor(9);
  const uint16_t rightLine = oneSensor(4);
  const uint16_t inverseEntry = ALL_SENSOR_MASK ^ centered;

  // Endpoint normalization clamps and scales without floating point.
  resetScenario();
  assert(settingsBlackStrength(0, 0) == 0);
  assert(settingsBlackStrength(0, 100) == 0);
  assert(settingsBlackStrength(0, 475) == 500);
  assert(settingsBlackStrength(0, 850) == 1000);
  assert(settingsBlackStrength(0, 1023) == 1000);

  // Centered, left and right errors drive analog-weighted PD correctly.
  resetScenario();
  assert(tick(centered) == NAVIGATION_ACTIVE);
  assert(commandedLeft == baseSpeed && commandedRight == baseSpeed);
  resetScenario();
  tick(leftLine);
  assert(previousError < 0 && commandedLeft < commandedRight);
  assert(commandedLeft >= 0 && commandedRight >= 0);
  resetScenario();
  tick(rightLine);
  assert(previousError > 0 && commandedLeft > commandedRight);
  assert(commandedLeft >= 0 && commandedRight >= 0);

  // Analog centroid moves smoothly within one unchanged binary group.
  resetScenario();
  kdX100 = 0;
  uint16_t analogFrame[SENSOR_COUNT];
  fillFrame(analogFrame, 100);
  analogFrame[5] = 500;
  analogFrame[6] = 850;
  tickFrame(analogFrame);
  const int16_t firstError = previousError;
  analogFrame[5] = 650;
  analogFrame[6] = 750;
  tickFrame(analogFrame);
  assert(previousError > firstError);

  // Approach speed decreases monotonically as error and outward trend grow.
  resetScenario();
  kpX100 = 0;
  kdX100 = 0;
  tick(centered);
  const int16_t centeredSpeed = commandedLeft;
  tick(oneSensor(8));
  const int16_t mediumSpeed = commandedLeft;
  tick(oneSensor(10));
  const int16_t largeSpeed = commandedLeft;
  tick(oneSensor(12));
  const int16_t outwardSpeed = commandedLeft;
  assert(centeredSpeed > mediumSpeed && mediumSpeed > largeSpeed &&
         largeSpeed > outwardSpeed);
  assert(outwardTrendCount >= TURN_OUTWARD_CONFIRM);
  assert((commandedLeft == 0 || commandedLeft >= LEFT_MOTOR_EFFECTIVE_MIN_PWM));
  assert((commandedRight == 0 || commandedRight >= RIGHT_MOTOR_EFFECTIVE_MIN_PWM));
  assert(slewFollowPwm(0, 160, 90) == 90);
  assert(slewFollowPwm(160, 90, 90) == 90);
  assert(slewFollowPwm(160, 0, 90) == 0);
  assert(slewFollowPwm(160, -110, 90) == -110);

  // A severe error requests bounded inside-wheel reverse immediately. The
  // common direction guard produces zero first, then allows the signed target.
  resetScenario();
  tick(centered);
  tick(oneSensor(0));
  assert(previousFollowRight == -FOLLOW_REVERSE_MAX_PWM);
  assert(commandedLeft > 0 && commandedRight == 0);
  tick(oneSensor(0));
  assert(commandedLeft > 0 && commandedRight < 0);
  assert(-commandedRight >= RIGHT_MOTOR_EFFECTIVE_MIN_PWM &&
         -commandedRight <= FOLLOW_REVERSE_MAX_PWM);

  resetScenario();
  tick(centered);
  tick(oneSensor(13));
  assert(previousFollowLeft == -FOLLOW_REVERSE_MAX_PWM);
  assert(commandedLeft == 0 && commandedRight > 0);
  tick(oneSensor(13));
  assert(commandedLeft < 0 && commandedRight > 0);
  assert(-commandedLeft >= LEFT_MOTOR_EFFECTIVE_MIN_PWM &&
         -commandedLeft <= FOLLOW_REVERSE_MAX_PWM);

  // A short gap retains bounded steering and confirms the full-array return.
  resetScenario();
  kdX100 = 0;
  tick(leftLine);
  const bool wasTurningLeft = commandedLeft < commandedRight;
  tick(0);
  assert(state == STATE_FORWARD_PROBE && probeReason == PROBE_LOST_LINE);
  assert((gapProbeLeft < gapProbeRight) == wasTurningLeft);
  assert(absoluteError(gapProbeLeft - gapProbeRight) <= GAP_STEERING_MAX_DELTA);
  tick(leftLine);
  assert(state == STATE_FORWARD_PROBE);
  repeat(leftLine, LINE_CONFIRM_TICKS - 1);
  assert(state == STATE_FOLLOW);

  // Multiple dotted gaps are independently crossed without stopping.
  repeat(centered, 2);
  for (uint8_t gap = 0; gap < 3; gap++)
  {
    repeat(0, 2);
    repeat(centered, LINE_CONFIRM_TICKS);
    assert(state == STATE_FOLLOW && terminalResult == NAVIGATION_ACTIVE);
  }

  // An outward-moving left/right edge loss starts the matching quick turn
  // immediately instead of consuming the centred dotted-gap allowance.
  resetScenario();
  tick(oneSensor(8));
  tick(oneSensor(10));
  tick(oneSensor(12));
  tick(0);
  assert(state == STATE_TURN_TO_ROUTE && selectedRoute == ROUTE_LEFT);
  assert(commandedLeft < 0 && commandedRight > 0);

  resetScenario();
  tick(oneSensor(5));
  tick(oneSensor(3));
  tick(oneSensor(1));
  tick(0);
  assert(state == STATE_TURN_TO_ROUTE && selectedRoute == ROUTE_RIGHT);
  assert(commandedLeft > 0 && commandedRight < 0);

  // Persistent centred loss consumes the gap allowance, then starts a bounded
  // history-directed U-turn rather than an endless reverse/search sequence.
  resetScenario();
  driveToUTurn(centered);
  tick(0);
  tick(0); // lets the 2.5 ms common reversal guard expire
  assert(commandedLeft > 0 && commandedRight < 0);
  assert(commandedLeft == UTURN_PWM && commandedRight == -UTURN_PWM);

  // The original center line is ignored before the U-turn minimum guard.
  resetScenario();
  driveToUTurn(centered);
  repeat(centered, UTURN_MIN_TICKS - 1);
  assert(state == STATE_U_TURN);
  tick(centered);
  assert(state == STATE_U_TURN && centerStableCount == 1);
  tick(centered);
  assert(state == STATE_FOLLOW);

  // An unsuccessful U-turn has one bounded timeout and a terminal lost result.
  resetScenario();
  driveToUTurn(centered);
  repeat(0, UTURN_TIMEOUT_TICKS - 1);
  assert(state == STATE_U_TURN);
  assert(tick(0) == NAVIGATION_LOST);
  assert(state == STATE_STOPPED && commandedLeft == 0 && commandedRight == 0);
  assert(tick(0) == NAVIGATION_LOST);

  // BOX OFF supports a short inverse section without a blind 500 ms lockout:
  // normal -> inverse, three stable inverse frames, then inverse -> normal.
  resetScenario(false);
  tick(centered);
  tick(inverseEntry);
  tick(inverseEntry);
  assert(lineIsInverse && state == STATE_FOLLOW);
  assert(!inverseTransitionArmed);
  repeat(inverseEntry, INVERSE_REARM_STABLE_TICKS);
  assert(inverseTransitionArmed);
  tick(centered);
  assert(lineIsInverse && state == STATE_FORWARD_PROBE);
  tick(centered);
  assert(!lineIsInverse && state == STATE_FOLLOW);
  assert(commandedLeft > 0 && commandedRight > 0);
  tick(rightLine);
  assert(previousError > 0 && commandedLeft > commandedRight);

  // Unrelated full-black geometry never becomes a polarity transition/finish.
  resetScenario(false);
  repeat(ALL_SENSOR_MASK, FINISH_CONFIRM_TICKS + 4);
  assert(terminalResult == NAVIGATION_ACTIVE && state != STATE_STOPPED);
  assert(!lineIsInverse);

  // BOX ON isolates the start box from junction/loss/finish logic.
  resetScenario(true);
  assert(state == STATE_START_BOX && !startCleared && !lineIsInverse);
  repeat(ALL_SENSOR_MASK, FINISH_CONFIRM_TICKS + 3);
  assert(state == STATE_START_BOX && !junctionActive && !finishArmed);
  repeat(centered, START_EXIT_CONFIRM_TICKS);
  assert(startCleared && state == STATE_FOLLOW);

  // Starting directly on the outgoing line is also valid.
  resetScenario(true);
  repeat(centered, START_EXIT_CONFIRM_TICKS);
  assert(startCleared && state == STATE_FOLLOW);
  repeat(inverseEntry, INVERSE_CONFIRM_TICKS + 2);
  assert(!lineIsInverse); // inverse processing is disabled for this run

  // Finish arms only after ordinary travel; a brief cross cannot finish.
  resetScenario(true);
  clearStartAndArmFinish(centered);
  repeat(ALL_SENSOR_MASK, FINISH_CONFIRM_TICKS - 2);
  repeat(centered, LINE_CONFIRM_TICKS + JUNCTION_EXIT_CONFIRM_TICKS);
  assert(terminalResult == NAVIGATION_ACTIVE && state != STATE_STOPPED);

  // A persistent near-full box after arming produces the terminal result.
  resetScenario(true);
  clearStartAndArmFinish(centered);
  NavigationResult finishResult = NAVIGATION_ACTIVE;
  for (uint8_t i = 0; i < FINISH_CONFIRM_TICKS; i++)
    finishResult = tick(ALL_SENSOR_MASK);
  assert(finishResult == NAVIGATION_FINISHED);
  assert(state == STATE_STOPPED && commandedLeft == 0 && commandedRight == 0);

  const uint16_t leftAndStraight = 0x3FC0;
  const uint16_t rightAndStraight = 0x00FF;

  // Sensor-driven angled approaches select signed left/right pivots.
  resetScenario();
  tick(oneSensor(8));
  repeat(leftAndStraight, 2);
  repeat(centered, LINE_CONFIRM_TICKS);
  assert(selectedRoute == ROUTE_LEFT && state == STATE_TURN_TO_ROUTE);
  tick(0);
  assert(commandedLeft == -TURN_PWM && commandedRight == TURN_PWM);

  resetScenario();
  tick(oneSensor(5));
  repeat(rightAndStraight, 2);
  repeat(centered, LINE_CONFIRM_TICKS);
  assert(selectedRoute == ROUTE_RIGHT && state == STATE_TURN_TO_ROUTE);
  tick(0);
  assert(commandedLeft == TURN_PWM && commandedRight == -TURN_PWM);

  // A centred approach to a cross preserves geometric continuity.
  resetScenario();
  tick(centered);
  repeat(ALL_SENSOR_MASK, 2);
  repeat(centered, LINE_CONFIRM_TICKS);
  assert(selectedRoute == ROUTE_STRAIGHT && state == STATE_ROUTE_COMMIT);

  // With no straight exit, all six configured orders select the first
  // available left/right route in their public priority table.
  for (uint8_t priority = 0; priority < ROUTE_PRIORITY_COUNT; priority++)
  {
    resetScenario();
    routePriority = priority;
    tick(centered);
    repeat(ALL_SENSOR_MASK, 2);
    tick(0);
    repeat(0, PROBE_BRAKE_TICKS);

    RouteDirection expected = ROUTE_NONE;
    for (uint8_t rank = 0; rank < 3 && expected == ROUTE_NONE; rank++)
    {
      const RouteDirection candidate = settingsPriorityAt(rank);
      if (candidate == ROUTE_LEFT || candidate == ROUTE_RIGHT)
        expected = candidate;
    }
    assert(selectedRoute == expected);
  }

  // Sensor-driven left T, right T, and three-way/cross classification.
  resetScenario();
  tick(centered);
  repeat(leftAndStraight, 2);
  repeat(centered, LINE_CONFIRM_TICKS);
  assert(junctionAvailableRoutes == (ROUTE_LEFT | ROUTE_STRAIGHT));
  resetScenario();
  tick(centered);
  repeat(rightAndStraight, 2);
  repeat(centered, LINE_CONFIRM_TICKS);
  assert(junctionAvailableRoutes == (ROUTE_RIGHT | ROUTE_STRAIGHT));
  resetScenario();
  tick(centered);
  repeat(ALL_SENSOR_MASK, 2);
  repeat(centered, LINE_CONFIRM_TICKS);
  assert(junctionAvailableRoutes ==
         (ROUTE_LEFT | ROUTE_STRAIGHT | ROUTE_RIGHT));

  // A separated angled crossing accumulates multiple route regions.
  resetScenario();
  const uint16_t separated = oneSensor(4) | oneSensor(5) | oneSensor(7);
  repeat(separated, JUNCTION_CONFIRM_TICKS);
  assert(junctionActive &&
         junctionAvailableRoutes == (ROUTE_RIGHT | ROUTE_STRAIGHT));

  // A route rearms after stable narrow exit, allowing a nearby junction.
  resetScenario();
  tick(centered);
  repeat(leftAndStraight, 2);
  repeat(centered, LINE_CONFIRM_TICKS);
  assert(state == STATE_ROUTE_COMMIT);
  repeat(centered, JUNCTION_EXIT_CONFIRM_TICKS);
  assert(state == STATE_FOLLOW && !junctionActive);
  repeat(separated, JUNCTION_CONFIRM_TICKS);
  assert(junctionActive);

  // The common motor guard still inserts a precise zero interval on reversal.
  resetScenario();
  moveLFR(-TURN_PWM, TURN_PWM);
  assert(commandedLeft == -TURN_PWM && commandedRight == TURN_PWM);
  moveLFR(TURN_PWM, -TURN_PWM);
  assert(commandedLeft == 0 && commandedRight == 0);
  simulatedMicros += MOTOR_DIRECTION_GUARD_US;
  moveLFR(TURN_PWM, -TURN_PWM);
  assert(commandedLeft == TURN_PWM && commandedRight == -TURN_PWM);

  // The measured host sensor/control work remains inside one control period.
  resetScenario();
  repeat(centered, 5);
  assert(navigationWorstSensorFrameMicros() == HOST_SENSOR_FRAME_US);
  assert(navigationWorstControlTickMicros() == HOST_SENSOR_FRAME_US);
  assert(navigationWorstControlTickMicros() <= CONTROL_PERIOD_US);

  std::cout << "navigation scenarios passed; sensor/control "
            << navigationWorstSensorFrameMicros() << "/"
            << navigationWorstControlTickMicros() << " us\n";
  return 0;
}
