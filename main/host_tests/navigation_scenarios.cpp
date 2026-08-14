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
bool sensorCalibrationValid = true;
uint16_t sensorThresholds[SENSOR_COUNT] = {};


uint32_t millis()
{
  return simulatedMicros / 1000UL;
}


uint32_t micros()
{
  return simulatedMicros;
}


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


uint16_t makeSensorPattern(const uint16_t values[])
{
  uint16_t pattern = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (values[i] > settingsThresholdForSensor(i))
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
void settingsApplyCalibration(const uint16_t[]) {}
uint16_t settingsThresholdForSensor(uint8_t) { return sensorThreshold; }


static NavigationResult tickFrame(const uint16_t frame[],
                                  uint16_t extraMs = 0)
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) sensorFrame[i] = frame[i];
  uint16_t values[SENSOR_COUNT] = {};
  const NavigationResult result = navigationTick(values);
  if (CONTROL_PERIOD_US > HOST_SENSOR_FRAME_US)
    simulatedMicros += CONTROL_PERIOD_US - HOST_SENSOR_FRAME_US;
  simulatedMicros += (uint32_t)extraMs * 1000UL;
  assert(result == NAVIGATION_ACTIVE);
  return result;
}


static NavigationResult tick(uint16_t pattern, uint16_t extraMs = 0)
{
  uint16_t frame[SENSOR_COUNT];
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    frame[i] = (pattern & ((uint16_t)1 << i)) ? 850 : 100;
  return tickFrame(frame, extraMs);
}


static void resetScenario()
{
  simulatedMicros = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) sensorFrame[i] = 100;
  commandedLeft = 0;
  commandedRight = 0;
  brakeWasCalled = false;
  kpX100 = 30;
  kdX100 = 8;
  baseSpeed = 160;
  routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
  motorDirectionGuardReset(hostLeftGuard);
  motorDirectionGuardReset(hostRightGuard);
  navigationStart();
  stopCallCount = 0;
}


static uint16_t oneSensor(uint8_t sensor)
{
  return (uint16_t)1 << sensor;
}


static void fillFrame(uint16_t frame[], uint16_t value)
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) frame[i] = value;
}


static void observeWideThen(uint16_t exitPattern)
{
  tick(ALL_SENSOR_MASK);
  tick(ALL_SENSOR_MASK);
  tick(exitPattern);
}


static void finishNarrowConfirmation(uint16_t narrow)
{
  tick(narrow);
  tick(narrow);
}


static void verifyAngledLoss(bool left, uint8_t outerSensor)
{
  resetScenario();
  tick(oneSensor(left ? 7 : 6));
  tick(oneSensor(left ? 8 : 5));
  tick(oneSensor(outerSensor));
  assert(outwardTrendCount >= TURN_OUTWARD_CONFIRM);
  assert(commandedLeft < baseSpeed || commandedRight < baseSpeed);

  tick(0);
  assert(state == STATE_TURN_TO_ROUTE && !brakeWasCalled &&
         stopCallCount == 1);
  tick(0);
  if (left)
  {
    assert(commandedLeft < 0 && commandedRight > 0);
  }
  else
  {
    assert(commandedLeft > 0 && commandedRight < 0);
  }
}


int main()
{
  const uint16_t centered = oneSensor(6) | oneSensor(7);
  const uint16_t inverseEntry = ALL_SENSOR_MASK ^ centered;
  const uint16_t leftAndStraight = 0x3FC0;

  // Real analog strengths move the PD error gradually within one binary group.
  resetScenario();
  kdX100 = 0;
  uint16_t analogFrame[SENSOR_COUNT];
  fillFrame(analogFrame, 100);
  analogFrame[5] = 450;
  analogFrame[6] = 900;
  tickFrame(analogFrame);
  const int16_t firstError = previousError;
  const int16_t firstLeft = commandedLeft;
  const int16_t firstRight = commandedRight;
  analogFrame[5] = 550;
  analogFrame[6] = 850;
  tickFrame(analogFrame);
  const int16_t secondError = previousError;
  const int16_t secondLeft = commandedLeft;
  const int16_t secondRight = commandedRight;
  analogFrame[5] = 650;
  analogFrame[6] = 750;
  tickFrame(analogFrame);
  assert(firstError < secondError && secondError < previousError);
  assert(firstLeft < secondLeft && secondLeft < commandedLeft);
  assert(firstRight > secondRight && secondRight > commandedRight);
  assert(commandedLeft - secondLeft <= FOLLOW_PWM_SLEW_STEP);
  assert(secondRight - commandedRight <= FOLLOW_PWM_SLEW_STEP);

  // A centred line stays equal and never pulses or stops.
  resetScenario();
  fillFrame(analogFrame, 100);
  analogFrame[6] = 850;
  analogFrame[7] = 850;
  tickFrame(analogFrame);
  assert(commandedLeft == baseSpeed && commandedRight == baseSpeed);
  for (uint8_t frame = 0; frame < 5; frame++)
  {
    tickFrame(analogFrame);
    assert(commandedLeft == baseSpeed && commandedRight == baseSpeed);
  }
  assert(navigationWorstSensorFrameMicros() == HOST_SENSOR_FRAME_US);
  assert(navigationWorstControlTickMicros() == HOST_SENSOR_FRAME_US);
  assert(navigationWorstControlTickMicros() <= CONTROL_PERIOD_US);

  // Hard curves can coast the inside wheel; corner slowdown keeps authority.
  resetScenario();
  kpX100 = 70;
  kdX100 = 0;
  tick(centered);
  for (uint8_t frame = 0; frame < 8; frame++) tick(oneSensor(0));
  assert(commandedLeft == 255 && commandedRight == 0);
  int16_t previousApplied = commandedRight;
  for (uint8_t frame = 0; frame < 3; frame++)
  {
    tick(centered);
    assert(commandedRight - previousApplied <= FOLLOW_PWM_SLEW_STEP);
    previousApplied = commandedRight;
  }
  assert(commandedRight >= RIGHT_MOTOR_EFFECTIVE_MIN_PWM);

  resetScenario();
  kpX100 = 70;
  kdX100 = 0;
  tick(oneSensor(6));
  tick(oneSensor(5));
  followMixerValid = false;
  tick(oneSensor(3));
  assert(outwardTrendCount >= TURN_OUTWARD_CONFIRM);
  assert(commandedLeft == 255 && commandedRight == 0);

  // NORMAL and INVERSE analog weighting produce symmetric steering.
  resetScenario();
  kdX100 = 0;
  fillFrame(analogFrame, 100);
  analogFrame[5] = 650;
  analogFrame[6] = 850;
  tickFrame(analogFrame);
  const int16_t normalLeft = commandedLeft;
  const int16_t normalRight = commandedRight;
  const int16_t normalError = previousError;

  resetScenario();
  kdX100 = 0;
  lineIsInverse = true;
  fillFrame(analogFrame, 850);
  analogFrame[5] = 240;
  analogFrame[6] = 111;
  tickFrame(analogFrame);
  assert(absoluteError(previousError - normalError) <= 2);
  assert(absoluteError(commandedLeft - normalLeft) <= 2);
  assert(absoluteError(commandedRight - normalRight) <= 2);

  // One continuous curved/diagonal group never accumulates junction votes.
  resetScenario();
  const uint16_t continuousDiagonal = 0x01F0;
  for (uint8_t frame = 0; frame < 6; frame++)
  {
    tick(continuousDiagonal);
    assert(state == STATE_FOLLOW && !junctionActive &&
           junctionFrameCount == 0);
  }

  // A separated sub-wide branch pattern still uses the existing vote path.
  resetScenario();
  const uint16_t separatedBranch = oneSensor(4) | oneSensor(5) | oneSensor(7);
  tick(separatedBranch);
  tick(separatedBranch);
  tick(separatedBranch);
  assert(junctionActive);
  assert(junctionAvailableRoutes == (ROUTE_STRAIGHT | ROUTE_RIGHT));

  // Existing main(5)-style route evidence: T, cross, and three-way.
  resetScenario();
  observeWideThen(0);
  simulatedMicros += (uint32_t)PROBE_BRAKE_HOLD_MS * 1000UL;
  tick(0);
  assert(junctionAvailableRoutes == (ROUTE_LEFT | ROUTE_RIGHT));
  assert(selectedRoute == ROUTE_LEFT);

  resetScenario();
  observeWideThen(centered);
  finishNarrowConfirmation(centered);
  assert(junctionAvailableRoutes ==
         (ROUTE_LEFT | ROUTE_STRAIGHT | ROUTE_RIGHT));

  resetScenario();
  tick(leftAndStraight);
  tick(leftAndStraight);
  tick(centered);
  finishNarrowConfirmation(centered);
  assert(junctionAvailableRoutes == (ROUTE_LEFT | ROUTE_STRAIGHT));

  // Every EEPROM priority order selects its first actually detected cross route.
  for (uint8_t priority = 0; priority < ROUTE_PRIORITY_COUNT; priority++)
  {
    resetScenario();
    routePriority = priority;
    const RouteDirection expected = settingsPriorityAt(0);
    observeWideThen(centered);
    finishNarrowConfirmation(centered);
    assert(selectedRoute == expected);
    assert(junctionActive);
  }

  // NORMAL -> INVERSE survives the temporary wide probe; cross never toggles.
  resetScenario();
  tick(centered);
  simulatedMicros += (uint32_t)INVERSE_TOGGLE_COOLDOWN_MS * 1000UL;
  tick(inverseEntry);
  assert(state == STATE_FORWARD_PROBE && !lineIsInverse);
  tick(inverseEntry);
  assert(lineIsInverse && state == STATE_FOLLOW);
  assert(commandedLeft > 0 && commandedRight > 0);

  simulatedMicros += (uint32_t)INVERSE_TOGGLE_COOLDOWN_MS * 1000UL;
  tick(centered);
  tick(centered);
  assert(!lineIsInverse && state == STATE_FOLLOW);

  resetScenario();
  simulatedMicros += (uint32_t)INVERSE_TOGGLE_COOLDOWN_MS * 1000UL;
  tick(ALL_SENSOR_MASK);
  tick(ALL_SENSOR_MASK);
  assert(!lineIsInverse);

  // Stable centred loss remains dotted/gap handling; outward loss turns now.
  resetScenario();
  tick(centered);
  tick(centered);
  tick(centered);
  tick(0);
  assert(state == STATE_FORWARD_PROBE && probeReason == PROBE_LOST_LINE);

  // Lost-line confirmation stays in motion and retains all confirmation reads.
  brakeWasCalled = false;
  tick(centered);
  assert(state == STATE_FORWARD_PROBE);
  assert(commandedLeft > 0 && commandedRight > 0 && !brakeWasCalled &&
         stopCallCount == 0);
  tick(centered);
  assert(state == STATE_FORWARD_PROBE);
  tick(centered);
  assert(state == STATE_FOLLOW && commandedLeft > 0 &&
         commandedRight > 0 && stopCallCount == 0);

  verifyAngledLoss(false, 3);
  verifyAngledLoss(true, 10);
  verifyAngledLoss(false, 2);
  verifyAngledLoss(true, 11);
  verifyAngledLoss(false, 1);
  verifyAngledLoss(true, 12);
  verifyAngledLoss(false, 0);
  verifyAngledLoss(true, 13);

  // Expected-side reacquisition slows; centre confirms moving for two reads.
  resetScenario();
  tick(oneSensor(6));
  tick(oneSensor(5));
  tick(oneSensor(3));
  tick(0);
  simulatedMicros += (uint32_t)TURN_MINIMUM_MS * 1000UL;
  tick(oneSensor(3));
  assert(commandedLeft == TURN_REACQUIRE_PWM &&
         commandedRight == -TURN_REACQUIRE_PWM);
  const uint16_t stopsBeforeTurnCenter = stopCallCount;
  tick(centered);
  assert(state == STATE_TURN_TO_ROUTE &&
         commandedLeft > 0 && commandedRight == 0);
  tick(centered);
  assert(state == STATE_ROUTE_COMMIT && commandedLeft > 0 &&
         commandedRight > 0 && stopCallCount == stopsBeforeTurnCenter);

  // Reverse and recovery search hand a narrow line directly to moving confirm.
  resetScenario();
  beginReverse();
  tick(centered);
  assert(state == STATE_RECOVERY_SEARCH && commandedLeft > 0 &&
         commandedRight > 0 && stopCallCount == 0);
  tick(centered);
  assert(state == STATE_FOLLOW && commandedLeft > 0 &&
         commandedRight > 0 && stopCallCount == 0);

  resetScenario();
  beginRecoverySearch();
  stopCallCount = 0;
  tick(centered);
  assert(state == STATE_RECOVERY_SEARCH && commandedLeft > 0 &&
         commandedRight > 0);
  tick(centered);
  assert(state == STATE_FOLLOW && commandedLeft > 0 &&
         commandedRight > 0 && stopCallCount == 0);

  // If a right pivot crosses centre, correction reverses gently.
  resetScenario();
  beginTurn(ROUTE_RIGHT);
  turnSawCenterLost = true;
  tick(oneSensor(9), TURN_MINIMUM_MS);
  tick(oneSensor(9));
  assert(commandedLeft == -TURN_CROSS_CORRECTION_PWM &&
         commandedRight == TURN_CROSS_CORRECTION_PWM);

  // A true sign reversal coasts first, then applies after the shared guard.
  resetScenario();
  moveLFR(-120, 120);
  assert(commandedLeft == -120 && commandedRight == 120);
  moveLFR(115, 115);
  assert(commandedLeft == 0 && commandedRight == 115);
  simulatedMicros += MOTOR_DIRECTION_GUARD_US;
  moveLFR(115, 115);
  assert(commandedLeft == 115 && commandedRight == 115);

  // A persistent filled area crawls, then exits without a finish/safety stop.
  resetScenario();
  tick(ALL_SENSOR_MASK, FORWARD_PROBE_WINDOW_MS);
  tick(ALL_SENSOR_MASK);
  assert(state == STATE_FORWARD_PROBE && wideAreaCrawl);
  assert(commandedLeft == WIDE_AREA_CRAWL_PWM &&
         commandedRight == WIDE_AREA_CRAWL_PWM);
  const uint8_t latchedLeftVotes = leftRouteVotes;
  const uint8_t latchedRightVotes = rightRouteVotes;
  for (uint8_t frame = 0; frame < 6; frame++) tick(ALL_SENSOR_MASK);
  assert(leftRouteVotes == latchedLeftVotes &&
         rightRouteVotes == latchedRightVotes);
  finishNarrowConfirmation(centered);
  tick(centered);
  assert(state != STATE_STOPPED && junctionActive);
  assert(junctionAvailableRoutes ==
         (ROUTE_LEFT | ROUTE_STRAIGHT | ROUTE_RIGHT));

  // Turn/recovery timeouts alternate reverse and search indefinitely.
  resetScenario();
  beginTurn(ROUTE_U_TURN);
  simulatedMicros += (uint32_t)U_TURN_TIMEOUT_MS * 1000UL;
  tick(0);
  assert(state == STATE_REVERSE_REACQUIRE);
  simulatedMicros += (uint32_t)RECOVERY_REVERSE_TIMEOUT_MS * 1000UL;
  tick(0);
  assert(state == STATE_RECOVERY_SEARCH);
  simulatedMicros += (uint32_t)TURN_SEARCH_TIMEOUT_MS * 1000UL;
  tick(0);
  assert(state == STATE_REVERSE_REACQUIRE);

  std::cout << "navigation scenarios passed; sensor/control "
            << navigationWorstSensorFrameMicros() << "/"
            << navigationWorstControlTickMicros() << " us\n";
  return 0;
}
