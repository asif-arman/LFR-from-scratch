#include <cassert>
#include <cstdint>
#include <iostream>

#include "../motor_guard.h"
#include "../settings.cpp"
#include "../navigation.cpp"


constexpr uint16_t HOST_SENSOR_FRAME_US = 1550;
constexpr uint16_t LONG_ROTATION_TEST_TICKS = 600;
static uint32_t simulatedMicros = 0;
static uint16_t sensorFrame[SENSOR_COUNT] = {};
static int16_t commandedLeft = 0;
static int16_t commandedRight = 0;
static uint16_t stopCallCount = 0;
static uint16_t sensorReadCount = 0;
static MotorDirectionGuardState hostLeftGuard = { 0, true, 0 };
static MotorDirectionGuardState hostRightGuard = { 0, true, 0 };


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
}


void motorInit() {}
void sensorInit() {}


void readSensors(uint16_t values[])
{
  sensorReadCount++;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
    values[sensor] = sensorFrame[sensor];
  simulatedMicros += HOST_SENSOR_FRAME_US;
}


uint16_t makeSensorPattern(const uint16_t values[])
{
  uint16_t pattern = 0;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    if (settingsBlackStrength(sensor, values[sensor]) >= 500)
      pattern |= (uint16_t)1 << sensor;
  }
  return pattern;
}


int16_t calculateLinePosition(uint16_t pattern)
{
  uint16_t sum = 0;
  uint8_t count = 0;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    if (pattern & ((uint16_t)1 << sensor))
    {
      sum += sensor * SENSOR_POSITION_SCALE;
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


static NavigationResult tickFrame(const uint16_t frame[])
{
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
    sensorFrame[sensor] = frame[sensor];
  uint16_t values[SENSOR_COUNT] = {};
  const NavigationResult result = navigationTick(values);
  if (CONTROL_PERIOD_US > HOST_SENSOR_FRAME_US)
    simulatedMicros += CONTROL_PERIOD_US - HOST_SENSOR_FRAME_US;
  return result;
}


static NavigationResult tick(uint16_t pattern)
{
  uint16_t frame[SENSOR_COUNT];
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
    frame[sensor] = (pattern & ((uint16_t)1 << sensor)) ? 850 : 100;
  return tickFrame(frame);
}


static void repeat(uint16_t pattern, uint16_t count)
{
  while (count-- != 0) assert(tick(pattern) == NAVIGATION_ACTIVE);
}


static uint16_t runBlankUntilUTurn()
{
  uint16_t blankFrames = 0;
  uint8_t uTurnEntries = 0;
  const uint16_t guardFrames = GAP_ALLOWANCE_MS * 1000UL /
      CONTROL_PERIOD_US + 10;
  while (state != STATE_U_TURN && blankFrames < guardFrames)
  {
    const NavState previousState = state;
    assert(tick(0) == NAVIGATION_ACTIVE);
    blankFrames++;
    if (previousState != STATE_U_TURN && state == STATE_U_TURN)
      uTurnEntries++;
  }
  assert(state == STATE_U_TURN);
  assert(uTurnEntries == 1);
  return blankFrames;
}


static uint16_t oneSensor(uint8_t sensor)
{
  return (uint16_t)1 << sensor;
}


static void fillFrame(uint16_t frame[], uint16_t value)
{
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
    frame[sensor] = value;
}


static void resetScenario(bool enableBoxMode = false)
{
  simulatedMicros = 0;
  fillFrame(sensorFrame, 100);
  commandedLeft = 0;
  commandedRight = 0;
  stopCallCount = 0;
  sensorReadCount = 0;
  kpX100 = 20;
  kdX100 = 50;
  baseSpeed = 200;
  sensorThreshold = 400;
  routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
  boxMode = enableBoxMode;
  sensorCalibrationValid = true;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    sensorMinimums[sensor] = 100;
    sensorMaximums[sensor] = 850;
  }
  motorDirectionGuardReset(hostLeftGuard);
  motorDirectionGuardReset(hostRightGuard);
  navigationStart();
  stopCallCount = 0;
}


static void finishReacquire(uint16_t centered)
{
  uint8_t guard = REACQUIRE_CONFIRM_TICKS + 2;
  while (state == STATE_REACQUIRE && guard-- != 0) tick(centered);
  assert(state == STATE_FOLLOW);
}


static void selectFullCross(uint8_t priority, uint16_t exitPattern)
{
  routePriority = priority;
  repeat(ALL_SENSOR_MASK, SIDE_CONFIRM_TICKS);
  assert(state == STATE_JUNCTION_PROBE);
  repeat(exitPattern, JUNCTION_CLEAR_TICKS);
}


int main()
{
  const uint16_t centered = oneSensor(6) | oneSensor(7);
  const uint16_t mildRight = oneSensor(5);
  const uint16_t rightEdge = oneSensor(4);
  const uint16_t leftEdge = oneSensor(9);
  const uint16_t leftOutgoing = oneSensor(12);
  const uint16_t thinLeftBranch = centered |
      oneSensor(10) | oneSensor(11);
  const uint16_t thinRightBranch = centered |
      oneSensor(2) | oneSensor(3);

  // Centered PD uses the requested 200 PWM normal speed.
  resetScenario();
  assert(tick(centered) == NAVIGATION_ACTIVE);
  assert(commandedLeft == 200 && commandedRight == 200);

  // Analog movement inside one binary cluster changes PWM continuously.
  resetScenario();
  kdX100 = 0;
  uint16_t analogFrame[SENSOR_COUNT];
  fillFrame(analogFrame, 100);
  analogFrame[5] = 500;
  analogFrame[6] = 850;
  tickFrame(analogFrame);
  const int16_t firstRight = commandedRight;
  analogFrame[5] = 650;
  analogFrame[6] = 750;
  tickFrame(analogFrame);
  assert(commandedLeft == 200 && commandedRight < firstRight);

  // Ordinary PD never reverses or exceeds the configured base speed.
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    resetScenario();
    tick(oneSensor(sensor));
    assert(commandedLeft >= 0 && commandedRight >= 0);
    assert(commandedLeft <= 200 && commandedRight <= 200);
  }

  // Large errors temporarily lower the drive base without changing the saved
  // speed. Ordinary PD stays forward-only even at the outermost sensor.
  resetScenario();
  kdX100 = 0;
  tick(oneSensor(0));
  assert(baseSpeed == 200);
  assert(commandedLeft <= OUTER_ERROR_PWM);
  assert(commandedRight >= 0);

  resetScenario();
  kdX100 = 0;
  tick(oneSensor(3));
  assert(baseSpeed == 200);
  assert(commandedLeft <= MODERATE_ERROR_PWM);
  assert(commandedRight >= 0);

  // GAP keeps the previous steering ratio at a bounded forward speed.
  resetScenario();
  kdX100 = 0;
  tick(mildRight);
  tick(0);
  assert(state == STATE_GAP);
  assert(commandedLeft > commandedRight && commandedRight > 0);
  assert(commandedLeft <= GAP_MAX_PWM && commandedRight <= GAP_MAX_PWM);

  // Centred and off-centre dots each cancel GAP and reset their own 200 ms
  // allowance. Repetition cannot accumulate into a false U-turn.
  resetScenario();
  constexpr uint8_t DOT_GAP_TICKS = 6;
  for (uint8_t dot = 0; dot < 12; dot++)
  {
    tick((dot & 1) == 0 ? centered : mildRight);
    assert(state == STATE_FOLLOW);
    for (uint8_t white = 0; white < DOT_GAP_TICKS; white++)
    {
      tick(0);
      assert(state == STATE_GAP);
    }
  }
  tick(centered);
  assert(state == STATE_FOLLOW);
  assert(commandedLeft == 200 && commandedRight == 200);

  // Only one uninterrupted blank interval beyond 200 ms starts one U-turn.
  resetScenario();
  tick(centered);
  tick(0);
  while ((uint32_t)(millis() - gapStartedAtMillis) <
         GAP_ALLOWANCE_MS - 5)
  {
    tick(0);
    assert(state == STATE_GAP);
  }
  runBlankUntilUTurn();
  assert(state == STATE_U_TURN);
  repeat(0, 20);
  assert(state == STATE_U_TURN);
  assert(stopCallCount == 0);

  // An outer-edge loss is a directed sharp turn, never GAP or U-turn.
  resetScenario();
  tick(rightEdge);
  assert(lastLinePosition < LINE_CENTER && lastLineSide == 1);
  tick(0);
  assert(state == STATE_TURN && turnDirection == 1);
  tick(centered);
  assert(state == STATE_REACQUIRE);
  assert(commandedLeft == 0 && commandedRight == 0);
  finishReacquire(centered);
  assert(commandedLeft <= REACQUIRE_PWM && commandedRight <= REACQUIRE_PWM);
  tick(centered);
  assert(commandedLeft == 200 && commandedRight == 200);

  resetScenario();
  tick(leftEdge);
  assert(lastLinePosition > LINE_CENTER && lastLineSide == -1);
  tick(0);
  assert(state == STATE_TURN && turnDirection == -1);
  tick(leftOutgoing);
  assert(state == STATE_REACQUIRE);
  assert(commandedLeft == 0 && commandedRight == 0);
  finishReacquire(centered);

  // A thick centred outgoing corner is a valid capture even though the frame
  // classifier temporarily calls it a junction feature.
  resetScenario();
  const uint16_t thickCentered =
      oneSensor(3) | oneSensor(4) | oneSensor(5) | oneSensor(6) |
      oneSensor(7) | oneSensor(8) | oneSensor(9) | oneSensor(10);
  tick(rightEdge);
  tick(0);
  assert(state == STATE_TURN);
  tick(thickCentered);
  assert(state == STATE_REACQUIRE);
  assert(commandedLeft == 0 && commandedRight == 0);
  repeat(thickCentered, REACQUIRE_CONFIRM_TICKS);
  assert(state == STATE_FOLLOW);

  // A T-junction clears to white and still chooses an available side.
  resetScenario();
  repeat(ALL_SENSOR_MASK, SIDE_CONFIRM_TICKS);
  assert(state == STATE_JUNCTION_PROBE);
  tick(0);
  assert(state == STATE_JUNCTION_PROBE);
  tick(0);
  assert(state == STATE_TURN && turnDirection == -1);

  // A cross with a center exit follows configured straight-first priority.
  resetScenario();
  selectFullCross(PRIORITY_STRAIGHT_LEFT_RIGHT, centered);
  assert(state == STATE_REACQUIRE);
  finishReacquire(centered);

  // Incomplete junction classification keeps crawling instead of turning back.
  resetScenario();
  const uint16_t ambiguousFeature = oneSensor(4) | oneSensor(9);
  repeat(ambiguousFeature, SIDE_CONFIRM_TICKS);
  assert(state == STATE_JUNCTION_PROBE);
  repeat(0, LONG_ROTATION_TEST_TICKS);
  assert(state == STATE_JUNCTION_PROBE);
  assert(commandedLeft == JUNCTION_CRAWL_PWM &&
         commandedRight == JUNCTION_CRAWL_PWM);
  assert(stopCallCount == 0);
  repeat(thinRightBranch, SIDE_CONFIRM_TICKS);
  repeat(0, JUNCTION_CLEAR_TICKS);
  assert(state == STATE_TURN && turnDirection == 1);

  // The first off-centre outgoing line after old-centre loss brakes the pivot.
  resetScenario();
  routePriority = PRIORITY_LEFT_STRAIGHT_RIGHT;
  repeat(thinLeftBranch, SIDE_CONFIRM_TICKS);
  repeat(centered, JUNCTION_CLEAR_TICKS);
  assert(state == STATE_TURN);
  repeat(0, TURN_CENTER_LOST_TICKS);
  assert(state == STATE_TURN && stopCallCount == 0);
  tick(leftOutgoing);
  assert(state == STATE_REACQUIRE);
  assert(commandedLeft == 0 && commandedRight == 0);
  tick(centered);
  assert(commandedLeft <= REACQUIRE_PWM &&
         commandedRight <= REACQUIRE_PWM);

  // One isolated edge dot can suggest a corner, but continuous blank sensors
  // eventually become one confirmed loss instead of latching TURN forever.
  resetScenario();
  tick(rightEdge);
  tick(0);
  assert(state == STATE_TURN);
  uint8_t edgeLossEntries = 0;
  const uint32_t edgeBlankStarted = turnBlankStartedAtMillis;
  while (state == STATE_TURN)
  {
    const NavState previousState = state;
    tick(0);
    if (previousState != STATE_U_TURN && state == STATE_U_TURN)
      edgeLossEntries++;
  }
  assert((uint32_t)(millis() - edgeBlankStarted) >= GAP_ALLOWANCE_MS);
  assert(state == STATE_U_TURN && edgeLossEntries == 1);

  // A loss-triggered U-turn accepts a usable line on any sensor immediately.
  resetScenario();
  tick(centered);
  tick(0);
  runBlankUntilUTurn();
  assert(state == STATE_U_TURN);
  repeat(0, LONG_ROTATION_TEST_TICKS);
  assert(state == STATE_U_TURN && stopCallCount == 0);
  tick(oneSensor(0));
  assert(state == STATE_REACQUIRE);
  assert(commandedLeft == 0 && commandedRight == 0);
  finishReacquire(oneSensor(0));
  assert(commandedLeft <= REACQUIRE_PWM &&
         commandedRight <= REACQUIRE_PWM);
  tick(centered);
  assert(commandedLeft == 200 && commandedRight == 200);

  // A newly found wide feature also cancels U-turn and starts junction work.
  resetScenario();
  tick(centered);
  tick(0);
  runBlankUntilUTurn();
  assert(state == STATE_U_TURN);
  tick(ALL_SENSOR_MASK);
  assert(state == STATE_JUNCTION_PROBE && junctionLocked);

  // A junction cancels pending loss immediately and never starts a U-turn.
  resetScenario();
  tick(centered);
  tick(0);
  tick(ALL_SENSOR_MASK);
  assert(state == STATE_JUNCTION_PROBE && junctionLocked);
  for (uint8_t frame = 0; frame < 40; frame++)
  {
    tick(0);
    assert(state != STATE_U_TURN);
  }

  // Generation 3 speed 100 migrates once to 200 without losing calibration.
  EEPROM.reset();
  SettingsRecord saved = {};
  saved.magic = EEPROM_MAGIC;
  saved.version = EEPROM_VERSION;
  saved.kp = 24;
  saved.kd = 55;
  saved.speed = 100;
  saved.threshold = 487;
  saved.priority = PRIORITY_RIGHT_LEFT_STRAIGHT;
  saved.flags = CALIBRATION_VALID_MASK | BOX_MODE_MASK |
      (3 << TUNING_VERSION_SHIFT);
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    saved.calibratedMinimums[sensor] = 50 + sensor;
    saved.calibratedMaximums[sensor] = 900 + sensor;
  }
  writeRecord(saved);
  settingsLoad();
  assert(kpX100 == 24 && kdX100 == 55 && baseSpeed == 200);
  assert(sensorThreshold == 487 && sensorCalibrationValid && boxMode);
  assert(routePriority == PRIORITY_RIGHT_LEFT_STRAIGHT);
  kpX100 = kdX100 = baseSpeed = 0;
  settingsLoad();
  assert(kpX100 == 24 && kdX100 == 55 && baseSpeed == 200);

  // Later OLED tuning is generation 4 and survives subsequent boots.
  kpX100 = 26;
  kdX100 = 52;
  baseSpeed = 173;
  settingsSaveIfChanged();
  kpX100 = kdX100 = baseSpeed = 0;
  settingsLoad();
  assert(kpX100 == 26 && kdX100 == 52 && baseSpeed == 173);

  // Optional black start/finish boxes remain supported and isolated.
  resetScenario(true);
  repeat(ALL_SENSOR_MASK, 3);
  assert(state == STATE_REACQUIRE && startBoxPending);
  repeat(centered, START_EXIT_CONFIRM_TICKS);
  assert(state == STATE_FOLLOW && !startBoxPending);
  repeat(centered, FINISH_ARM_TICKS);
  NavigationResult finishResult = NAVIGATION_ACTIVE;
  for (uint8_t i = 0; i < FINISH_CONFIRM_TICKS; i++)
    finishResult = tick(ALL_SENSOR_MASK);
  assert(finishResult == NAVIGATION_FINISHED && state == STATE_STOPPED);

  // The common motor guard inserts only its required reversal interval.
  resetScenario();
  moveLFR(-TURN_PWM, TURN_PWM);
  moveLFR(TURN_PWM, -TURN_PWM);
  assert(commandedLeft == 0 && commandedRight == 0);
  simulatedMicros += MOTOR_DIRECTION_GUARD_US;
  moveLFR(TURN_PWM, -TURN_PWM);
  assert(commandedLeft == TURN_PWM && commandedRight == -TURN_PWM);

  // FOLLOW, GAP and U-turn all receive one fresh sensor frame per tick.
  resetScenario();
  tick(centered);
  tick(0);
  const uint16_t readsBeforeLoss = sensorReadCount;
  const uint16_t blankReads = runBlankUntilUTurn();
  assert(state == STATE_U_TURN);
  tick(oneSensor(0));
  assert(state == STATE_REACQUIRE);
  assert(sensorReadCount - readsBeforeLoss == blankReads + 1);
  assert(navigationWorstSensorFrameMicros() == HOST_SENSOR_FRAME_US);
  assert(navigationWorstControlTickMicros() == HOST_SENSOR_FRAME_US);
  assert(navigationWorstControlTickMicros() <= CONTROL_PERIOD_US);

  std::cout << "navigation scenarios passed; sensor/control "
            << navigationWorstSensorFrameMicros() << "/"
            << navigationWorstControlTickMicros() << " us\n";
  return 0;
}
