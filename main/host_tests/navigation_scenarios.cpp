#include <cassert>
#include <cstdint>
#include <iostream>

#include "../motor_guard.h"
#include "../settings.cpp"
#include "../navigation.cpp"


constexpr uint16_t HOST_SENSOR_FRAME_US = 1550;
static uint32_t simulatedMicros = 0;
static uint16_t sensorFrame[SENSOR_COUNT] = {};
static int16_t commandedLeft = 0;
static int16_t commandedRight = 0;
static uint16_t stopCallCount = 0;
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
  while (count-- != 0) tick(pattern);
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
  kpX100 = 20;
  kdX100 = 50;
  baseSpeed = 100;
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


static void selectFullCross(uint8_t priority, uint16_t centered)
{
  routePriority = priority;
  repeat(ALL_SENSOR_MASK, SIDE_CONFIRM_TICKS);
  assert(state == STATE_JUNCTION_PROBE);
  repeat(centered, JUNCTION_CLEAR_TICKS);
}


static void completeTurn(uint16_t centered, uint16_t variedCenter)
{
  assert(state == STATE_TURN);
  tick(centered); // the old straight line alone cannot finish the turn
  assert(state == STATE_TURN);
  repeat(0, TURN_MIN_TICKS);
  tick(variedCenter);
  tick(0); // one noisy frame resets confirmation
  repeat(variedCenter, TURN_CENTER_CONFIRM_TICKS);
  assert(state == STATE_REACQUIRE);
  assert(commandedLeft == 0 && commandedRight == 0);
  repeat(centered, REACQUIRE_CONFIRM_TICKS);
  assert(state == STATE_FOLLOW);
}


static void completeUTurn(uint16_t centered)
{
  assert(state == STATE_U_TURN);
  repeat(0, UTURN_MIN_TICKS);
  repeat(centered, TURN_CENTER_CONFIRM_TICKS);
  assert(state == STATE_REACQUIRE);
  assert(commandedLeft == 0 && commandedRight == 0);
  repeat(centered, REACQUIRE_CONFIRM_TICKS);
  assert(state == STATE_FOLLOW);
}


int main()
{
  const uint16_t centered = oneSensor(6) | oneSensor(7);
  const uint16_t variedCenter = oneSensor(5) | oneSensor(6) |
      oneSensor(7) | oneSensor(8);
  const uint16_t leftLine = oneSensor(9);
  const uint16_t rightLine = oneSensor(4);
  const uint16_t thinLeftBranch = centered |
      oneSensor(10) | oneSensor(11);
  const uint16_t thinRightBranch = centered |
      oneSensor(2) | oneSensor(3);
  const uint16_t thickCurve = oneSensor(8) | oneSensor(9) |
      oneSensor(10) | oneSensor(11) | oneSensor(12) | oneSensor(13);

  // Calibrated normalization still preserves useful analog strength.
  resetScenario();
  assert(settingsBlackStrength(0, 100) == 0);
  assert(settingsBlackStrength(0, 475) == 500);
  assert(settingsBlackStrength(0, 850) == 1000);

  // A centered analog line produces equal forward outputs at the safe base.
  resetScenario();
  assert(tick(centered) == NAVIGATION_ACTIVE);
  assert(commandedLeft == 100 && commandedRight == 100);

  // Analog movement inside the same binary cluster changes PWM continuously.
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
  assert(commandedLeft == baseSpeed && commandedRight < firstRight);
  assert(commandedRight > 0);

  // Every ordinary PD output stays forward and at or below base speed.
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    resetScenario();
    tick(oneSensor(sensor));
    assert(commandedLeft >= 0 && commandedRight >= 0);
    assert(commandedLeft <= baseSpeed && commandedRight <= baseSpeed);
  }

  // A useful sub-90 command is preserved instead of being forced to zero.
  resetScenario();
  kdX100 = 0;
  tick(rightLine); // error +250, Kp 0.20 -> 50 PWM correction
  assert(commandedLeft == 100 && commandedRight == 50);

  // A short gap preserves reduced steering and returns through REACQUIRE.
  resetScenario();
  kdX100 = 0;
  tick(leftLine);
  const bool steeringLeft = commandedLeft < commandedRight;
  tick(0);
  assert(state == STATE_GAP);
  assert(commandedLeft >= 0 && commandedRight >= 0);
  assert((commandedLeft < commandedRight) == steeringLeft);
  repeat(0, 2);
  tick(leftLine);
  assert(state == STATE_GAP);
  tick(leftLine);
  assert(state == STATE_REACQUIRE);
  repeat(leftLine, REACQUIRE_CONFIRM_TICKS);
  assert(state == STATE_FOLLOW);

  // Persistent white begins a sensor-controlled U-turn, not a one-frame loss.
  resetScenario();
  tick(centered);
  tick(0);
  repeat(0, GAP_ALLOWANCE_TICKS - 1);
  assert(state == STATE_GAP);
  tick(0);
  assert(state == STATE_U_TURN);
  assert(commandedLeft >= 0 && commandedRight <= 0);

  // LEFT completes after old-center loss and two new-center frames, including
  // reasonable width variation and a noisy frame.
  resetScenario();
  routePriority = PRIORITY_LEFT_STRAIGHT_RIGHT;
  repeat(thinLeftBranch, SIDE_CONFIRM_TICKS);
  assert(state == STATE_JUNCTION_PROBE);
  repeat(centered, JUNCTION_CLEAR_TICKS);
  assert(state == STATE_TURN && turnDirection == -1);
  tick(centered);
  assert(commandedLeft < 0 && commandedRight > 0);
  completeTurn(centered, variedCenter);

  // RIGHT uses the mirrored simple sensor-controlled turn.
  resetScenario();
  routePriority = PRIORITY_RIGHT_STRAIGHT_LEFT;
  repeat(thinRightBranch, SIDE_CONFIRM_TICKS);
  repeat(centered, JUNCTION_CLEAR_TICKS);
  assert(state == STATE_TURN && turnDirection == 1);
  tick(centered);
  assert(commandedLeft > 0 && commandedRight < 0);
  completeTurn(centered, variedCenter);

  // Normal S>L>R selects straight at a full L/S/R crossing.
  resetScenario();
  selectFullCross(PRIORITY_STRAIGHT_LEFT_RIGHT, centered);
  assert(state == STATE_REACQUIRE && lastChosenRoute == ROUTE_STRAIGHT);
  repeat(centered, REACQUIRE_CONFIRM_TICKS);
  assert(state == STATE_FOLLOW && !junctionLocked);

  // A dead end after that straight choice latches straight-last through U-turn.
  tick(0);
  repeat(0, GAP_ALLOWANCE_TICKS);
  assert(state == STATE_U_TURN && avoidStraightAtNextJunction);
  completeUTurn(centered);
  assert(avoidStraightAtNextJunction);

  // Default side preference is L before R, so the return crossing chooses L.
  selectFullCross(PRIORITY_STRAIGHT_LEFT_RIGHT, centered);
  assert(state == STATE_TURN && turnDirection == -1);
  assert(avoidStraightAtNextJunction);
  completeTurn(centered, centered);
  assert(!avoidStraightAtNextJunction && !junctionLocked);

  // A later unrelated crossing returns to normal S>L>R behavior.
  selectFullCross(PRIORITY_STRAIGHT_LEFT_RIGHT, centered);
  assert(state == STATE_REACQUIRE && lastChosenRoute == ROUTE_STRAIGHT);

  // Mirror the one-return latch for a user order that ranks RIGHT before LEFT.
  resetScenario();
  selectFullCross(PRIORITY_STRAIGHT_RIGHT_LEFT, centered);
  repeat(centered, REACQUIRE_CONFIRM_TICKS);
  tick(0);
  repeat(0, GAP_ALLOWANCE_TICKS);
  assert(state == STATE_U_TURN && avoidStraightAtNextJunction);
  completeUTurn(centered);
  selectFullCross(PRIORITY_STRAIGHT_RIGHT_LEFT, centered);
  assert(state == STATE_TURN && turnDirection == 1);

  // One branch frame is noise; two frames begin the forward junction probe.
  resetScenario();
  tick(thinLeftBranch);
  assert(state == STATE_FOLLOW);
  tick(centered);
  assert(state == STATE_FOLLOW);
  repeat(thinLeftBranch, SIDE_CONFIRM_TICKS);
  assert(state == STATE_JUNCTION_PROBE);

  // Thin and thick single-cluster curves remain ordinary PD, not junctions.
  resetScenario();
  repeat(leftLine, 5);
  assert(state == STATE_FOLLOW && !junctionLocked);
  resetScenario();
  repeat(thickCurve, 5);
  assert(state == STATE_FOLLOW && !junctionLocked);

  // A wide crossing remains black-line behavior and returns to correct PD.
  resetScenario();
  selectFullCross(PRIORITY_STRAIGHT_LEFT_RIGHT, centered);
  repeat(centered, REACQUIRE_CONFIRM_TICKS);
  tick(rightLine);
  assert(state == STATE_FOLLOW && commandedLeft > commandedRight);

  // Optional black start/finish boxes remain isolated from normal following.
  resetScenario(true);
  assert(state == STATE_REACQUIRE && startBoxPending);
  repeat(ALL_SENSOR_MASK, 3);
  assert(startBoxPending && commandedLeft == START_BOX_EXIT_PWM);
  repeat(centered, START_EXIT_CONFIRM_TICKS);
  assert(state == STATE_FOLLOW && !startBoxPending);
  repeat(centered, FINISH_ARM_TICKS);
  NavigationResult finishResult = NAVIGATION_ACTIVE;
  for (uint8_t tickIndex = 0; tickIndex < FINISH_CONFIRM_TICKS; tickIndex++)
    finishResult = tick(ALL_SENSOR_MASK);
  assert(finishResult == NAVIGATION_FINISHED && state == STATE_STOPPED);

  // Version 2 tuning migrates once while calibration and route policy survive.
  EEPROM.reset();
  SettingsRecord saved = {};
  saved.magic = EEPROM_MAGIC;
  saved.version = EEPROM_VERSION;
  saved.kp = 80;
  saved.kd = 2;
  saved.speed = 220;
  saved.threshold = 487;
  saved.priority = PRIORITY_RIGHT_LEFT_STRAIGHT;
  saved.flags = CALIBRATION_VALID_MASK | BOX_MODE_MASK |
      (2 << TUNING_VERSION_SHIFT);
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    saved.calibratedMinimums[sensor] = 50 + sensor;
    saved.calibratedMaximums[sensor] = 900 + sensor;
  }
  writeRecord(saved);
  settingsLoad();
  assert(kpX100 == 20 && kdX100 == 50 && baseSpeed == 100);
  assert(sensorThreshold == 487 &&
         routePriority == PRIORITY_RIGHT_LEFT_STRAIGHT && boxMode);
  assert(sensorCalibrationValid);
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    assert(sensorMinimums[sensor] == 50 + sensor);
    assert(sensorMaximums[sensor] == 900 + sensor);
  }

  // Future OLED tuning is version 3 and survives a later reload unchanged.
  kpX100 = 24;
  kdX100 = 55;
  baseSpeed = 105;
  settingsSaveIfChanged();
  kpX100 = kdX100 = baseSpeed = 0;
  settingsLoad();
  assert(kpX100 == 24 && kdX100 == 55 && baseSpeed == 105);

  // A turn that never leaves/finds center times out with a safe stop.
  resetScenario();
  routePriority = PRIORITY_LEFT_STRAIGHT_RIGHT;
  repeat(thinLeftBranch, SIDE_CONFIRM_TICKS);
  repeat(centered, JUNCTION_CLEAR_TICKS);
  repeat(centered, TURN_TIMEOUT_TICKS - 1);
  assert(state == STATE_TURN);
  assert(tick(centered) == NAVIGATION_LOST);
  assert(state == STATE_STOPPED && commandedLeft == 0 && commandedRight == 0);

  // A U-turn that never finds a returned line also stops safely.
  resetScenario();
  tick(centered);
  tick(0);
  repeat(0, GAP_ALLOWANCE_TICKS);
  repeat(0, UTURN_TIMEOUT_TICKS - 1);
  assert(state == STATE_U_TURN);
  assert(tick(0) == NAVIGATION_LOST);
  assert(state == STATE_STOPPED && commandedLeft == 0 && commandedRight == 0);

  // An intersection that never clears cannot trap the robot in a crawl.
  resetScenario();
  repeat(ALL_SENSOR_MASK, SIDE_CONFIRM_TICKS);
  assert(state == STATE_JUNCTION_PROBE);
  repeat(ALL_SENSOR_MASK, JUNCTION_PROBE_TIMEOUT_TICKS - 1);
  assert(state == STATE_JUNCTION_PROBE);
  assert(tick(ALL_SENSOR_MASK) == NAVIGATION_LOST);
  assert(state == STATE_STOPPED && commandedLeft == 0 && commandedRight == 0);

  // REACQUIRE also fails closed when no credible outgoing line appears.
  resetScenario();
  setReacquireState();
  repeat(0, REACQUIRE_TIMEOUT_TICKS - 1);
  assert(state == STATE_REACQUIRE);
  assert(tick(0) == NAVIGATION_LOST);
  assert(state == STATE_STOPPED && commandedLeft == 0 && commandedRight == 0);

  // The common motor guard still enforces one precise zero interval.
  resetScenario();
  moveLFR(-TURN_PWM, TURN_PWM);
  assert(commandedLeft == -TURN_PWM && commandedRight == TURN_PWM);
  moveLFR(TURN_PWM, -TURN_PWM);
  assert(commandedLeft == 0 && commandedRight == 0);
  simulatedMicros += MOTOR_DIRECTION_GUARD_US;
  moveLFR(TURN_PWM, -TURN_PWM);
  assert(commandedLeft == TURN_PWM && commandedRight == -TURN_PWM);

  // Host-instrumented sensor/control work remains within one control period.
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
