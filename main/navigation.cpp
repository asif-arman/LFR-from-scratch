#include "navigation.h"

#include "config.h"
#include "hardware.h"
#include "navigation_settings.h"
#include "robot.h"
#include "settings.h"


constexpr uint16_t ALL_SENSOR_MASK =
    ((uint16_t)1 << SENSOR_COUNT) - 1;
constexpr uint16_t CENTER_REGION_MASK = 0x01E0; // S5..S8


enum NavState : uint8_t
{
  STATE_FOLLOW,
  STATE_GAP,
  STATE_JUNCTION_PROBE,
  STATE_TURN,
  STATE_U_TURN,
  STATE_REACQUIRE,
  STATE_STOPPED
};


enum ReacquireMotion : uint8_t
{
  REACQUIRE_FORWARD,
  REACQUIRE_TURN,
  REACQUIRE_U_TURN
};


enum SensorFrameKind : uint8_t
{
  FRAME_NO_LINE,
  FRAME_NORMAL_LINE,
  FRAME_JUNCTION_FEATURE
};


static NavState state = STATE_STOPPED;
static NavigationResult terminalResult = NAVIGATION_ACTIVE;
static uint32_t nextControlAtMicros = 0;
static uint16_t worstSensorFrameMicros = 0;
static uint16_t worstControlTickMicros = 0;
static uint32_t gapStartedAtMillis = 0;
static uint32_t turnBlankStartedAtMillis = 0;

static int16_t previousError = 0;
static bool previousErrorValid = false;
static int16_t lastForwardLeft = 0;
static int16_t lastForwardRight = 0;
static bool lastForwardValid = false;
static int16_t lastLinePosition = LINE_CENTER;
static int8_t lastLineSide = 1;

static int16_t gapLeft = 0;
static int16_t gapRight = 0;
static uint8_t reacquireStableCount = 0;
static ReacquireMotion reacquireMotion = REACQUIRE_FORWARD;

static uint8_t leftBranchFrames = 0;
static uint8_t rightBranchFrames = 0;
static uint8_t junctionFeatureFrames = 0;
static uint8_t probeLeftFrames = 0;
static uint8_t probeRightFrames = 0;
static uint8_t probeClearFrames = 0;
static bool sawLeft = false;
static bool sawStraight = false;
static bool sawRight = false;
static bool junctionLocked = false;

static int8_t turnDirection = 1;
static uint8_t oldCenterAbsentCount = 0;
static bool oldCenterWasLeft = false;

static bool startBoxPending = false;
static bool finishArmed = false;
static uint8_t finishArmTicks = 0;
static uint8_t finishCandidateTicks = 0;


static uint8_t activeCount(uint16_t pattern)
{
  uint8_t count = 0;
  while (pattern != 0)
  {
    count += pattern & 1;
    pattern >>= 1;
  }
  return count;
}


static uint8_t patternSpan(uint16_t pattern)
{
  uint8_t first = 0;
  while (first < SENSOR_COUNT &&
         !(pattern & ((uint16_t)1 << first)))
  {
    first++;
  }
  if (first == SENSOR_COUNT) return 0;

  uint8_t last = SENSOR_COUNT - 1;
  while (!(pattern & ((uint16_t)1 << last))) last--;
  return last - first;
}


static uint8_t patternGroups(uint16_t pattern)
{
  uint8_t groups = 0;
  bool insideGroup = false;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    const bool active = (pattern & ((uint16_t)1 << sensor)) != 0;
    if (active && !insideGroup) groups++;
    insideGroup = active;
  }
  return groups;
}


static bool hasAdjacentCluster(uint16_t pattern,
                               uint8_t firstSensor,
                               uint8_t lastSensor)
{
  uint8_t run = 0;
  for (uint8_t sensor = firstSensor; sensor <= lastSensor; sensor++)
  {
    if (pattern & ((uint16_t)1 << sensor))
    {
      if (++run >= SIDE_CLUSTER_MIN_ACTIVE) return true;
    }
    else
    {
      run = 0;
    }
  }
  return false;
}


static bool centerSeesLine(uint16_t pattern)
{
  return (pattern & CENTER_REGION_MASK) != 0;
}


static bool isUsableLine(uint16_t pattern)
{
  const uint8_t count = activeCount(pattern);
  return count > 0 &&
         count <= USABLE_LINE_MAX_ACTIVE &&
         patternSpan(pattern) <= USABLE_LINE_MAX_SPAN &&
         patternGroups(pattern) == 1;
}


static bool isCredibleCenterLine(uint16_t pattern)
{
  return isUsableLine(pattern) && centerSeesLine(pattern);
}


static bool isBoxPattern(uint16_t pattern)
{
  return activeCount(pattern) >= BOX_MIN_ACTIVE &&
         patternSpan(pattern) >= BOX_MIN_SPAN;
}


static bool leftSideEvidence(uint16_t pattern)
{
  if (!hasAdjacentCluster(pattern, 10, 13)) return false;
  return patternGroups(pattern) >= 2 ||
      (centerSeesLine(pattern) &&
       patternSpan(pattern) >= JUNCTION_SIDE_MIN_SPAN);
}


static bool rightSideEvidence(uint16_t pattern)
{
  if (!hasAdjacentCluster(pattern, 0, 3)) return false;
  return patternGroups(pattern) >= 2 ||
      (centerSeesLine(pattern) &&
       patternSpan(pattern) >= JUNCTION_SIDE_MIN_SPAN);
}


static SensorFrameKind classifySensorFrame(uint16_t pattern)
{
  if (pattern == 0) return FRAME_NO_LINE;
  if (isUsableLine(pattern)) return FRAME_NORMAL_LINE;
  return FRAME_JUNCTION_FEATURE;
}


static bool isTurnCaptureLine(uint16_t pattern,
                              SensorFrameKind frameKind)
{
  if (frameKind == FRAME_NORMAL_LINE) return true;

  // A thick line can be classified as a wide feature even though it is the
  // outgoing corner line. One strong contiguous group touching the centre is
  // safe to capture; separated groups remain junction evidence.
  return patternGroups(pattern) == 1 &&
      activeCount(pattern) <= TURN_CAPTURE_MAX_ACTIVE &&
      centerSeesLine(pattern);
}


static void resetPd()
{
  previousError = 0;
  previousErrorValid = false;
}


static int16_t calculateAnalogLinePosition(const uint16_t sensorValues[])
{
  uint32_t weightedSum = 0;
  uint32_t strengthSum = 0;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    const uint16_t value = min(sensorValues[sensor], (uint16_t)1023);
    const uint16_t strength = settingsBlackStrength(sensor, value);
    if (strength < ANALOG_NOISE_FLOOR) continue;

    weightedSum += (uint32_t)strength *
        sensor * SENSOR_POSITION_SCALE;
    strengthSum += strength;
  }

  return strengthSum == 0 ? -1 : weightedSum / strengthSum;
}


static void mixPdCorrection(int16_t correction,
                            uint8_t speed,
                            int16_t &left,
                            int16_t &right)
{
  if (correction >= 0)
  {
    left = speed;
    right = speed - correction;
  }
  else
  {
    left = speed + correction;
    right = speed;
  }
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
}


static uint8_t pdDriveSpeed(int16_t position, uint8_t speedCap)
{
  int16_t error = calculateLineError(position);
  if (error < 0) error = -error;
  uint8_t correctionSpeed = baseSpeed;
  if (error >= OUTER_ERROR_THRESHOLD) correctionSpeed = OUTER_ERROR_PWM;
  else if (error >= MODERATE_ERROR_THRESHOLD)
    correctionSpeed = MODERATE_ERROR_PWM;
  return min(speedCap, correctionSpeed);
}


static void applyPd(int16_t position, uint8_t speed)
{
  if (position < 0)
  {
    return;
  }

  const int16_t error = calculateLineError(position);
  const int16_t delta = previousErrorValid ? error - previousError : 0;
  int32_t correction = ((int32_t)kpX100 * error +
                        (int32_t)kdX100 * delta) / 100;
  if (correction > baseSpeed) correction = baseSpeed;
  else if (correction < -(int16_t)baseSpeed)
    correction = -(int16_t)baseSpeed;

  previousError = error;
  previousErrorValid = true;

  speed = pdDriveSpeed(position, speed);
  int16_t left;
  int16_t right;
  mixPdCorrection((int16_t)correction, speed, left, right);
  lastForwardLeft = left;
  lastForwardRight = right;
  lastForwardValid = true;
  moveLFR(left, right);
}


static void rememberOrdinaryLine(int16_t position)
{
  if (position < 0) return;
  lastLinePosition = position;
  lastLineSide = position > LINE_CENTER ? -1 : 1;
}


static bool lastLineWasSharpRight()
{
  return lastLinePosition <= RIGHT_EDGE_MAX_POSITION;
}


static bool lastLineWasSharpLeft()
{
  return lastLinePosition >= LEFT_EDGE_MIN_POSITION;
}


static void resetJunctionDetection()
{
  leftBranchFrames = 0;
  rightBranchFrames = 0;
  junctionFeatureFrames = 0;
}


static void safeStop(NavigationResult result)
{
  stopMotors();
  resetPd();
  state = STATE_STOPPED;
  terminalResult = result;
  junctionLocked = false;
}


static void setReacquireState(ReacquireMotion motion)
{
  state = STATE_REACQUIRE;
  reacquireStableCount = 0;
  reacquireMotion = motion;
  resetPd();
}


static void resumeFollowWithPd(int16_t linePosition)
{
  state = STATE_FOLLOW;
  resetPd();
  applyPd(linePosition, baseSpeed);
}


static void beginBrakeReacquire(ReacquireMotion motion)
{
  // A signed pivot has rotational momentum. Brake for this control interval;
  // the next fresh frame starts capped PD instead of jumping straight to 200.
  setReacquireState(motion);
  brakeMotors();
}


static void commandTurn()
{
  moveLFR(turnDirection * TURN_PWM,
          -turnDirection * TURN_PWM);
}


static void commandUTurn()
{
  moveLFR(turnDirection * UTURN_PWM,
          -turnDirection * UTURN_PWM);
}


static void beginTurn(RouteDirection route,
                      uint8_t confirmedAbsentFrames = 0)
{
  turnDirection = route == ROUTE_LEFT ? -1 : 1;
  state = STATE_TURN;
  oldCenterAbsentCount = min(confirmedAbsentFrames,
                             TURN_CENTER_LOST_TICKS);
  oldCenterWasLeft = oldCenterAbsentCount >= TURN_CENTER_LOST_TICKS;
  turnBlankStartedAtMillis = confirmedAbsentFrames == 0 ? 0 : millis();
  lastForwardValid = false;
  resetPd();
  commandTurn();
}


static void beginUTurn(bool oldLineAlreadyGone)
{
  turnDirection = lastLineSide < 0 ? -1 : 1;
  state = STATE_U_TURN;
  oldCenterAbsentCount = oldLineAlreadyGone ? TURN_CENTER_LOST_TICKS : 0;
  oldCenterWasLeft = oldLineAlreadyGone;
  lastForwardValid = false;
  resetPd();
  commandUTurn();
}


static void beginGap()
{
  if (lastForwardValid)
  {
    const int16_t maximum = lastForwardLeft > lastForwardRight ?
        lastForwardLeft : lastForwardRight;
    if (maximum > GAP_MAX_PWM)
    {
      gapLeft = ((int32_t)lastForwardLeft * GAP_MAX_PWM) / maximum;
      gapRight = ((int32_t)lastForwardRight * GAP_MAX_PWM) / maximum;
    }
    else
    {
      gapLeft = lastForwardLeft;
      gapRight = lastForwardRight;
    }
  }
  else
  {
    gapLeft = GAP_MAX_PWM;
    gapRight = GAP_MAX_PWM;
  }

  gapLeft = constrain(gapLeft, 0, 255);
  gapRight = constrain(gapRight, 0, 255);
  state = STATE_GAP;
  gapStartedAtMillis = millis();
  resetPd();
  moveLFR(gapLeft, gapRight);
}


static void beginJunctionProbe(uint16_t pattern)
{
  state = STATE_JUNCTION_PROBE;
  junctionLocked = true;
  sawLeft = leftBranchFrames >= SIDE_CONFIRM_TICKS;
  sawRight = rightBranchFrames >= SIDE_CONFIRM_TICKS;
  sawStraight = false;
  probeLeftFrames = sawLeft ? SIDE_CONFIRM_TICKS : 0;
  probeRightFrames = sawRight ? SIDE_CONFIRM_TICKS : 0;
  probeClearFrames = 0;
  if (leftSideEvidence(pattern)) sawLeft = true;
  if (rightSideEvidence(pattern)) sawRight = true;
  resetPd();
  moveLFR(JUNCTION_CRAWL_PWM, JUNCTION_CRAWL_PWM);
}


static bool routeIsAvailable(RouteDirection route)
{
  if (route == ROUTE_LEFT) return sawLeft;
  if (route == ROUTE_STRAIGHT) return sawStraight;
  if (route == ROUTE_RIGHT) return sawRight;
  return false;
}


static RouteDirection chooseRoute()
{
  for (uint8_t index = 0; index < 3; index++)
  {
    const RouteDirection route = settingsPriorityAt(index);
    if (routeIsAvailable(route)) return route;
  }
  return ROUTE_NONE;
}


static bool commitRoute(RouteDirection route, int16_t linePosition)
{
  if (route == ROUTE_NONE) return false;

  if (route == ROUTE_LEFT || route == ROUTE_RIGHT)
  {
    beginTurn(route, linePosition < 0 ? TURN_CENTER_LOST_TICKS : 0);
  }
  else if (route == ROUTE_STRAIGHT)
  {
    setReacquireState(REACQUIRE_FORWARD);
    if (linePosition >= 0)
    {
      applyPd(linePosition, baseSpeed);
      reacquireStableCount = 1;
    }
    else moveLFR(JUNCTION_CRAWL_PWM, JUNCTION_CRAWL_PWM);
  }
  return true;
}


static bool finishJunctionProbe(int16_t linePosition)
{
  return commitRoute(chooseRoute(), linePosition);
}


static void clearJunctionLock()
{
  junctionLocked = false;
  sawLeft = false;
  sawStraight = false;
  sawRight = false;
  resetJunctionDetection();
}


static bool updateFinishBox(uint16_t pattern)
{
  if (!boxMode || !finishArmed)
  {
    finishCandidateTicks = 0;
    return false;
  }

  if (isBoxPattern(pattern))
  {
    if (finishCandidateTicks < FINISH_CONFIRM_TICKS)
      finishCandidateTicks++;
    if (finishCandidateTicks >= FINISH_CONFIRM_TICKS)
    {
      safeStop(NAVIGATION_FINISHED);
      return true;
    }
  }
  else
  {
    finishCandidateTicks = 0;
  }
  return false;
}


static NavigationResult tickFollow(uint16_t pattern,
                                   SensorFrameKind frameKind,
                                   int16_t linePosition)
{
  if (frameKind == FRAME_NO_LINE)
  {
    if (!junctionLocked && lastLineWasSharpRight())
    {
      // The all-white frame proves that the old edge line has left the array.
      beginTurn(ROUTE_RIGHT, TURN_CENTER_LOST_TICKS);
      return NAVIGATION_ACTIVE;
    }
    if (!junctionLocked && lastLineWasSharpLeft())
    {
      beginTurn(ROUTE_LEFT, TURN_CENTER_LOST_TICKS);
      return NAVIGATION_ACTIVE;
    }
    beginGap();
    return NAVIGATION_ACTIVE;
  }

  if (boxMode && isUsableLine(pattern) && !finishArmed)
  {
    if (finishArmTicks < FINISH_ARM_TICKS) finishArmTicks++;
    if (finishArmTicks >= FINISH_ARM_TICKS) finishArmed = true;
  }

  if (!junctionLocked && frameKind == FRAME_JUNCTION_FEATURE)
  {
    if (leftSideEvidence(pattern))
    {
      if (leftBranchFrames < SIDE_CONFIRM_TICKS) leftBranchFrames++;
    }
    else leftBranchFrames = 0;

    if (rightSideEvidence(pattern))
    {
      if (rightBranchFrames < SIDE_CONFIRM_TICKS) rightBranchFrames++;
    }
    else rightBranchFrames = 0;

    if (junctionFeatureFrames < SIDE_CONFIRM_TICKS) junctionFeatureFrames++;
    if (leftBranchFrames >= SIDE_CONFIRM_TICKS ||
        rightBranchFrames >= SIDE_CONFIRM_TICKS ||
        junctionFeatureFrames >= SIDE_CONFIRM_TICKS)
    {
      beginJunctionProbe(pattern);
      return NAVIGATION_ACTIVE;
    }

    moveLFR(JUNCTION_CRAWL_PWM, JUNCTION_CRAWL_PWM);
    return NAVIGATION_ACTIVE;
  }

  resetJunctionDetection();
  applyPd(linePosition, baseSpeed);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickGap(uint16_t pattern,
                                SensorFrameKind frameKind,
                                int16_t linePosition)
{
  if (frameKind == FRAME_NORMAL_LINE)
  {
    // One dot is enough: leave GAP now, so the next white frame starts a
    // completely new uninterrupted-gap allowance.
    resumeFollowWithPd(linePosition);
    return NAVIGATION_ACTIVE;
  }
  else if (frameKind == FRAME_JUNCTION_FEATURE)
  {
    resetJunctionDetection();
    beginJunctionProbe(pattern);
    return NAVIGATION_ACTIVE;
  }

  // Milliseconds make every dot independent of sensor/loop timing. Only one
  // uninterrupted 200 ms blank interval is a true lost-line condition.
  if ((uint32_t)(millis() - gapStartedAtMillis) >= GAP_ALLOWANCE_MS)
  {
    if (junctionLocked)
    {
      setReacquireState(REACQUIRE_FORWARD);
      moveLFR(JUNCTION_CRAWL_PWM, JUNCTION_CRAWL_PWM);
      return NAVIGATION_ACTIVE;
    }
    beginUTurn(true);
    return NAVIGATION_ACTIVE;
  }

  moveLFR(gapLeft, gapRight);
  return NAVIGATION_ACTIVE;
}


static void updateProbeSideEvidence(uint16_t pattern)
{
  if (leftSideEvidence(pattern))
  {
    if (probeLeftFrames < SIDE_CONFIRM_TICKS) probeLeftFrames++;
    if (probeLeftFrames >= SIDE_CONFIRM_TICKS) sawLeft = true;
  }
  else probeLeftFrames = 0;

  if (rightSideEvidence(pattern))
  {
    if (probeRightFrames < SIDE_CONFIRM_TICKS) probeRightFrames++;
    if (probeRightFrames >= SIDE_CONFIRM_TICKS) sawRight = true;
  }
  else probeRightFrames = 0;
}


static NavigationResult tickJunctionProbe(uint16_t pattern,
                                           SensorFrameKind frameKind,
                                           int16_t linePosition)
{
  updateProbeSideEvidence(pattern);

  if (frameKind == FRAME_JUNCTION_FEATURE)
  {
    probeClearFrames = 0;
  }
  else
  {
    if (probeClearFrames < JUNCTION_CLEAR_TICKS) probeClearFrames++;
    if (probeClearFrames >= JUNCTION_CLEAR_TICKS)
    {
      sawStraight = isCredibleCenterLine(pattern);
      if (finishJunctionProbe(linePosition)) return NAVIGATION_ACTIVE;
      probeClearFrames = JUNCTION_CLEAR_TICKS;
    }
  }
  moveLFR(JUNCTION_CRAWL_PWM, JUNCTION_CRAWL_PWM);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickTurn(uint16_t pattern,
                                 SensorFrameKind frameKind,
                                 int16_t linePosition)
{
  if (pattern == 0)
  {
    if (turnBlankStartedAtMillis == 0)
      turnBlankStartedAtMillis = millis();
  }
  else turnBlankStartedAtMillis = 0;

  if (!oldCenterWasLeft)
  {
    if (!centerSeesLine(pattern))
    {
      if (oldCenterAbsentCount < TURN_CENTER_LOST_TICKS)
        oldCenterAbsentCount++;
      if (oldCenterAbsentCount >= TURN_CENTER_LOST_TICKS)
        oldCenterWasLeft = true;
    }
    else oldCenterAbsentCount = 0;
  }

  if (oldCenterWasLeft && linePosition >= 0 &&
      isTurnCaptureLine(pattern, frameKind))
  {
    // A centred thick corner can look like a junction for one frame. Capture
    // it now, brake rotational momentum, then let capped PD settle the robot.
    beginBrakeReacquire(REACQUIRE_TURN);
    return NAVIGATION_ACTIVE;
  }

  if (frameKind == FRAME_JUNCTION_FEATURE)
  {
    // A junction is line evidence, never permission to keep rotating or to
    // escalate loss recovery. Let the normal junction chooser handle it.
    resetJunctionDetection();
    beginJunctionProbe(pattern);
    return NAVIGATION_ACTIVE;
  }

  if (pattern == 0 && turnBlankStartedAtMillis != 0 &&
      (uint32_t)(millis() - turnBlankStartedAtMillis) >= GAP_ALLOWANCE_MS)
  {
    // An isolated edge dot may suggest a corner briefly, but continuous blank
    // sensors must not leave TURN latched forever. It is now confirmed loss.
    beginUTurn(true);
    return NAVIGATION_ACTIVE;
  }

  commandTurn();
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickUTurn(uint16_t pattern,
                                  SensorFrameKind frameKind,
                                  int16_t linePosition)
{
  if (!oldCenterWasLeft)
  {
    if (!centerSeesLine(pattern))
    {
      if (oldCenterAbsentCount < TURN_CENTER_LOST_TICKS)
        oldCenterAbsentCount++;
      if (oldCenterAbsentCount >= TURN_CENTER_LOST_TICKS)
        oldCenterWasLeft = true;
    }
    else oldCenterAbsentCount = 0;
  }

  if (oldCenterWasLeft && linePosition >= 0 &&
      isTurnCaptureLine(pattern, frameKind))
  {
    // A newly encountered line always wins over loss recovery. Brake this
    // tick so the U-turn cannot carry the robot across it at speed.
    beginBrakeReacquire(REACQUIRE_U_TURN);
    return NAVIGATION_ACTIVE;
  }

  if (frameKind == FRAME_JUNCTION_FEATURE)
  {
    resetJunctionDetection();
    beginJunctionProbe(pattern);
    return NAVIGATION_ACTIVE;
  }

  commandUTurn();
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickReacquire(uint16_t pattern,
                                      SensorFrameKind frameKind,
                                      int16_t linePosition)
{
  if (startBoxPending)
  {
    if (linePosition >= 0 && centerSeesLine(pattern))
    {
      applyPd(linePosition, baseSpeed);
      if (reacquireStableCount < START_EXIT_CONFIRM_TICKS)
        reacquireStableCount++;
      if (reacquireStableCount >= START_EXIT_CONFIRM_TICKS)
      {
        startBoxPending = false;
        finishArmTicks = 0;
        finishArmed = false;
        state = STATE_FOLLOW;
        resetPd();
      }
    }
    else
    {
      reacquireStableCount = 0;
      moveLFR(START_BOX_EXIT_PWM, START_BOX_EXIT_PWM);
    }
    return NAVIGATION_ACTIVE;
  }

  const bool capturedLine = linePosition >= 0 &&
      (frameKind == FRAME_NORMAL_LINE ||
       reacquireMotion == REACQUIRE_TURN ||
       reacquireMotion == REACQUIRE_U_TURN);
  if (capturedLine)
  {
    // The cap is temporary: it removes pivot momentum without changing the
    // user's saved 200 straight-line speed. Three fresh frames prove stability.
    applyPd(linePosition, REACQUIRE_PWM);
    if (reacquireStableCount < REACQUIRE_CONFIRM_TICKS)
      reacquireStableCount++;
    if (reacquireStableCount >= REACQUIRE_CONFIRM_TICKS)
    {
      if (junctionLocked) clearJunctionLock();
      state = STATE_FOLLOW;
      resetPd();
    }
    return NAVIGATION_ACTIVE;
  }

  if (!junctionLocked && frameKind == FRAME_JUNCTION_FEATURE)
  {
    resetJunctionDetection();
    beginJunctionProbe(pattern);
    return NAVIGATION_ACTIVE;
  }

  reacquireStableCount = 0;
  if (reacquireMotion == REACQUIRE_TURN) commandTurn();
  else if (reacquireMotion == REACQUIRE_U_TURN) commandUTurn();
  else moveLFR(JUNCTION_CRAWL_PWM, JUNCTION_CRAWL_PWM);
  return NAVIGATION_ACTIVE;
}


void navigationStart()
{
  state = boxMode ? STATE_REACQUIRE : STATE_FOLLOW;
  terminalResult = NAVIGATION_ACTIVE;
  gapStartedAtMillis = 0;
  turnBlankStartedAtMillis = 0;
  lastForwardLeft = 0;
  lastForwardRight = 0;
  lastForwardValid = false;
  lastLinePosition = LINE_CENTER;
  lastLineSide = 1;
  reacquireStableCount = 0;
  reacquireMotion = REACQUIRE_FORWARD;
  junctionLocked = false;
  oldCenterAbsentCount = 0;
  oldCenterWasLeft = false;
  startBoxPending = boxMode;
  finishArmed = false;
  finishArmTicks = 0;
  finishCandidateTicks = 0;
  sawLeft = false;
  sawStraight = false;
  sawRight = false;
  resetJunctionDetection();
  resetPd();
  nextControlAtMicros = micros();
  worstSensorFrameMicros = 0;
  worstControlTickMicros = 0;
  stopMotors();
}


void navigationStop()
{
  safeStop(NAVIGATION_ACTIVE);
}


NavigationResult navigationTick(uint16_t sensorValues[])
{
  if (state == STATE_STOPPED)
  {
    stopMotors();
    return terminalResult;
  }

  const uint32_t scheduledAt = micros();
  if ((int32_t)(scheduledAt - nextControlAtMicros) < 0)
    return NAVIGATION_ACTIVE;

  nextControlAtMicros += CONTROL_PERIOD_US;
  if ((int32_t)(scheduledAt - nextControlAtMicros) >= 0)
    nextControlAtMicros = scheduledAt + CONTROL_PERIOD_US;

  const uint32_t controlStartedAt = micros();
  const uint32_t sensorStartedAt = micros();
  readSensors(sensorValues);
  const uint32_t sensorElapsed = micros() - sensorStartedAt;
  if (sensorElapsed > worstSensorFrameMicros)
    worstSensorFrameMicros = sensorElapsed > 65535UL ? 65535 : sensorElapsed;

  const uint16_t pattern = makeSensorPattern(sensorValues);
  const SensorFrameKind frameKind = classifySensorFrame(pattern);
  // Classify every fresh frame before motion. During a pivot, calculate a
  // position for a strong centred thick line too, so it cannot be ignored.
  const bool turnCaptureLine = isTurnCaptureLine(pattern, frameKind);
  const int16_t linePosition =
      (frameKind == FRAME_NORMAL_LINE || turnCaptureLine) ?
      calculateAnalogLinePosition(sensorValues) : -1;
  if (frameKind == FRAME_NORMAL_LINE && linePosition >= 0)
    rememberOrdinaryLine(linePosition);

  NavigationResult result = NAVIGATION_ACTIVE;
  if (updateFinishBox(pattern))
  {
    result = NAVIGATION_FINISHED;
  }
  else switch (state)
  {
    case STATE_FOLLOW:
      result = tickFollow(pattern, frameKind, linePosition);
      break;
    case STATE_GAP:
      result = tickGap(pattern, frameKind, linePosition);
      break;
    case STATE_JUNCTION_PROBE:
      result = tickJunctionProbe(pattern, frameKind, linePosition);
      break;
    case STATE_TURN:
      result = tickTurn(pattern, frameKind, linePosition);
      break;
    case STATE_U_TURN:
      result = tickUTurn(pattern, frameKind, linePosition);
      break;
    case STATE_REACQUIRE:
      result = tickReacquire(pattern, frameKind, linePosition);
      break;
    default:
      safeStop(NAVIGATION_LOST);
      result = NAVIGATION_LOST;
      break;
  }

  const uint32_t controlElapsed = micros() - controlStartedAt;
  if (controlElapsed > worstControlTickMicros)
    worstControlTickMicros = controlElapsed > 65535UL ? 65535 : controlElapsed;
  return result;
}


uint16_t navigationWorstSensorFrameMicros()
{
  return worstSensorFrameMicros;
}


uint16_t navigationWorstControlTickMicros()
{
  return worstControlTickMicros;
}
