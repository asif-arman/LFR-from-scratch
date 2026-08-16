#include "navigation.h"

#include "config.h"
#include "hardware.h"
#include "navigation_settings.h"
#include "robot.h"
#include "settings.h"


constexpr uint16_t ALL_SENSOR_MASK =
    ((uint16_t)1 << SENSOR_COUNT) - 1;
constexpr uint16_t CENTER_SENSOR_MASK =
    ((uint16_t)1 << 6) | ((uint16_t)1 << 7);
constexpr uint16_t LEFT_BACKGROUND_MASK = 0x3F00;
constexpr uint16_t RIGHT_BACKGROUND_MASK = 0x003F;


enum NavState : uint8_t
{
  STATE_START_BOX,
  STATE_FOLLOW,
  STATE_FORWARD_PROBE,
  STATE_PROBE_BRAKE,
  STATE_TURN_TO_ROUTE,
  STATE_ROUTE_COMMIT,
  STATE_U_TURN,
  STATE_STOPPED
};


enum ProbeReason : uint8_t
{
  PROBE_LOST_LINE,
  PROBE_WIDE_FEATURE
};


enum TurnPhase : uint8_t
{
  TURN_LEAVE_CENTER,
  TURN_CAPTURE_SIDE,
  TURN_REACQUIRE_CENTER,
  TURN_BRAKE,
  TURN_FORWARD_CONFIRM
};


static NavState state = STATE_STOPPED;
static ProbeReason probeReason = PROBE_LOST_LINE;
static NavigationResult terminalResult = NAVIGATION_ACTIVE;

static uint32_t nextControlAtMicros = 0;
static uint16_t worstSensorFrameMicros = 0;
static uint16_t worstControlTickMicros = 0;
static uint16_t stateTicks = 0;
static uint8_t inverseRearmStableCount = 0;
static bool inverseTransitionArmed = true;

static int16_t previousError = 0;
static bool previousErrorValid = false;
static int16_t previousFollowLeft = 0;
static int16_t previousFollowRight = 0;
static bool followMixerValid = false;
static int8_t lastReliableSide = 1;
static const uint16_t *frameSensorValues = 0;
static int16_t gapProbeLeft = 0;
static int16_t gapProbeRight = 0;

static int16_t recentErrors[3];
static uint8_t recentErrorCount = 0;
static uint8_t outwardTrendCount = 0;

static uint8_t centerStableCount = 0;
static uint8_t exitStableCount = 0;
static uint8_t inverseConfirmCount = 0;
static uint8_t probeNarrowConfirmCount = 0;
static uint8_t finishCandidateTicks = 0;
static uint8_t finishArmTicks = 0;
static uint8_t junctionFrameCount = 0;
static uint8_t leftRouteVotes = 0;
static uint8_t straightRouteVotes = 0;
static uint8_t rightRouteVotes = 0;

static uint8_t junctionAvailableRoutes = 0;
static RouteDirection selectedRoute = ROUTE_NONE;
static bool junctionActive = false;
static bool lineIsInverse = false;
static bool startCleared = false;
static bool finishArmed = false;
static TurnPhase turnPhase = TURN_LEAVE_CENTER;
static uint16_t turnTicks = 0;
static uint8_t turnCenterLostCount = 0;
static uint8_t turnSideCaptureCount = 0;
static uint8_t turnBrakeCount = 0;
static uint8_t routeCommitLostCount = 0;


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


static int16_t calculatePdLinePosition(uint16_t pattern);


static bool hasAdjacentCluster(
    uint16_t pattern,
    uint8_t firstSensor,
    uint8_t lastSensor
)
{
  uint8_t run = 0;
  for (uint8_t sensor = firstSensor; sensor <= lastSensor; sensor++)
  {
    if (pattern & ((uint16_t)1 << sensor))
    {
      if (++run >= SIDE_CLUSTER_MIN_ACTIVE)
      {
        return true;
      }
    }
    else
    {
      run = 0;
    }
  }
  return false;
}


static uint8_t patternSpan(uint16_t pattern)
{
  uint8_t first = 0;
  while (first < SENSOR_COUNT &&
         !(pattern & ((uint16_t)1 << first)))
  {
    first++;
  }

  if (first == SENSOR_COUNT)
  {
    return 0;
  }

  uint8_t last = SENSOR_COUNT - 1;
  while (!(pattern & ((uint16_t)1 << last)))
  {
    last--;
  }

  return last - first;
}


static bool isNarrowLine(uint16_t pattern)
{
  const uint8_t count = activeCount(pattern);
  return count > 0 &&
         count <= NARROW_LINE_MAX_ACTIVE &&
         patternSpan(pattern) <= NARROW_LINE_MAX_SPAN;
}


static bool centerSeesLine(uint16_t pattern)
{
  return (pattern & CENTER_SENSOR_MASK) != 0;
}


static bool isBoxPattern(uint16_t pattern)
{
  return activeCount(pattern) >= BOX_MIN_ACTIVE &&
         patternSpan(pattern) >= BOX_MIN_SPAN;
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


static uint8_t routesInPattern(uint16_t pattern)
{
  uint8_t routes = 0;

  if (hasAdjacentCluster(pattern, 8, 13))
  {
    routes |= ROUTE_LEFT;
  }
  if (centerSeesLine(pattern))
  {
    routes |= ROUTE_STRAIGHT;
  }
  if (hasAdjacentCluster(pattern, 0, 5))
  {
    routes |= ROUTE_RIGHT;
  }

  return routes;
}


static uint8_t routeCount(uint8_t routes)
{
  uint8_t count = 0;
  if (routes & ROUTE_LEFT) count++;
  if (routes & ROUTE_STRAIGHT) count++;
  if (routes & ROUTE_RIGHT) count++;
  return count;
}


static uint16_t absoluteError(int16_t error)
{
  return error < 0 ? (uint16_t)-error : (uint16_t)error;
}


static bool isCredibleStraightContinuation(uint16_t pattern)
{
  if (!isNarrowLine(pattern)) return false;
  const int16_t position = calculatePdLinePosition(pattern);
  return position >= 0 &&
         absoluteError(calculateLineError(position)) <=
             STRAIGHT_CENTER_ERROR_LIMIT;
}


static int8_t sideFromRecentHistory()
{
  return lastReliableSide;
}


static void resetPd()
{
  previousError = 0;
  previousErrorValid = false;
  followMixerValid = false;
}


static void resetTurnIntent()
{
  recentErrorCount = 0;
  outwardTrendCount = 0;
}


static void rememberError(int16_t error)
{
  if (recentErrorCount < 3)
  {
    recentErrors[recentErrorCount++] = error;
  }
  else
  {
    recentErrors[0] = recentErrors[1];
    recentErrors[1] = recentErrors[2];
    recentErrors[2] = error;
  }

  if (recentErrorCount >= 2)
  {
    const int16_t previous = recentErrors[recentErrorCount - 2];
    const bool sameSide = (error < 0 && previous < 0) ||
                          (error > 0 && previous > 0);
    if (sameSide &&
        absoluteError(error) >= absoluteError(previous) + TURN_TREND_MIN_STEP)
    {
      if (outwardTrendCount < 255) outwardTrendCount++;
    }
    else
    {
      outwardTrendCount = 0;
    }
  }

  if (error < 0) lastReliableSide = -1;
  else if (error > 0) lastReliableSide = 1;
}


static bool outwardSharpTurnLikely()
{
  if (recentErrorCount == 0 ||
      outwardTrendCount < TURN_OUTWARD_CONFIRM)
    return false;
  return absoluteError(recentErrors[recentErrorCount - 1]) >=
      TURN_LOSS_ERROR_LIMIT;
}


static int16_t calculatePdLinePosition(uint16_t pattern)
{
  if (frameSensorValues == 0)
  {
    return calculateLinePosition(pattern);
  }

  uint32_t weightedSum = 0;
  uint32_t strengthSum = 0;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++)
  {
    if (!(pattern & ((uint16_t)1 << sensor))) continue;

    const uint16_t value = min(frameSensorValues[sensor], (uint16_t)1023);
    uint16_t strength = settingsBlackStrength(sensor, value);
    if (lineIsInverse) strength = 1000 - strength;

    weightedSum += (uint32_t)strength *
                   sensor * SENSOR_POSITION_SCALE;
    strengthSum += strength;
  }

  if (strengthSum == 0)
  {
    return calculateLinePosition(pattern);
  }
  return weightedSum / strengthSum;
}


static int16_t effectiveForwardPwm(int16_t pwm, uint8_t effectiveMinimum)
{
  return pwm > 0 && pwm < effectiveMinimum ? 0 : pwm;
}


static uint8_t calculateApproachSpeed(uint16_t absolute,
                                      uint8_t speedLimit)
{
  uint16_t approachSpeed = min(baseSpeed, speedLimit);
  if (absolute > ADAPTIVE_SPEED_START_ERROR)
  {
    const uint16_t reduction = min(
        (uint16_t)((absolute - ADAPTIVE_SPEED_START_ERROR) /
            ADAPTIVE_SPEED_REDUCTION_DIVISOR),
        (uint16_t)ADAPTIVE_SPEED_MAX_REDUCTION);
    approachSpeed = reduction >= approachSpeed ? 0 :
        approachSpeed - reduction;
  }

  return approachSpeed;
}


static void applyPd(uint16_t pattern, uint8_t speedLimit = 255)
{
  const int16_t position = calculatePdLinePosition(pattern);
  if (position < 0)
  {
    return;
  }

  const int16_t error = calculateLineError(position);
  int16_t correction = calculateProportionalCorrection(error);
  if (previousErrorValid)
  {
    const int16_t errorDelta = constrain(
        error - previousError,
        -DERIVATIVE_ERROR_DELTA_LIMIT,
        DERIVATIVE_ERROR_DELTA_LIMIT);
    correction += calculateDerivativeCorrection(
        previousError + errorDelta, previousError);
  }

  previousError = error;
  previousErrorValid = true;
  rememberError(error);

  const uint16_t absolute = absoluteError(error);
  const int16_t approachSpeed = calculateApproachSpeed(absolute, speedLimit);

  int16_t leftPwm = constrain(approachSpeed + correction,
                              0, 255);
  int16_t rightPwm = constrain(approachSpeed - correction,
                               0, 255);
  leftPwm = effectiveForwardPwm(leftPwm, LEFT_MOTOR_EFFECTIVE_MIN_PWM);
  rightPwm = effectiveForwardPwm(rightPwm, RIGHT_MOTOR_EFFECTIVE_MIN_PWM);
  previousFollowLeft = leftPwm;
  previousFollowRight = rightPwm;
  followMixerValid = true;

  moveLFR(leftPwm, rightPwm);
}


static void resetJunctionEvidence()
{
  junctionFrameCount = 0;
  leftRouteVotes = 0;
  straightRouteVotes = 0;
  rightRouteVotes = 0;
}


static void addRouteEvidence(uint8_t routes)
{
  if (junctionFrameCount < 255) junctionFrameCount++;
  if ((routes & ROUTE_LEFT) && leftRouteVotes < 255) leftRouteVotes++;
  if ((routes & ROUTE_STRAIGHT) && straightRouteVotes < 255) straightRouteVotes++;
  if ((routes & ROUTE_RIGHT) && rightRouteVotes < 255) rightRouteVotes++;
}


static uint8_t confirmedRoutes()
{
  uint8_t routes = 0;
  if (leftRouteVotes >= ROUTE_MIN_VOTES) routes |= ROUTE_LEFT;
  if (straightRouteVotes >= ROUTE_MIN_VOTES) routes |= ROUTE_STRAIGHT;
  if (rightRouteVotes >= ROUTE_MIN_VOTES) routes |= ROUTE_RIGHT;
  return routes;
}


static bool isInverseTransition(uint16_t blackPattern)
{
  const uint16_t candidateLine = lineIsInverse ?
      blackPattern : (ALL_SENSOR_MASK ^ blackPattern);
  const uint16_t candidateBackground = lineIsInverse ?
      (ALL_SENSOR_MASK ^ blackPattern) : blackPattern;

  return isNarrowLine(candidateLine) &&
         centerSeesLine(candidateLine) &&
         patternGroups(candidateLine) == 1 &&
         routesInPattern(candidateLine) == ROUTE_STRAIGHT &&
         (candidateBackground & LEFT_BACKGROUND_MASK) != 0 &&
         (candidateBackground & RIGHT_BACKGROUND_MASK) != 0;
}


static bool currentPolarityIsStable(uint16_t blackPattern)
{
  const uint16_t currentLine = lineIsInverse ?
      (ALL_SENSOR_MASK ^ blackPattern) : blackPattern;
  return isNarrowLine(currentLine) &&
         patternGroups(currentLine) == 1;
}


static bool updateInverseMode(uint16_t blackPattern)
{
  const bool transitionCandidate = isInverseTransition(blackPattern);

  if (!inverseTransitionArmed)
  {
    inverseConfirmCount = 0;
    if (!transitionCandidate && currentPolarityIsStable(blackPattern))
    {
      if (inverseRearmStableCount < INVERSE_REARM_STABLE_TICKS)
        inverseRearmStableCount++;
      if (inverseRearmStableCount >= INVERSE_REARM_STABLE_TICKS)
        inverseTransitionArmed = true;
    }
    else
    {
      inverseRearmStableCount = 0;
    }
    return false;
  }

  if (transitionCandidate)
  {
    if (++inverseConfirmCount >= INVERSE_CONFIRM_TICKS)
    {
      lineIsInverse = !lineIsInverse;
      inverseConfirmCount = 0;
      inverseRearmStableCount = 0;
      inverseTransitionArmed = false;

      // A polarity boundary looks wide in the old polarity. Cancel only that
      // temporary probe; an active junction selection remains locked.
      if (state == STATE_FORWARD_PROBE &&
          probeReason == PROBE_WIDE_FEATURE && !junctionActive)
      {
        state = STATE_FOLLOW;
        junctionAvailableRoutes = 0;
        resetJunctionEvidence();
      }
      resetPd();
      return true;
    }
  }
  else
  {
    inverseConfirmCount = 0;
  }
  return false;
}


static void beginForwardProbe(ProbeReason reason)
{
  const int16_t probeLimit = min(baseSpeed, FORWARD_PROBE_MAX_PWM);
  if (followMixerValid)
  {
    gapProbeLeft = constrain(previousFollowLeft,
                             (int16_t)GAP_PROBE_MIN_PWM, probeLimit);
    gapProbeRight = constrain(previousFollowRight,
                              (int16_t)GAP_PROBE_MIN_PWM, probeLimit);
    if (gapProbeLeft > gapProbeRight + GAP_STEERING_MAX_DELTA)
      gapProbeLeft = gapProbeRight + GAP_STEERING_MAX_DELTA;
    else if (gapProbeRight > gapProbeLeft + GAP_STEERING_MAX_DELTA)
      gapProbeRight = gapProbeLeft + GAP_STEERING_MAX_DELTA;
  }
  else
  {
    gapProbeLeft = probeLimit;
    gapProbeRight = probeLimit;
  }

  state = STATE_FORWARD_PROBE;
  probeReason = reason;
  stateTicks = 0;
  probeNarrowConfirmCount = 0;
  finishCandidateTicks = 0;
  if (junctionActive)
  {
    exitStableCount = 0;
  }
  resetPd();
}


static void beginProbeBrake()
{
  brakeMotors();
  state = STATE_PROBE_BRAKE;
  stateTicks = 0;
}


static void safeStop(NavigationResult result)
{
  stopMotors();
  state = STATE_STOPPED;
  terminalResult = result;
}


static void clearJunctionAfterExit();


static void beginRouteCommit()
{
  state = STATE_ROUTE_COMMIT;
  stateTicks = 0;
  exitStableCount = 0;
  routeCommitLostCount = 0;
  resetPd();
}


static void beginTurn(RouteDirection route, bool centerAlreadyLost = false)
{
  selectedRoute = route;
  state = STATE_TURN_TO_ROUTE;
  stateTicks = 0;
  turnTicks = 0;
  centerStableCount = 0;
  turnCenterLostCount = 0;
  turnSideCaptureCount = 0;
  turnBrakeCount = 0;
  turnPhase = centerAlreadyLost ? TURN_CAPTURE_SIDE : TURN_LEAVE_CENTER;
  resetPd();
  resetTurnIntent();
}


static void beginUTurn()
{
  selectedRoute = ROUTE_U_TURN;
  state = STATE_U_TURN;
  stateTicks = 0;
  centerStableCount = 0;
  resetPd();
  resetTurnIntent();
}


static void choosePriorityRoute()
{
  for (uint8_t index = 0; index < 3; index++)
  {
    const RouteDirection route = settingsPriorityAt(index);
    if (junctionAvailableRoutes & route)
    {
      selectedRoute = route;
      if (route == ROUTE_STRAIGHT)
      {
        beginRouteCommit();
      }
      else
      {
        beginTurn(route);
      }
      return;
    }
  }

  beginUTurn();
}


static void startJunction(uint8_t availableRoutes)
{
  junctionActive = true;
  // Freeze this mask. No later turn or exit frame may manufacture a route or
  // replace the selected OLED-priority decision inside the same junction.
  junctionAvailableRoutes = availableRoutes;
  resetJunctionEvidence();
  choosePriorityRoute();
}


static void clearJunctionAfterExit()
{
  junctionActive = false;
  junctionAvailableRoutes = 0;
  selectedRoute = ROUTE_NONE;
  resetJunctionEvidence();
}


static NavigationResult tickFollow(uint16_t blackPattern, uint16_t pattern)
{
  (void)blackPattern;

  const uint8_t count = activeCount(pattern);
  if (count == 0)
  {
    if (outwardSharpTurnLikely())
    {
      const RouteDirection turnRoute = lastReliableSide < 0 ?
          ROUTE_LEFT : ROUTE_RIGHT;
      const int8_t direction = turnRoute == ROUTE_LEFT ? -1 : 1;
      // The all-clear frame already proves the incoming centre line is gone.
      beginTurn(turnRoute, true);
      moveLFR(direction * TURN_PWM, -direction * TURN_PWM);
      return NAVIGATION_ACTIVE;
    }

    beginForwardProbe(PROBE_LOST_LINE);
    moveLFR(gapProbeLeft, gapProbeRight);
    return NAVIGATION_ACTIVE;
  }

  if (count >= WIDE_FEATURE_MIN_ACTIVE)
  {
    // Ambiguous wide features start the monitored probe immediately, before
    // multi-frame classification, so confirmation cannot add hidden travel.
    resetJunctionEvidence();
    addRouteEvidence(routesInPattern(pattern) & ~ROUTE_STRAIGHT);
    beginForwardProbe(PROBE_WIDE_FEATURE);
    if (boxMode && finishArmed && isBoxPattern(pattern))
      finishCandidateTicks = 1;
    moveLFR(min(baseSpeed, WIDE_AREA_CRAWL_PWM),
            min(baseSpeed, WIDE_AREA_CRAWL_PWM));
    return NAVIGATION_ACTIVE;
  }

  // A single continuous narrow group is an ordinary line, even when a curve
  // touches both the centre and a side route region.
  if (isNarrowLine(pattern) && patternGroups(pattern) == 1)
  {
    resetJunctionEvidence();
    if (boxMode && startCleared && !finishArmed)
    {
      if (finishArmTicks < FINISH_ARM_TICKS) finishArmTicks++;
      if (finishArmTicks >= FINISH_ARM_TICKS) finishArmed = true;
    }
    applyPd(pattern);
    return NAVIGATION_ACTIVE;
  }

  const uint8_t routes = routesInPattern(pattern);
  if (routeCount(routes) >= 2)
  {
    addRouteEvidence(routes);
    if (junctionFrameCount >= JUNCTION_CONFIRM_TICKS)
    {
      const uint8_t available = confirmedRoutes();
      if (routeCount(available) >= 2)
      {
        startJunction(available);
        return NAVIGATION_ACTIVE;
      }
    }
  }
  else
  {
    resetJunctionEvidence();
  }

  applyPd(pattern);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickStartBox(uint16_t pattern)
{
  stateTicks++;
  if (isCredibleStraightContinuation(pattern))
  {
    applyPd(pattern, REACQUIRE_FORWARD_PWM);
    if (++probeNarrowConfirmCount >= START_EXIT_CONFIRM_TICKS)
    {
      startCleared = true;
      finishArmTicks = 0;
      finishArmed = false;
      state = STATE_FOLLOW;
      stateTicks = 0;
      resetPd();
    }
    return NAVIGATION_ACTIVE;
  }

  probeNarrowConfirmCount = 0;
  moveLFR(START_BOX_EXIT_PWM, START_BOX_EXIT_PWM);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickForwardProbe(uint16_t pattern)
{
  stateTicks++;
  const uint8_t count = activeCount(pattern);

  if (probeReason == PROBE_LOST_LINE)
  {
    if (isNarrowLine(pattern))
    {
      applyPd(pattern, REACQUIRE_FORWARD_PWM);
      if (++probeNarrowConfirmCount >= LINE_CONFIRM_TICKS)
      {
        state = STATE_FOLLOW;
        stateTicks = 0;
        resetPd();
      }
      return NAVIGATION_ACTIVE;
    }

    probeNarrowConfirmCount = 0;
    if (stateTicks >= LOSS_PROBE_TICKS)
    {
      stopMotors();
      beginUTurn();
      return NAVIGATION_ACTIVE;
    }

    // Preserve a bounded form of the last FOLLOW steering through a short gap.
    moveLFR(gapProbeLeft, gapProbeRight);
    return NAVIGATION_ACTIVE;
  }

  if (boxMode && finishArmed)
  {
    if (isBoxPattern(pattern))
    {
      if (finishCandidateTicks < FINISH_CONFIRM_TICKS)
        finishCandidateTicks++;
      if (finishCandidateTicks >= FINISH_CONFIRM_TICKS)
      {
        safeStop(NAVIGATION_FINISHED);
        return NAVIGATION_FINISHED;
      }
    }
    else
    {
      finishCandidateTicks = 0;
    }
  }

  // Vote only during the compact initial evidence window; a persistent filled
  // area then crawls without manufacturing unlimited side-route evidence.
  if (stateTicks <= JUNCTION_CONFIRM_TICKS ||
      count < WIDE_FEATURE_MIN_ACTIVE)
  {
    addRouteEvidence(routesInPattern(pattern) & ~ROUTE_STRAIGHT);
  }

  if (isNarrowLine(pattern))
  {
    stopMotors();
    if (++probeNarrowConfirmCount < LINE_CONFIRM_TICKS)
      return NAVIGATION_ACTIVE;

    uint8_t observed = confirmedRoutes();
    if (isCredibleStraightContinuation(pattern))
      observed |= ROUTE_STRAIGHT;
    else
    {
      const int16_t error = calculateLineError(
          calculatePdLinePosition(pattern));
      observed |= error < 0 ? ROUTE_LEFT : ROUTE_RIGHT;
    }

    junctionAvailableRoutes |= observed;
    if (routeCount(junctionAvailableRoutes) >= 2)
      startJunction(junctionAvailableRoutes);
    else if (junctionAvailableRoutes & ROUTE_STRAIGHT)
    {
      state = STATE_FOLLOW;
      resetPd();
    }
    else if (junctionAvailableRoutes != 0)
      startJunction(junctionAvailableRoutes);
    else
      beginUTurn();
    return NAVIGATION_ACTIVE;
  }

  probeNarrowConfirmCount = 0;
  if (count == 0 && junctionFrameCount >= JUNCTION_CONFIRM_TICKS)
  {
    beginProbeBrake();
    return NAVIGATION_ACTIVE;
  }

  moveLFR(WIDE_AREA_CRAWL_PWM, WIDE_AREA_CRAWL_PWM);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickProbeBrake(uint16_t pattern)
{
  stateTicks++;
  brakeMotors();
  if (stateTicks < PROBE_BRAKE_TICKS)
  {
    return NAVIGATION_ACTIVE;
  }

  stopMotors();
  const uint8_t observed = confirmedRoutes();
  junctionAvailableRoutes |= observed & ~ROUTE_STRAIGHT;

  if (isCredibleStraightContinuation(pattern))
  {
    junctionAvailableRoutes |= ROUTE_STRAIGHT;
  }

  if (activeCount(pattern) >= WIDE_FEATURE_MIN_ACTIVE)
  {
    // A filled track area is not a finish. Resume the same probe without
    // restarting its evidence window and crawl until a narrow/empty exit.
    state = STATE_FORWARD_PROBE;
    stateTicks = 0;
    moveLFR(WIDE_AREA_CRAWL_PWM, WIDE_AREA_CRAWL_PWM);
    return NAVIGATION_ACTIVE;
  }

  if (junctionAvailableRoutes == 0)
  {
    beginUTurn();
    return NAVIGATION_ACTIVE;
  }

  startJunction(junctionAvailableRoutes);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickTurn(uint16_t pattern)
{
  stateTicks++;
  if (turnTicks < 65535) turnTicks++;
  if (turnTicks >= TURN_TIMEOUT_TICKS)
  {
    safeStop(NAVIGATION_LOST);
    return NAVIGATION_LOST;
  }

  const int8_t direction = selectedRoute == ROUTE_LEFT ? -1 : 1;
  const bool expectedSide = isNarrowLine(pattern) &&
      patternGroups(pattern) == 1 &&
      (direction < 0 ? hasAdjacentCluster(pattern, 8, 13) :
                       hasAdjacentCluster(pattern, 0, 5));

  if (turnPhase == TURN_LEAVE_CENTER)
  {
    if (!centerSeesLine(pattern))
    {
      if (turnCenterLostCount < TURN_CENTER_LOST_TICKS)
        turnCenterLostCount++;
    }
    else
    {
      turnCenterLostCount = 0;
    }

    if (turnCenterLostCount >= TURN_CENTER_LOST_TICKS)
      turnPhase = TURN_CAPTURE_SIDE;
    moveLFR(direction * TURN_PWM, -direction * TURN_PWM);
    return NAVIGATION_ACTIVE;
  }

  if (turnPhase == TURN_CAPTURE_SIDE)
  {
    if (expectedSide)
    {
      if (turnSideCaptureCount < TURN_SIDE_CAPTURE_TICKS)
        turnSideCaptureCount++;
    }
    else
    {
      turnSideCaptureCount = 0;
    }

    if (turnSideCaptureCount >= TURN_SIDE_CAPTURE_TICKS)
    {
      turnPhase = TURN_REACQUIRE_CENTER;
      centerStableCount = 0;
      moveLFR(direction * TURN_REACQUIRE_PWM,
              -direction * TURN_REACQUIRE_PWM);
    }
    else
    {
      moveLFR(direction * TURN_PWM, -direction * TURN_PWM);
    }
    return NAVIGATION_ACTIVE;
  }

  if (turnPhase == TURN_REACQUIRE_CENTER)
  {
    if (isCredibleStraightContinuation(pattern))
    {
      if (centerStableCount < TURN_REACQUIRE_TICKS) centerStableCount++;
      if (centerStableCount >= TURN_REACQUIRE_TICKS)
      {
        turnPhase = TURN_BRAKE;
        turnBrakeCount = 1;
        brakeMotors();
        return NAVIGATION_ACTIVE;
      }
    }
    else
    {
      centerStableCount = 0;
    }

    moveLFR(direction * TURN_REACQUIRE_PWM,
            -direction * TURN_REACQUIRE_PWM);
    return NAVIGATION_ACTIVE;
  }

  if (turnPhase == TURN_BRAKE)
  {
    brakeMotors();
    if (turnBrakeCount < TURN_BRAKE_TICKS) turnBrakeCount++;
    if (turnBrakeCount >= TURN_BRAKE_TICKS)
    {
      turnPhase = TURN_FORWARD_CONFIRM;
      centerStableCount = 0;
      resetPd();
    }
    return NAVIGATION_ACTIVE;
  }

  if (isCredibleStraightContinuation(pattern))
  {
    applyPd(pattern, REACQUIRE_FORWARD_PWM);
    if (centerStableCount < TURN_FORWARD_CONFIRM_TICKS) centerStableCount++;
    if (centerStableCount >= TURN_FORWARD_CONFIRM_TICKS)
      beginRouteCommit();
    return NAVIGATION_ACTIVE;
  }

  // The outgoing line was not stable after braking. Resume the same locked
  // turn; no route detection or priority lookup occurs here.
  centerStableCount = 0;
  turnPhase = TURN_REACQUIRE_CENTER;
  resetPd();
  moveLFR(direction * TURN_REACQUIRE_PWM,
          -direction * TURN_REACQUIRE_PWM);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickRouteCommit(uint16_t pattern)
{
  stateTicks++;
  const bool selectedTurn = selectedRoute == ROUTE_LEFT ||
                            selectedRoute == ROUTE_RIGHT;
  if (selectedTurn)
  {
    if (turnTicks < 65535) turnTicks++;
    if (turnTicks >= TURN_TIMEOUT_TICKS)
    {
      safeStop(NAVIGATION_LOST);
      return NAVIGATION_LOST;
    }
  }

  if (isCredibleStraightContinuation(pattern))
  {
    routeCommitLostCount = 0;
    if (exitStableCount < 255) exitStableCount++;
    applyPd(pattern, REACQUIRE_FORWARD_PWM);
  }
  else
  {
    exitStableCount = 0;
    resetPd();

    if (selectedTurn)
    {
      const int8_t direction = selectedRoute == ROUTE_LEFT ? -1 : 1;
      state = STATE_TURN_TO_ROUTE;
      turnPhase = TURN_REACQUIRE_CENTER;
      centerStableCount = 0;
      moveLFR(direction * TURN_REACQUIRE_PWM,
              -direction * TURN_REACQUIRE_PWM);
      return NAVIGATION_ACTIVE;
    }

    if (pattern == 0)
    {
      if (routeCommitLostCount < LOSS_PROBE_TICKS) routeCommitLostCount++;
      if (routeCommitLostCount >= LOSS_PROBE_TICKS)
      {
        beginUTurn();
        return NAVIGATION_ACTIVE;
      }
      moveLFR(REACQUIRE_FORWARD_PWM, REACQUIRE_FORWARD_PWM);
      return NAVIGATION_ACTIVE;
    }

    routeCommitLostCount = 0;
    moveLFR(WIDE_AREA_CRAWL_PWM, WIDE_AREA_CRAWL_PWM);
    return NAVIGATION_ACTIVE;
  }

  if (exitStableCount >= JUNCTION_EXIT_CONFIRM_TICKS)
  {
    clearJunctionAfterExit();
    state = STATE_FOLLOW;
    resetPd();
  }

  return NAVIGATION_ACTIVE;
}


static NavigationResult tickUTurn(uint16_t pattern)
{
  stateTicks++;
  const int8_t direction = sideFromRecentHistory();

  // The minimum guard prevents the departing line from ending the rotation.
  if (stateTicks >= UTURN_MIN_TICKS &&
      isCredibleStraightContinuation(pattern))
  {
    applyPd(pattern, REACQUIRE_FORWARD_PWM);
    if (++centerStableCount >= TURN_CENTER_CONFIRM_TICKS)
    {
      clearJunctionAfterExit();
      state = STATE_FOLLOW;
      stateTicks = 0;
      resetPd();
    }
    return NAVIGATION_ACTIVE;
  }

  centerStableCount = 0;
  if (stateTicks >= UTURN_TIMEOUT_TICKS)
  {
    safeStop(NAVIGATION_LOST);
    return NAVIGATION_LOST;
  }

  moveLFR(direction * UTURN_PWM,
          -direction * UTURN_PWM);
  return NAVIGATION_ACTIVE;
}


void navigationStart()
{
  state = boxMode ? STATE_START_BOX : STATE_FOLLOW;
  terminalResult = NAVIGATION_ACTIVE;
  stateTicks = 0;
  lineIsInverse = false;
  inverseRearmStableCount = 0;
  inverseTransitionArmed = true;
  centerStableCount = 0;
  inverseConfirmCount = 0;
  probeNarrowConfirmCount = 0;
  finishCandidateTicks = 0;
  finishArmTicks = 0;
  startCleared = !boxMode;
  finishArmed = false;
  lastReliableSide = 1;
  junctionActive = false;
  junctionAvailableRoutes = 0;
  selectedRoute = ROUTE_NONE;
  turnPhase = TURN_LEAVE_CENTER;
  turnTicks = 0;
  turnCenterLostCount = 0;
  turnSideCaptureCount = 0;
  turnBrakeCount = 0;
  routeCommitLostCount = 0;
  nextControlAtMicros = micros();
  worstSensorFrameMicros = 0;
  worstControlTickMicros = 0;
  resetJunctionEvidence();
  resetPd();
  resetTurnIntent();
  stopMotors();
}


void navigationStop()
{
  safeStop(NAVIGATION_ACTIVE);
  junctionActive = false;
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

  frameSensorValues = sensorValues;
  const uint16_t blackPattern = makeSensorPattern(sensorValues);

  const bool polarityState = !boxMode && (state == STATE_FOLLOW ||
      state == STATE_FORWARD_PROBE || state == STATE_ROUTE_COMMIT);
  if (polarityState) updateInverseMode(blackPattern);
  else inverseConfirmCount = 0;

  const uint16_t linePattern = !boxMode && lineIsInverse ?
      (ALL_SENSOR_MASK ^ blackPattern) : blackPattern;

  NavigationResult result = NAVIGATION_ACTIVE;
  switch (state)
  {
    case STATE_START_BOX:
      result = tickStartBox(linePattern);
      break;
    case STATE_FOLLOW:
      result = tickFollow(blackPattern, linePattern);
      break;
    case STATE_FORWARD_PROBE:
      result = tickForwardProbe(linePattern);
      break;
    case STATE_PROBE_BRAKE:
      result = tickProbeBrake(linePattern);
      break;
    case STATE_TURN_TO_ROUTE:
      result = tickTurn(linePattern);
      break;
    case STATE_ROUTE_COMMIT:
      result = tickRouteCommit(linePattern);
      break;
    case STATE_U_TURN:
      result = tickUTurn(linePattern);
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
