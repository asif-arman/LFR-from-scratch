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
  STATE_FOLLOW,
  STATE_FORWARD_PROBE,
  STATE_PROBE_BRAKE,
  STATE_TURN_TO_ROUTE,
  STATE_ROUTE_COMMIT,
  STATE_REVERSE_REACQUIRE,
  STATE_RECOVERY_SEARCH,
  STATE_STOPPED
};


enum ProbeReason : uint8_t
{
  PROBE_LOST_LINE,
  PROBE_WIDE_FEATURE
};


static NavState state = STATE_STOPPED;
static ProbeReason probeReason = PROBE_LOST_LINE;

static uint32_t stateStartedAt = 0;
static uint32_t probeStartedAt = 0;
static uint32_t commitStartedAt = 0;
static uint32_t lastInverseToggleAt = 0;
static uint32_t nextControlAtMicros = 0;
static uint16_t worstSensorFrameMicros = 0;
static uint16_t worstControlTickMicros = 0;

static int16_t previousError = 0;
static bool previousErrorValid = false;
static int16_t previousFollowLeft = 0;
static int16_t previousFollowRight = 0;
static bool followMixerValid = false;
static int8_t lastReliableSide = 1;
static int8_t searchFirstSide = 1;
static const uint16_t *frameSensorValues = 0;

static int16_t recentErrors[3];
static uint8_t recentErrorCount = 0;
static uint8_t outwardTrendCount = 0;

static uint8_t lostReadCount = 0;
static uint8_t centerStableCount = 0;
static uint8_t exitStableCount = 0;
static uint8_t inverseConfirmCount = 0;
static uint8_t probeNarrowConfirmCount = 0;
static uint8_t junctionFrameCount = 0;
static uint8_t leftRouteVotes = 0;
static uint8_t straightRouteVotes = 0;
static uint8_t rightRouteVotes = 0;

static uint8_t junctionAvailableRoutes = 0;
static uint8_t attemptedRoutes = 0;
static RouteDirection selectedRoute = ROUTE_NONE;
static bool junctionActive = false;
static bool turnSawCenterLost = false;
static bool uTurnAttempted = false;
static bool lineIsInverse = false;
static bool wideAreaCrawl = false;
static bool lostLineConfirmed = false;


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


static bool turnLossLikely()
{
  if (recentErrorCount == 0) return false;
  return outwardTrendCount >= TURN_OUTWARD_CONFIRM ||
         absoluteError(recentErrors[recentErrorCount - 1]) >=
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

    const uint16_t threshold = settingsThresholdForSensor(sensor);
    const uint16_t value = min(frameSensorValues[sensor], (uint16_t)1023);
    uint16_t strength = 0;
    if (!lineIsInverse && value > threshold)
    {
      const uint16_t range = threshold < 1023 ? 1023 - threshold : 1;
      strength = ((uint32_t)(value - threshold) * 1000UL) / range;
    }
    else if (lineIsInverse && value < threshold)
    {
      const uint16_t range = threshold > 0 ? threshold : 1;
      strength = ((uint32_t)(threshold - value) * 1000UL) / range;
    }

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


static int16_t slewFollowPwm(int16_t previous, int16_t target)
{
  if (target > previous + FOLLOW_PWM_SLEW_STEP)
    return previous + FOLLOW_PWM_SLEW_STEP;
  if (target < previous - FOLLOW_PWM_SLEW_STEP)
    return previous - FOLLOW_PWM_SLEW_STEP;
  return target;
}


static int16_t effectiveMotorPwm(int16_t pwm, uint8_t effectiveMinimum)
{
  return pwm > 0 && pwm < effectiveMinimum ? 0 : pwm;
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

  int16_t approachSpeed = min(baseSpeed, speedLimit);
  if (speedLimit == 255 &&
      outwardTrendCount >= TURN_OUTWARD_CONFIRM &&
      absoluteError(error) > TURN_SLOW_START_ERROR)
  {
    const uint16_t reduction =
        (absoluteError(error) - TURN_SLOW_START_ERROR) /
        TURN_APPROACH_REDUCTION_DIVISOR;
    approachSpeed -= min(reduction, (uint16_t)baseSpeed);
    if (approachSpeed < TURN_APPROACH_MIN_PWM)
      approachSpeed = TURN_APPROACH_MIN_PWM;
  }

  int16_t leftPwm = constrain(approachSpeed + correction, 0, 255);
  int16_t rightPwm = constrain(approachSpeed - correction, 0, 255);
  leftPwm = effectiveMotorPwm(leftPwm, LEFT_MOTOR_EFFECTIVE_MIN_PWM);
  rightPwm = effectiveMotorPwm(rightPwm, RIGHT_MOTOR_EFFECTIVE_MIN_PWM);
  if (followMixerValid)
  {
    leftPwm = slewFollowPwm(previousFollowLeft, leftPwm);
    rightPwm = slewFollowPwm(previousFollowRight, rightPwm);
  }
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
  if (millis() - lastInverseToggleAt < INVERSE_TOGGLE_COOLDOWN_MS)
  {
    return false;
  }

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


static bool updateInverseMode(uint16_t blackPattern)
{
  if (isInverseTransition(blackPattern))
  {
    if (++inverseConfirmCount >= INVERSE_CONFIRM_FRAMES)
    {
      lineIsInverse = !lineIsInverse;
      lastInverseToggleAt = millis();
      inverseConfirmCount = 0;

      // A polarity boundary looks wide in the old polarity. Cancel only that
      // temporary probe; an active junction selection remains locked.
      if (state == STATE_FORWARD_PROBE &&
          probeReason == PROBE_WIDE_FEATURE && !junctionActive)
      {
        state = STATE_FOLLOW;
        junctionAvailableRoutes = 0;
        wideAreaCrawl = false;
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
  state = STATE_FORWARD_PROBE;
  probeReason = reason;
  probeStartedAt = millis();
  lostReadCount = reason == PROBE_LOST_LINE ? 1 : 0;
  lostLineConfirmed = false;
  probeNarrowConfirmCount = 0;
  wideAreaCrawl = false;
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
  stateStartedAt = millis();
}


static void beginReverse()
{
  state = STATE_REVERSE_REACQUIRE;
  stateStartedAt = millis();
  centerStableCount = 0;
  resetPd();
}


static void beginRecoverySearch()
{
  stopMotors();
  state = STATE_RECOVERY_SEARCH;
  stateStartedAt = millis();
  centerStableCount = 0;
  searchFirstSide = sideFromRecentHistory();
  resetPd();
  resetTurnIntent();
}


static void beginMovingReacquire(uint16_t pattern)
{
  state = STATE_RECOVERY_SEARCH;
  stateStartedAt = millis();
  centerStableCount = centerSeesLine(pattern) ? 1 : 0;
  searchFirstSide = sideFromRecentHistory();
  resetPd();
  resetTurnIntent();
  applyPd(pattern, REACQUIRE_FORWARD_PWM);
}


static void safeStop()
{
  stopMotors();
  state = STATE_STOPPED;
}


static void clearJunctionAfterExit();


static void beginRouteCommit()
{
  state = STATE_ROUTE_COMMIT;
  commitStartedAt = millis();
  exitStableCount = 0;
  lostReadCount = 0;
  resetPd();
}


static void beginTurn(RouteDirection route)
{
  selectedRoute = route;
  state = STATE_TURN_TO_ROUTE;
  stateStartedAt = millis();
  centerStableCount = 0;
  turnSawCenterLost = false;
  resetPd();
  resetTurnIntent();
}


static void chooseNextRoute()
{
  const uint8_t untried = junctionAvailableRoutes & ~attemptedRoutes;

  for (uint8_t index = 0; index < 3; index++)
  {
    const RouteDirection route = settingsPriorityAt(index);
    if (untried & route)
    {
      selectedRoute = route;
      attemptedRoutes |= route;
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

  if (!uTurnAttempted)
  {
    uTurnAttempted = true;
    selectedRoute = ROUTE_U_TURN;
    beginTurn(ROUTE_U_TURN);
    return;
  }

  clearJunctionAfterExit();
  beginRecoverySearch();
}


static void startJunction(uint8_t availableRoutes)
{
  junctionActive = true;
  junctionAvailableRoutes = availableRoutes;
  attemptedRoutes = 0;
  uTurnAttempted = false;
  resetJunctionEvidence();
  chooseNextRoute();
}


static void clearJunctionAfterExit()
{
  junctionActive = false;
  junctionAvailableRoutes = 0;
  attemptedRoutes = 0;
  selectedRoute = ROUTE_NONE;
  uTurnAttempted = false;
  resetJunctionEvidence();
}


static void failSelectedRoute()
{
  // The selected route was marked attempted when chosen. Reverse first; only
  // after the old line/junction is reacquired may another route be selected.
  beginReverse();
}


static NavigationResult tickFollow(uint16_t blackPattern, uint16_t pattern)
{
  (void)blackPattern;

  const uint8_t count = activeCount(pattern);
  if (count == 0)
  {
    if (turnLossLikely())
    {
      // One sensor-frame coast before the pivot changes motor direction.
      stopMotors();
      beginTurn(lastReliableSide < 0 ? ROUTE_LEFT : ROUTE_RIGHT);
      return NAVIGATION_ACTIVE;
    }

    beginForwardProbe(PROBE_LOST_LINE);
    moveLFR(min(baseSpeed, FORWARD_PROBE_MAX_PWM),
            min(baseSpeed, FORWARD_PROBE_MAX_PWM));
    return NAVIGATION_ACTIVE;
  }

  if (count >= WIDE_FEATURE_MIN_ACTIVE)
  {
    // Ambiguous wide features start the monitored probe immediately, before
    // multi-frame classification, so confirmation cannot add hidden travel.
    resetJunctionEvidence();
    addRouteEvidence(routesInPattern(pattern) & ~ROUTE_STRAIGHT);
    beginForwardProbe(PROBE_WIDE_FEATURE);
    moveLFR(min(baseSpeed, WIDE_AREA_CRAWL_PWM),
            min(baseSpeed, WIDE_AREA_CRAWL_PWM));
    return NAVIGATION_ACTIVE;
  }

  // A single continuous narrow group is an ordinary line, even when a curve
  // touches both the centre and a side route region.
  if (isNarrowLine(pattern) && patternGroups(pattern) == 1)
  {
    resetJunctionEvidence();
    applyPd(pattern);
    return NAVIGATION_ACTIVE;
  }

  const uint8_t routes = routesInPattern(pattern);
  if (routeCount(routes) >= 2)
  {
    addRouteEvidence(routes);
    if (junctionFrameCount >= JUNCTION_CONFIRM_FRAMES)
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


static NavigationResult tickForwardProbe(uint16_t pattern)
{
  const uint32_t elapsed = millis() - probeStartedAt;
  const uint8_t count = activeCount(pattern);

  if (isNarrowLine(pattern))
  {
    if (probeReason == PROBE_WIDE_FEATURE)
    {
      // Preserve the established cross exit/route-lock behavior.
      stopMotors();
    }
    else
    {
      // A recovered ordinary line is confirmed while moving slowly.
      applyPd(pattern, REACQUIRE_FORWARD_PWM);
    }

    if (++probeNarrowConfirmCount < CENTER_STABLE_READS)
    {
      return NAVIGATION_ACTIVE;
    }

    if (probeReason == PROBE_WIDE_FEATURE)
    {
      junctionAvailableRoutes |= ROUTE_STRAIGHT;
      if (!junctionActive)
      {
        const uint8_t observed = confirmedRoutes();
        junctionAvailableRoutes |= observed;
        startJunction(junctionAvailableRoutes);
      }
      else
      {
        beginRouteCommit();
      }
    }
    else if (junctionActive)
    {
      state = STATE_ROUTE_COMMIT;
      resetPd();
    }
    else
    {
      state = STATE_FOLLOW;
      resetPd();
    }
    return NAVIGATION_ACTIVE;
  }

  if (probeNarrowConfirmCount != 0 &&
      probeReason == PROBE_LOST_LINE)
  {
    resetPd();
  }
  probeNarrowConfirmCount = 0;

  if (probeReason == PROBE_LOST_LINE)
  {
    if (count == 0 && lostReadCount < 255)
    {
      lostReadCount++;
      if (lostReadCount >= LOST_LINE_CONFIRM_READS)
      {
        lostLineConfirmed = true;
      }
    }
    else if (count != 0)
    {
      lostReadCount = 0;
    }
  }
  else
  {
    // Vote during the short cross window and while observing a real exit, but
    // do not keep manufacturing side votes from a persistent filled area.
    if (!wideAreaCrawl || count < WIDE_FEATURE_MIN_ACTIVE)
    {
      addRouteEvidence(routesInPattern(pattern) & ~ROUTE_STRAIGHT);
    }

    // Empty beyond the bar proves STRAIGHT is unavailable. Brake now and use
    // only side routes observed in several frames.
    if (count == 0 && junctionFrameCount >= JUNCTION_CONFIRM_FRAMES)
    {
      beginProbeBrake();
      return NAVIGATION_ACTIVE;
    }

    if (elapsed >= FORWARD_PROBE_WINDOW_MS &&
        count >= WIDE_FEATURE_MIN_ACTIVE)
    {
      if (!wideAreaCrawl)
      {
        wideAreaCrawl = true;
      }
      moveLFR(WIDE_AREA_CRAWL_PWM, WIDE_AREA_CRAWL_PWM);
      return NAVIGATION_ACTIVE;
    }
  }

  if (elapsed >= FORWARD_PROBE_WINDOW_MS)
  {
    if (probeReason == PROBE_LOST_LINE && !lostLineConfirmed)
    {
      // Loss never passed the consecutive-read filter. Keep the existing
      // forward command and rescan without inserting a stop pulse.
      state = STATE_FOLLOW;
      resetPd();
      return NAVIGATION_ACTIVE;
    }
    if (probeReason == PROBE_LOST_LINE)
    {
      // Coast for one frame, then reverse on the next sensor update.
      stopMotors();
      beginReverse();
      return NAVIGATION_ACTIVE;
    }
    beginProbeBrake();
    return NAVIGATION_ACTIVE;
  }

  // Continuously sampled, non-blocking forward motion.
  const uint8_t probePwm = probeReason == PROBE_WIDE_FEATURE ?
      min(baseSpeed, WIDE_AREA_CRAWL_PWM) :
      min(baseSpeed, FORWARD_PROBE_MAX_PWM);
  moveLFR(probePwm, probePwm);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickProbeBrake(uint16_t pattern)
{
  brakeMotors();
  if (millis() - stateStartedAt < PROBE_BRAKE_HOLD_MS)
  {
    return NAVIGATION_ACTIVE;
  }

  stopMotors();
  const uint8_t observed = confirmedRoutes();
  junctionAvailableRoutes |= observed & ~ROUTE_STRAIGHT;

  if (isNarrowLine(pattern))
  {
    junctionAvailableRoutes |= ROUTE_STRAIGHT;
  }

  if (activeCount(pattern) >= WIDE_FEATURE_MIN_ACTIVE)
  {
    // A filled track area is not a finish. Resume the same probe without
    // restarting its evidence window and crawl until a narrow/empty exit.
    state = STATE_FORWARD_PROBE;
    wideAreaCrawl = true;
    moveLFR(WIDE_AREA_CRAWL_PWM, WIDE_AREA_CRAWL_PWM);
    return NAVIGATION_ACTIVE;
  }

  if (junctionAvailableRoutes == 0)
  {
    beginReverse();
    return NAVIGATION_ACTIVE;
  }

  startJunction(junctionAvailableRoutes);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickTurn(uint16_t pattern)
{
  const uint32_t elapsed = millis() - stateStartedAt;
  const bool isUTurn = selectedRoute == ROUTE_U_TURN;
  const uint32_t timeout = isUTurn ? U_TURN_TIMEOUT_MS :
                                      TURN_SEARCH_TIMEOUT_MS;
  const uint32_t minimum = isUTurn ? U_TURN_MINIMUM_MS : TURN_MINIMUM_MS;

  int8_t direction;
  if (selectedRoute == ROUTE_LEFT)
  {
    direction = -1;
  }
  else if (selectedRoute == ROUTE_RIGHT)
  {
    direction = 1;
  }
  else
  {
    direction = sideFromRecentHistory();
  }

  const bool narrow = isNarrowLine(pattern);
  int16_t lineError = 0;
  if (narrow)
  {
    lineError = calculateLineError(calculatePdLinePosition(pattern));
  }

  if (!centerSeesLine(pattern))
  {
    turnSawCenterLost = true;
  }

  const bool nearCenter = narrow &&
      (centerSeesLine(pattern) ||
       absoluteError(lineError) <= TURN_CENTER_ERROR_LIMIT);
  if (turnSawCenterLost && nearCenter && elapsed >= minimum)
  {
    // End pivot torque immediately, then confirm with both wheels forward.
    applyPd(pattern, REACQUIRE_FORWARD_PWM);
    if (++centerStableCount >= TURN_CENTER_CONFIRM_READS)
    {
      beginRouteCommit();
    }
    return NAVIGATION_ACTIVE;
  }
  if (centerStableCount != 0)
  {
    centerStableCount = 0;
    resetPd();
  }

  if (elapsed >= timeout)
  {
    stopMotors();
    failSelectedRoute();
    return NAVIGATION_ACTIVE;
  }

  if (turnSawCenterLost && narrow)
  {
    const int16_t centerLimit = (int16_t)TURN_CENTER_ERROR_LIMIT;
    const bool crossedCenter =
        (direction < 0 && lineError > centerLimit) ||
        (direction > 0 && lineError < -centerLimit);
    if (crossedCenter)
    {
      moveLFR(-direction * TURN_CROSS_CORRECTION_PWM,
              direction * TURN_CROSS_CORRECTION_PWM);
      return NAVIGATION_ACTIVE;
    }

    const bool onExpectedSide =
        (direction < 0 && lineError < 0) ||
        (direction > 0 && lineError > 0);
    if (onExpectedSide)
    {
      moveLFR(direction * TURN_REACQUIRE_PWM,
              -direction * TURN_REACQUIRE_PWM);
      return NAVIGATION_ACTIVE;
    }
  }

  moveLFR(direction * SEARCH_TURN_PWM,
          -direction * SEARCH_TURN_PWM);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickRouteCommit(uint16_t pattern)
{
  if (pattern == 0)
  {
    if (turnLossLikely())
    {
      // Coast for this frame before the existing pivot changes direction.
      stopMotors();
      RouteDirection turnRoute = selectedRoute;
      if (turnRoute != ROUTE_LEFT && turnRoute != ROUTE_RIGHT)
      {
        turnRoute = lastReliableSide < 0 ? ROUTE_LEFT : ROUTE_RIGHT;
      }
      beginTurn(turnRoute);
      return NAVIGATION_ACTIVE;
    }

    beginForwardProbe(PROBE_LOST_LINE);
    moveLFR(min(baseSpeed, FORWARD_PROBE_MAX_PWM),
            min(baseSpeed, FORWARD_PROBE_MAX_PWM));
    return NAVIGATION_ACTIVE;
  }

  if (isNarrowLine(pattern) && centerSeesLine(pattern))
  {
    if (exitStableCount < 255) exitStableCount++;
  }
  else
  {
    exitStableCount = 0;
  }

  applyPd(pattern);

  if (millis() - commitStartedAt >= ROUTE_COMMIT_VALIDATION_MS &&
      exitStableCount >= JUNCTION_EXIT_STABLE_READS)
  {
    clearJunctionAfterExit();
    state = STATE_FOLLOW;
    resetPd();
  }

  return NAVIGATION_ACTIVE;
}


static NavigationResult tickReverse(uint16_t pattern)
{
  const bool reacquired = pattern != 0 &&
      (isNarrowLine(pattern) ||
       activeCount(pattern) >= WIDE_FEATURE_MIN_ACTIVE ||
       routeCount(routesInPattern(pattern)) >= 2);

  if (reacquired)
  {
    if (junctionActive)
    {
      // A committed junction may need a pivot/reverse direction change.
      stopMotors();
      chooseNextRoute();
    }
    else if (isNarrowLine(pattern))
    {
      // Replace reverse torque immediately with low forward tracking.
      beginMovingReacquire(pattern);
    }
    else
    {
      beginRecoverySearch();
    }
    return NAVIGATION_ACTIVE;
  }

  if (millis() - stateStartedAt >= RECOVERY_REVERSE_TIMEOUT_MS)
  {
    beginRecoverySearch();
    return NAVIGATION_ACTIVE;
  }

  moveLFR(-RECOVERY_REVERSE_PWM, -RECOVERY_REVERSE_PWM);
  return NAVIGATION_ACTIVE;
}


static NavigationResult tickRecoverySearch(uint16_t pattern)
{
  const uint32_t elapsed = millis() - stateStartedAt;

  if (isNarrowLine(pattern))
  {
    applyPd(pattern, REACQUIRE_FORWARD_PWM);
    if (centerSeesLine(pattern))
    {
      if (++centerStableCount >= TURN_CENTER_CONFIRM_READS)
      {
        if (junctionActive)
        {
          stopMotors();
          chooseNextRoute();
        }
        else
        {
          state = STATE_FOLLOW;
          resetPd();
        }
      }
    }
    else centerStableCount = 0;
    return NAVIGATION_ACTIVE;
  }

  centerStableCount = 0;
  resetPd();
  if (elapsed >= TURN_SEARCH_TIMEOUT_MS)
  {
    beginReverse();
    return NAVIGATION_ACTIVE;
  }

  // Sweep first toward the last reliable/history side, then alternate in
  // bounded segments. Exact 30/45/60/90 degree guesses are never used.
  const uint8_t segment = elapsed / SEARCH_SWEEP_MS;
  const int8_t direction = (segment & 1) ? -searchFirstSide : searchFirstSide;
  moveLFR(direction * SEARCH_TURN_PWM,
          -direction * SEARCH_TURN_PWM);
  return NAVIGATION_ACTIVE;
}


void navigationStart()
{
  state = STATE_FOLLOW;
  stateStartedAt = millis();
  lineIsInverse = false;
  lastInverseToggleAt = 0;
  lostReadCount = 0;
  centerStableCount = 0;
  inverseConfirmCount = 0;
  lastReliableSide = 1;
  junctionActive = false;
  junctionAvailableRoutes = 0;
  attemptedRoutes = 0;
  selectedRoute = ROUTE_NONE;
  uTurnAttempted = false;
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
  safeStop();
  junctionActive = false;
  attemptedRoutes = 0;
}


NavigationResult navigationTick(uint16_t sensorValues[])
{
  if (state == STATE_STOPPED)
  {
    stopMotors();
    return NAVIGATION_ACTIVE;
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

  const bool polarityState = state == STATE_FOLLOW ||
      state == STATE_FORWARD_PROBE || state == STATE_ROUTE_COMMIT;
  if (polarityState) updateInverseMode(blackPattern);
  else inverseConfirmCount = 0;

  const uint16_t linePattern = lineIsInverse ?
      (ALL_SENSOR_MASK ^ blackPattern) : blackPattern;

  NavigationResult result = NAVIGATION_ACTIVE;
  switch (state)
  {
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
    case STATE_REVERSE_REACQUIRE:
      result = tickReverse(linePattern);
      break;
    case STATE_RECOVERY_SEARCH:
      result = tickRecoverySearch(linePattern);
      break;
    default:
      beginRecoverySearch();
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
