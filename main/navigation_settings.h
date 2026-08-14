#pragma once

#include <Arduino.h>


// ================================================================
// FORWARD-PROBE TIMING
// ================================================================

// Fixed RUN control period. One fresh 14-sensor frame is acquired per tick.
constexpr uint16_t CONTROL_PERIOD_US = 3500;

// Limits a one-frame derivative kick without adding a lagging history filter.
constexpr int16_t DERIVATIVE_ERROR_DELTA_LIMIT = 300;

// Default: 28 ms. Short continuously sampled look-ahead window for a gap or
// wide mark. It is a tunable time window, not an exact distance measurement.
constexpr uint16_t FORWARD_PROBE_WINDOW_MS = 28;

// Default: 200 PWM. Caps normal look-ahead speed without changing baseSpeed.
constexpr uint8_t FORWARD_PROBE_MAX_PWM = 200;

// Positive FOLLOW commands below these values do not move the motors reliably
// through the L298N, so the mixer coasts that wheel instead of letting it stall.
constexpr uint8_t LEFT_MOTOR_EFFECTIVE_MIN_PWM = 90;
constexpr uint8_t RIGHT_MOTOR_EFFECTIVE_MIN_PWM = 90;

// Maximum FOLLOW-output change per sensor frame. Increase if steering feels
// delayed; decrease if analog noise still makes the drive visibly abrupt.
constexpr uint8_t FOLLOW_PWM_SLEW_STEP = 36;

// Default: 100 PWM. Persistent wide areas continue at this sampled crawl speed.
// Increase if the motors stall; decrease if a filled area is crossed too fast.
constexpr uint8_t WIDE_AREA_CRAWL_PWM = 100;

// Default: 8 ms. Active-brake hold after a probe expires. Increase if momentum
// remains; decrease if the drive train jerks or the bridge runs hot.
constexpr uint8_t PROBE_BRAKE_HOLD_MS = 8;


// ================================================================
// SENSOR-PATTERN FILTERS
// ================================================================

// Default: 3 reads. Empty frames required to confirm loss. Increase for noisy
// sensors/dots; decrease if recovery starts too late at sharp corners.
constexpr uint8_t LOST_LINE_CONFIRM_READS = 3;

// Default: 5 sensors. Largest active count treated as a narrow usable line.
// Increase for a physically wider line; decrease if branches look narrow.
constexpr uint8_t NARROW_LINE_MAX_ACTIVE = 5;

// Default: 6 sensor spaces. Largest first-to-last span of a narrow line.
// Increase for angled/thick lines; decrease to reject separated branches.
constexpr uint8_t NARROW_LINE_MAX_SPAN = 6;

// Default: 8 sensors. A pattern this broad starts the shared probe immediately.
// Increase if wide bends trigger it; decrease if crosses are detected late.
constexpr uint8_t WIDE_FEATURE_MIN_ACTIVE = 8;

// Default: 2 sensors. Minimum adjacent sensors proving a side branch. Increase
// to reject noise; decrease if a thin angled branch is missed.
constexpr uint8_t SIDE_CLUSTER_MIN_ACTIVE = 2;

// Default: 3 frames. Candidate junction frames required. Increase to reject
// brief bends; decrease when short junctions pass before being classified.
constexpr uint8_t JUNCTION_CONFIRM_FRAMES = 3;

// Default: 2 votes. Route must appear this many times in the frame window.
// Increase for stricter detection; decrease for very short angled branches.
constexpr uint8_t ROUTE_MIN_VOTES = 2;

// Default: 3 reads. S6/S7 must stably see the outgoing line. Increase for
// noisy centres; decrease if turns stop reacting too slowly.
constexpr uint8_t CENTER_STABLE_READS = 3;

// Default: 4 reads. Stable narrow-line reads needed to exit a junction. Increase
// if attempted routes clear too early; decrease for very short exit lanes.
constexpr uint8_t JUNCTION_EXIT_STABLE_READS = 4;

// Default: 150 position units. S6/S7 and a narrow group immediately beside
// them count as centred turn reacquisition.
constexpr uint16_t TURN_CENTER_ERROR_LIMIT = 150;

// Default: 2 moving reads. The pivot ends on the first centred frame and the
// outgoing line is confirmed at low forward speed.
constexpr uint8_t TURN_CENTER_CONFIRM_READS = 2;

// Three same-side errors with increasing magnitude establish outward motion.
constexpr uint16_t TURN_TREND_MIN_STEP = 40;
constexpr uint8_t TURN_OUTWARD_CONFIRM = 2;

// A line lost beyond this error enters turn recovery immediately.
constexpr uint16_t TURN_LOSS_ERROR_LIMIT = 350;

// Slow only a consistently outward-moving approach near the outer sensors.
// The 120 PWM floor should remain above the measured motor dead zone.
constexpr uint16_t TURN_SLOW_START_ERROR = 250;
constexpr uint8_t TURN_APPROACH_MIN_PWM = 120;
constexpr uint8_t TURN_APPROACH_REDUCTION_DIVISOR = 4;


// ================================================================
// RECOVERY, TURN, AND VALIDATION MOTION
// ================================================================

// Default: 110 PWM. Straight reverse speed during reacquisition. Increase if
// reverse is weak; decrease if it overshoots the previous line/junction.
constexpr uint8_t RECOVERY_REVERSE_PWM = 110;

// Default: 115 PWM. Low forward confirmation speed. Increase if either motor
// stalls; decrease if reacquisition overshoots the line.
constexpr uint8_t REACQUIRE_FORWARD_PWM = 115;

// Default: 500 ms. Maximum reverse reacquisition time. Increase for high
// momentum; decrease when limited space exists behind the robot.
constexpr uint16_t RECOVERY_REVERSE_TIMEOUT_MS = 500;

// Default: 120 PWM. Pivot/sweep speed. Increase if motors stall; decrease if
// S6/S7 are crossed too quickly.
constexpr uint8_t SEARCH_TURN_PWM = 120;

// Default: 100 PWM. Reduced pivot after the outgoing group becomes credible.
// Increase if the pivot stalls; decrease if it crosses centre too quickly.
constexpr uint8_t TURN_REACQUIRE_PWM = 100;

// Default: 90 PWM. Opposite correction after the line crosses centre. Increase
// if correction stalls; decrease if it oscillates across the centre.
constexpr uint8_t TURN_CROSS_CORRECTION_PWM = 90;

// Default: 300 ms. Time before a recovery sweep reverses direction. Increase
// for broad turns; decrease to scan the opposite side sooner.
constexpr uint16_t SEARCH_SWEEP_MS = 300;

// Default: 1400 ms. Maximum normal route/recovery pivot time. Increase for slow
// motors; decrease to reach safe stop sooner after a mechanical fault.
constexpr uint16_t TURN_SEARCH_TIMEOUT_MS = 1400;

// Default: 35 ms. Prevents the old centre line ending a branch turn instantly.
// Increase if turns stop on the incoming line; decrease for gentle branches.
constexpr uint8_t TURN_MINIMUM_MS = 35;

// Default: 250 ms. Minimum bounded U-turn pivot before centre acceptance.
// Increase if the robot accepts a half turn; decrease on a fast chassis.
constexpr uint16_t U_TURN_MINIMUM_MS = 250;

// Default: 1800 ms. Maximum bounded U-turn. Increase for a slow chassis;
// decrease when spin space is restricted.
constexpr uint16_t U_TURN_TIMEOUT_MS = 1800;

// Default: 300 ms. Selected route must remain valid this long. Increase to
// validate farther beyond a junction; decrease for closely spaced junctions.
constexpr uint16_t ROUTE_COMMIT_VALIDATION_MS = 300;

// ================================================================
// INVERSE-SECTION AND HISTORY FILTERS
// ================================================================

// Default: 2 frames. Polarity-transition frames required before inversion.
// Increase if wide marks toggle polarity; decrease if a short marker is missed.
constexpr uint8_t INVERSE_CONFIRM_FRAMES = 2;

// Default: 500 ms. Minimum time between polarity changes. Increase if an
// inverse section chatters; decrease only for exceptionally short sections.
constexpr uint16_t INVERSE_TOGGLE_COOLDOWN_MS = 500;
