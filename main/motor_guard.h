#pragma once

#include <Arduino.h>


// Shorter than one RUN control period. Increase only if the driver or gearbox
// still jerks on a true direction reversal.
constexpr uint16_t MOTOR_DIRECTION_GUARD_US = 2500;


struct MotorDirectionGuardState
{
  int8_t lastSign;
  bool outputIsZero;
  uint32_t zeroStartedAt;
};


inline void motorDirectionGuardReset(MotorDirectionGuardState &state)
{
  state.lastSign = 0;
  state.outputIsZero = true;
  state.zeroStartedAt = 0;
}


inline int16_t motorDirectionGuardCommand(MotorDirectionGuardState &state,
                                          int16_t target,
                                          uint32_t now)
{
  target = constrain(target, -255, 255);
  if (target == 0)
  {
    if (!state.outputIsZero) state.zeroStartedAt = now;
    state.outputIsZero = true;
    return 0;
  }

  const int8_t targetSign = target < 0 ? -1 : 1;
  if (state.lastSign != 0 && targetSign != state.lastSign)
  {
    if (!state.outputIsZero)
    {
      state.outputIsZero = true;
      state.zeroStartedAt = now;
      return 0;
    }
    if ((uint32_t)(now - state.zeroStartedAt) < MOTOR_DIRECTION_GUARD_US)
      return 0;
  }

  state.lastSign = targetSign;
  state.outputIsZero = false;
  return target;
}
