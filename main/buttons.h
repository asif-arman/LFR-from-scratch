#pragma once

#include <Arduino.h>


// ": uint8_t" forces the enum to use only one byte.
enum ButtonEvent : uint8_t
{
  BUTTON_NONE,

  BUTTON_UP_CLICK,
  BUTTON_DOWN_CLICK,
  BUTTON_ACTION_CLICK,
  BUTTON_ACTION_LONG_PRESS
};


void buttonsInit();

ButtonEvent readButtonEvent();
