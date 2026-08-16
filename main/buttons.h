#pragma once

#include <Arduino.h>


// ": uint8_t" forces the enum to use only one byte.
enum ButtonEvent : uint8_t
{
  BUTTON_NONE,

  BUTTON_UP_CLICK,
  BUTTON_DOWN_CLICK,

  BUTTON_LEFT_DOUBLE_CLICK,
  BUTTON_RIGHT_CLICK
};


void buttonsInit();

ButtonEvent readButtonEvent();
