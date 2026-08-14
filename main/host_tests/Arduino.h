#pragma once

#include <cstdint>

using std::int8_t;
using std::int16_t;
using std::int32_t;
using std::uint8_t;
using std::uint16_t;
using std::uint32_t;

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21

uint32_t millis();
uint32_t micros();

template <typename T>
T min(T left, T right)
{
  return left < right ? left : right;
}

#define constrain(value, low, high) \
    ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
