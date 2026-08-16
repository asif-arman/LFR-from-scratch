#pragma once

#include <cstdio>
#include <cstring>

#include "Arduino.h"


#define U8X8_PIN_NONE 0
static const uint8_t u8x8_font_chroma48medium8_r[] = { 0 };


namespace HostOled
{
inline char rows[8][17] = {};
inline uint8_t column = 0;
inline uint8_t row = 0;


inline void clear()
{
  for (uint8_t y = 0; y < 8; y++)
  {
    for (uint8_t x = 0; x < 16; x++) rows[y][x] = ' ';
    rows[y][16] = '\0';
  }
  column = 0;
  row = 0;
}


inline void write(const char *text)
{
  while (*text != '\0' && row < 8 && column < 16)
    rows[row][column++] = *text++;
}
}


class U8X8_SH1106_128X64_NONAME_HW_I2C
{
public:
  explicit U8X8_SH1106_128X64_NONAME_HW_I2C(uint8_t) {}

  void setI2CAddress(uint8_t) {}
  void setBusClock(uint32_t) {}
  void begin() { HostOled::clear(); }
  void setFont(const uint8_t *) {}
  void clearDisplay() { HostOled::clear(); }

  void setCursor(uint8_t column, uint8_t row)
  {
    HostOled::column = column;
    HostOled::row = row;
  }

  void print(const char *text) { HostOled::write(text); }

  void print(char character)
  {
    char text[2] = { character, '\0' };
    HostOled::write(text);
  }

  template <typename T>
  void print(T value)
  {
    char text[12];
    std::snprintf(text, sizeof(text), "%ld", static_cast<long>(value));
    HostOled::write(text);
  }
};
