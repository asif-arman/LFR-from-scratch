#pragma once

#include <cstdint>
#include <cstring>


class EEPROMClass
{
public:
  template <typename T>
  void get(int address, T &value) const
  {
    std::memcpy(&value, bytes + address, sizeof(T));
  }

  void update(int address, std::uint8_t value)
  {
    bytes[address] = value;
  }

  void reset(std::uint8_t value = 0xFF)
  {
    std::memset(bytes, value, sizeof(bytes));
  }

private:
  std::uint8_t bytes[1024] = {};
};


inline EEPROMClass EEPROM;
