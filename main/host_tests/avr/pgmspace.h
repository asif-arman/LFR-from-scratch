#pragma once

#include <cstdint>


#define PROGMEM
#define pgm_read_byte(address) (*reinterpret_cast<const std::uint8_t *>(address))
