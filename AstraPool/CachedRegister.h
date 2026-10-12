#pragma once
#include <Arduino.h>

struct CachedRegister {
  uint16_t addr;
  uint8_t count;
  uint16_t words[2];
  bool valid;
  uint32_t updated;
  uint8_t code;
};
