#pragma once
#include <cstdint>
struct WireStub {
  void begin(int, int) {}
  void setClock(long) {}
  void beginTransmission(uint8_t) {}
  void write(uint8_t) {}
  uint8_t endTransmission(bool = true) { return 0; }
  uint8_t requestFrom(int, int) { return 14; }
  int read() { return 0; }
};
inline WireStub Wire;
