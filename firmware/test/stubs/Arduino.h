// Minimal Arduino API stub so the firmware compiles on a host for CI (no hardware access).
#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <string>
#include <cstdarg>
#define OUTPUT 1
#define INPUT 0
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline void delay(unsigned long) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int analogRead(int) { return 2048; }
template <class T, class L, class H> inline T constrain(T x, L lo, H hi) { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }
class String {
 public:
  std::string s;
  String() {}
  String(const char* c) : s(c) {}
  String& operator+=(char c) { s += c; return *this; }
  bool operator==(const char* c) const { return s == c; }
  void trim() { size_t a = s.find_first_not_of(" \r\t"); size_t b = s.find_last_not_of(" \r\t");
                s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1); }
  const char* c_str() const { return s.c_str(); }
};
struct SerialStub {
  void begin(long) {}
  int available() { return 0; }
  int read() { return -1; }
  void println(const char* m) { std::puts(m); }
  int printf(const char* f, ...) { va_list ap; va_start(ap, f); int n = std::vprintf(f, ap); va_end(ap); return n; }
};
inline SerialStub Serial;
