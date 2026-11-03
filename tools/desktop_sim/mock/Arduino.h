// Minimal desktop mock of the Arduino / Teensy API for testing
#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <string>
using std::fminf; using std::fmaxf;
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define F(x) x
#define DMAMEM
uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
extern int g_pinState[64];
extern int g_pwm[64];
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { if (p >= 0 && p < 64) g_pinState[p] = v; }
inline void analogWrite(int p, int v) { if (p >= 0 && p < 64) g_pwm[p] = v; }
inline int analogRead(int) { return 0; }
inline void analogWriteFrequency(int, float) {}
inline void analogWriteResolution(int) {}
inline void analogReadResolution(int) {}

class Print {
public:
  virtual size_t write(const uint8_t* b, size_t n) = 0;
  size_t print(const char* s) { return write((const uint8_t*)s, strlen(s)); }
  size_t println(const char* s) { size_t n = print(s); return n + print("\n"); }
  size_t println() { return print("\n"); }
  int printf(const char* fmt, ...) {
    char buf[1024]; va_list ap; va_start(ap, fmt); int n = vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    print(buf); return n;
  }
  virtual ~Print() {}
};
class Stream : public Print {
public:
  std::deque<char> in;
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return c; }
  void inject(const char* s) { while (*s) in.push_back(*s++); }
  int availableForWrite() { return 4096; }
  explicit operator bool() const { return true; }
};
class usb_serial_class : public Stream {
public:
  bool quiet = false;
  FILE* out = stdout;
  void begin(long) {}
  size_t write(const uint8_t* b, size_t n) override { if (!quiet) fwrite(b, 1, n, out); return n; }
};
class HardwareSerial : public Stream {
public:
  std::string sent;
  void begin(long) {}
  void addMemoryForWrite(void*, size_t) {}
  size_t write(const uint8_t* b, size_t n) override { sent.append((const char*)b, n); if (sent.size() > 100000) sent.erase(0, 50000); return n; }
};
extern usb_serial_class Serial;
extern HardwareSerial Serial2;
