#pragma once
#include "Arduino.h"
#define BUILTIN_SDCARD 254
#define FILE_WRITE 1
class File : public Print {
public:
  FILE* f = nullptr;
  explicit operator bool() const { return f != nullptr; }
  size_t write(const uint8_t* b, size_t n) override { return f ? fwrite(b, 1, n, f) : 0; }
  void close() { if (f) fclose(f); f = nullptr; }
};
class SDClass {
public:
  bool begin(int) { return true; }
  bool exists(const char* n) { std::string p = std::string("sdcard/") + n; FILE* f = fopen(p.c_str(), "r"); if (f) { fclose(f); return true; } return false; }
  File open(const char* n, int) { File x; std::string p = std::string("sdcard/") + n; x.f = fopen(p.c_str(), "w"); return x; }
};
extern SDClass SD;
