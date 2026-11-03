// flight_log.h - RAM ring-buffer flight log, saved to the built-in SD card
//
// SD cards can stall for 100+ ms during a write, which would freeze the
// control loop. So in flight everything goes into RAM (DMAMEM) and is
// written to SD only after landing. Files: FLTnnn.CSV (data) and
// FLTnnn.TXT (event messages).
#pragma once
#include <SD.h>
#include "config.h"
#include "flight_types.h"

struct LogRecord {
  uint32_t tMs;
  uint8_t state;
  uint8_t flags;
  uint16_t baroRejects;
  float alt, vel, baroAlt, accUp, accBodyZ;
  float tiltX, tiltY, rateX, rateY, rateZ;
  float gimbalX, gimbalY, railV, predTouchdown;
};

#define LOG_CAPACITY (LOG_HZ * LOG_SECONDS)
#define EVENT_BUF_SIZE 6144

DMAMEM static LogRecord g_logBuf[LOG_CAPACITY];
DMAMEM static char g_eventBuf[EVENT_BUF_SIZE];

class FlightLog {
public:
  bool recording = false;
  bool saved = false;
  uint32_t count = 0;    // records written (may exceed capacity: ring)
  uint32_t eventLen = 0;
  char lastFile[16] = "";

  void clear() {
    count = 0;
    eventLen = 0;
    g_eventBuf[0] = 0;
    saved = false;
  }

  void add(const LogRecord& r) {
    if (!recording) return;
    g_logBuf[count % LOG_CAPACITY] = r;
    count++;
  }

  void addEvent(const char* line) {
    uint32_t n = strlen(line);
    if (eventLen + n + 2 >= EVENT_BUF_SIZE) return;
    memcpy(g_eventBuf + eventLen, line, n);
    eventLen += n;
    g_eventBuf[eventLen++] = '\n';
    g_eventBuf[eventLen] = 0;
  }

  // returns true on success. Takes a second or two - only call on the ground.
  bool saveToSD() {
    if (!SD.begin(BUILTIN_SDCARD)) return false;
    char csvName[16], txtName[16];
    int idx = 0;
    for (idx = 0; idx < 1000; idx++) {
      snprintf(csvName, sizeof(csvName), "FLT%03d.CSV", idx);
      if (!SD.exists(csvName)) break;
    }
    snprintf(txtName, sizeof(txtName), "FLT%03d.TXT", idx);
    File f = SD.open(csvName, FILE_WRITE);
    if (!f) return false;
    f.println("t_ms,state,flags,baro_rejects,alt_m,vel_mps,baro_alt_m,acc_up_mps2,acc_body_z_g,"
              "tilt_x_deg,tilt_y_deg,rate_x_dps,rate_y_dps,rate_z_dps,gimbal_x_deg,gimbal_y_deg,"
              "rail_v,pred_touchdown_mps");
    uint32_t n = count < LOG_CAPACITY ? count : LOG_CAPACITY;
    uint32_t start = count < LOG_CAPACITY ? 0 : count % LOG_CAPACITY;
    char line[256];
    for (uint32_t i = 0; i < n; i++) {
      const LogRecord& r = g_logBuf[(start + i) % LOG_CAPACITY];
      int len = snprintf(line, sizeof(line),
                         "%lu,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f\n",
                         (unsigned long)r.tMs, r.state, r.flags, r.baroRejects, r.alt, r.vel, r.baroAlt,
                         r.accUp, r.accBodyZ, r.tiltX, r.tiltY, r.rateX, r.rateY, r.rateZ, r.gimbalX,
                         r.gimbalY, r.railV, r.predTouchdown);
      f.write((const uint8_t*)line, len);
    }
    f.close();
    File e = SD.open(txtName, FILE_WRITE);
    if (e) {
      e.write((const uint8_t*)g_eventBuf, eventLen);
      e.close();
    }
    strncpy(lastFile, csvName, sizeof(lastFile));
    saved = true;
    return true;
  }
};
