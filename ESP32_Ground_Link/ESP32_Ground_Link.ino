// =====================================================================
//  ESP32_Ground_Link.ino - Wi-Fi ground station bridge for the Teensy
// =====================================================================
//  Board: classic ESP32 dev board (ESP32-WROOM-32), Arduino-ESP32 core.
//  No extra libraries needed.
//
//  Wiring (3.3 V logic on both boards, no level shifter needed):
//    ESP32 GPIO16 (RX) <- Teensy pin 8 (TX2)
//    ESP32 GPIO17 (TX) -> Teensy pin 7 (RX2)
//    ESP32 GND         -- Teensy GND
//    ESP32 5V/VIN      <- 5 V BEC (NOT the Teensy 3.3 V pin - Wi-Fi spikes
//                         draw several hundred mA)
//
//  Use: connect your phone/laptop to Wi-Fi "TVC-ROCKET", open
//  http://192.168.4.1 . While the page is open it sends a heartbeat; if a
//  countdown was started from this page and the heartbeat stops (phone
//  disconnects, walks out of range) the rocket aborts the countdown.
// =====================================================================
#include <WiFi.h>
#include <WebServer.h>

const char* AP_SSID = "TVC-ROCKET";
const char* AP_PASS = "launchsafe";  // 8+ characters - change it
#define TEENSY_BAUD 460800
#define LINK_RX_PIN 16
#define LINK_TX_PIN 17
HardwareSerial& teensy = Serial1;  // UART to the flight computer

WebServer server(80);

String lastTelem = "";
#define MSG_KEEP 30
String msgText[MSG_KEEP];
uint32_t msgSeqs[MSG_KEEP];
uint32_t msgSeq = 0;
String lineBuf;

const char PAGE[] = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TVC Ground</title><style>
body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:12px}
h1{font-size:18px;margin:0 0 8px}#st{font-size:28px;font-weight:700;margin:6px 0}
.g{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.c{background:#222;border-radius:8px;padding:8px}.c b{display:block;font-size:20px}
.c span{font-size:12px;color:#aaa}
button{font-size:16px;padding:12px;border:0;border-radius:8px;margin:4px 0;width:100%;background:#335;color:#fff}
#abort{background:#c22;font-size:22px;font-weight:700}#launch{background:#a60}
.b{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px}
#log{background:#000;font-family:monospace;font-size:12px;height:220px;overflow:auto;padding:6px;border-radius:8px;margin-top:8px;white-space:pre-wrap}
#link{font-size:12px}</style></head><body>
<h1>TVC Rocket Ground Station <span id="link">...</span></h1>
<div id="st">-</div>
<div class="g">
<div class="c"><span>altitude m</span><b id="alt">-</b></div>
<div class="c"><span>velocity m/s</span><b id="vel">-</b></div>
<div class="c"><span>tilt deg</span><b id="tilt">-</b></div>
<div class="c"><span>gimbal X</span><b id="gx">-</b></div>
<div class="c"><span>gimbal Y</span><b id="gy">-</b></div>
<div class="c"><span>battery V</span><b id="bat">-</b></div>
</div>
<div id="flags" style="font-size:13px;margin-top:6px"></div>
<button id="abort" onclick="cmd('ABORT')">ABORT</button>
<div class="b">
<button onclick="cmd('STATUS')">STATUS</button>
<button onclick="cmd('ARM')">ARM</button>
<button onclick="cmd('DISARM')">DISARM</button>
<button id="launch" onclick="launch()">LAUNCH</button>
<button onclick="cmd('TVCTEST')">TVC TEST</button>
<button onclick="cmd('STOP')">STOP TEST</button>
<button onclick="cmd('SERVOTEST')">SERVO SWEEP</button>
<button onclick="cmd('SAVE')">SAVE LOG</button>
</div>
<div id="log"></div>
<script>
let since=0,fails=0;
function $(i){return document.getElementById(i)}
function cmd(c){fetch('/cmd?c='+c).then(r=>r.text()).then(t=>add('> '+t)).catch(()=>add('! command failed'))}
function launch(){if(confirm('Everyone 30+ ft away, area clear, sky clear?\nStart countdown?'))cmd('LAUNCH')}
function add(t){const l=$('log');l.textContent+=t+'\n';l.scrollTop=l.scrollHeight}
function poll(){fetch('/data?since='+since).then(r=>r.json()).then(j=>{
 fails=0;$('link').textContent='link OK';
 if(j.t){const f=j.t.split(',');// T,ms,state,alt,vel,tilt,tx,ty,gx,gy,batt,pred,flags
  $('st').textContent=f[2];$('alt').textContent=f[3];$('vel').textContent=f[4];$('tilt').textContent=f[5];
  $('gx').textContent=f[8];$('gy').textContent=f[9];$('bat').textContent=f[10];
  const fl=+f[12];$('flags').textContent=(fl&1?'PYROS ARMED  ':'safe  ')+(fl&16?'asc-ign OK  ':'asc-ign OPEN  ')+(fl&32?'land-ign OK  ':'land-ign OPEN  ')+(fl&2?'LEGS OUT':'');}
 for(const m of j.m){add(m);}since=j.seq;
}).catch(()=>{fails++;$('link').textContent='LINK LOST';}).finally(()=>setTimeout(poll,250))}
poll();
</script></body></html>)HTML";

String jsonEscape(const String& s) {
  String o;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') o += '\\';
    if (c >= 32) o += c;
  }
  return o;
}

void handleRoot() { server.send(200, "text/html", PAGE); }

void handleData() {
  teensy.println("HB");  // heartbeat: a ground client is connected and polling
  uint32_t since = server.hasArg("since") ? strtoul(server.arg("since").c_str(), 0, 10) : 0;
  String j = "{\"t\":\"" + jsonEscape(lastTelem) + "\",\"seq\":" + String(msgSeq) + ",\"m\":[";
  bool first = true;
  for (uint32_t s = (msgSeq > MSG_KEEP ? msgSeq - MSG_KEEP : 0) + 1; s <= msgSeq; s++) {
    if (s <= since) continue;
    int i = s % MSG_KEEP;
    if (msgSeqs[i] != s) continue;
    if (!first) j += ",";
    j += "\"" + jsonEscape(msgText[i]) + "\"";
    first = false;
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleCmd() {
  static const char* allowed[] = {"STATUS", "ARM",    "DISARM", "LAUNCH", "ABORT", "TVCTEST",
                                  "STOP",   "SERVOTEST", "SAVE", "RESET",  "LEGS",  "LEGLOCK"};
  String c = server.arg("c");
  c.toUpperCase();
  for (unsigned i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++) {
    if (c == allowed[i]) {
      teensy.println(c);
      server.send(200, "text/plain", "sent " + c);
      return;
    }
  }
  server.send(400, "text/plain", "unknown command");
}

void handleLine(const String& line) {
  if (line.startsWith("T,")) {
    lastTelem = line;
  } else if (line.startsWith("M,")) {
    msgSeq++;
    msgText[msgSeq % MSG_KEEP] = line.substring(2);
    msgSeqs[msgSeq % MSG_KEEP] = msgSeq;
  }
}

void setup() {
  Serial.begin(115200);
  teensy.setRxBufferSize(4096);
  teensy.begin(TEENSY_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCmd);
  server.begin();
  Serial.print("Ground link up. Join Wi-Fi ");
  Serial.print(AP_SSID);
  Serial.print(" and open http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();
  while (teensy.available()) {
    char ch = (char)teensy.read();
    if (ch == '\n') {
      handleLine(lineBuf);
      lineBuf = "";
    } else if (ch != '\r') {
      lineBuf += ch;
      if (lineBuf.length() > 240) lineBuf = "";
    }
  }
}
