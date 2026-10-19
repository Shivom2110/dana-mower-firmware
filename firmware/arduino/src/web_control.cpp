// Wi-Fi web-control env (giga_r1_m7_web). The SmartMow website in /website
// commands the drive inverters over HTTP; CAN is the same Tm4Can driver the
// full controller and bench tool use.
//
// Like the bench tool, this env ignores the handles, ignition, deck switch,
// and E-stop input. The physical E-stop must still cut inverter power. The
// deck (blade) inverters are never run from the website.
//
// HTTP API (all GET, CORS open, JSON replies):
//   /status            telemetry for every configured inverter
//   /arm               request Power Ready (contactor closed), motors off
//   /drive?l=<n>&r=<n> wheel commands -1..+1 (× TM4_DRIVE_MAX_RPM); must be
//                      repeated within WEB_COMMAND_TIMEOUT_MS or the wheels stop
//   /stop              ramp wheels to 0 and stop gracefully, stay armed
//   /disarm            motors off immediately, request return to Startup
//
// Timing: the Wi-Fi stack and socket calls can stall loop() for hundreds of
// ms, which made the inverter drop SYNC/RPDOs (Fault 74, CO Synchro Lost).
// CAN servicing and the drive control (ramp + dead-man timeout) therefore
// run in their own realtime thread every 1 ms. loop() only handles HTTP and
// serial; it touches shared state under busLock and never does network or
// serial I/O while holding it.
//
// RPDOs stream from boot (power request = Startup) like the full controller.
// Wi-Fi is brought up before CAN because WiFi.begin*() blocks for seconds.
#include <Arduino.h>
#include <WiFi.h>
#include <mbed.h>
#include <stdlib.h>
#include <string.h>

#include "slew_rate_limiter.h"
#include "tm4_can.h"
#include "types.h"
#include "web_config.h"

namespace {
Tm4Can tm4Bus;
WiFiServer server(WEB_HTTP_PORT);
WiFiClient client;
bool wifiUp = false;

rtos::Thread canThread(osPriorityRealtime, 4096);
rtos::Mutex busLock;

// Shared between the CAN thread and loop(); guarded by busLock.
bool armed = false;
float targetLeft = 0.0f;
float targetRight = 0.0f;
uint32_t lastDriveMs = 0;
bool driveFresh = false;
const char* lastError = "";
SlewRateLimiter leftLimiter(WEB_SLEW_PER_SEC);
SlewRateLimiter rightLimiter(WEB_SLEW_PER_SEC);
float wheelLeft = 0.0f;
float wheelRight = 0.0f;
uint32_t maxGapMs = 0;      // longest gap between CAN services, this second
uint32_t lastMaxGapMs = 0;  // previous second's value, for reporting

// loop() only.
char req[512];
uint16_t reqLen = 0;
uint32_t clientSinceMs = 0;
uint32_t lastReportMs = 0;
uint32_t lastWifiRetryMs = 0;

constexpr uint32_t CONTROL_PERIOD_MS = 10;

int16_t toRpm(float norm, int16_t maxRpm, int8_t direction) {
  return (int16_t)lroundf(clamp(norm, -1.0f, 1.0f) * maxRpm * direction);
}

const char* phaseName(Tm4Inverter::Phase p) {
  switch (p) {
    case Tm4Inverter::Phase::OFF: return "OFF";
    case Tm4Inverter::Phase::PWM_REQUESTED: return "PWM_REQ";
    case Tm4Inverter::Phase::RUNNING: return "RUN";
    case Tm4Inverter::Phase::STOPPING: return "STOPPING";
  }
  return "?";
}

const char* roleName(InverterRole r) {
  switch (r) {
    case InverterRole::DECK: return "deck";
    case InverterRole::DRIVE_LEFT: return "left";
    case InverterRole::DRIVE_RIGHT: return "right";
  }
  return "?";
}

// ---- Shared-state helpers (caller holds busLock) ---------------------------

void clearDrive() {
  targetLeft = targetRight = 0.0f;
  driveFresh = false;
}

void disarm(const char* why) {
  armed = false;
  clearDrive();
  leftLimiter.reset();
  rightLimiter.reset();
  wheelLeft = wheelRight = 0.0f;
  for (uint8_t k = 0; k < tm4Bus.count(); k++) tm4Bus.inverter(k).forceOff();
  tm4Bus.setPowerRequest(false);
  lastError = why;
}

void control(uint32_t now, float dt) {
  // CAN health lost while armed: everything off, operator must re-arm.
  static bool wasHealthy = false;
  const bool healthy = tm4Bus.healthy();
  if (armed && wasHealthy && !healthy) disarm("CAN health lost - re-arm");
  wasHealthy = healthy;

  if (driveFresh && now - lastDriveMs > WEB_COMMAND_TIMEOUT_MS) {
    clearDrive();
    lastError = "command timeout";
  }

  wheelLeft = leftLimiter.step(armed ? targetLeft : 0.0f, dt);
  wheelRight = rightLimiter.step(armed ? targetRight : 0.0f, dt);

  // Run while a command is fresh or the ramp-down is still in progress; once
  // both wheels are at 0 with no command, stop so the inverter brake engages.
  const bool moving = wheelLeft != 0.0f || wheelRight != 0.0f;
  const bool run = armed && (driveFresh || moving);

  for (uint8_t k = 0; k < tm4Bus.count(); k++) {
    Tm4Inverter& inv = tm4Bus.inverter(k);
    const InverterConfig& cfg = inv.config();
    switch (cfg.role) {
      case InverterRole::DECK:
        inv.request(false, 0);
        break;
      case InverterRole::DRIVE_LEFT:
        inv.request(run, toRpm(wheelLeft, TM4_DRIVE_MAX_RPM, cfg.direction));
        break;
      case InverterRole::DRIVE_RIGHT:
        inv.request(run, toRpm(wheelRight, TM4_DRIVE_MAX_RPM, cfg.direction));
        break;
    }
  }
}

void canTask() {
  uint32_t lastServiceMs = millis();
  uint32_t lastControlMs = 0;
  for (;;) {
    const uint32_t now = millis();
    busLock.lock();
    const uint32_t gap = now - lastServiceMs;
    if (gap > maxGapMs) maxGapMs = gap;
    lastServiceMs = now;
    tm4Bus.update(now);
    if (lastControlMs == 0 || now - lastControlMs >= CONTROL_PERIOD_MS) {
      const float dt = lastControlMs == 0 ? 0.0f : (float)(now - lastControlMs) / 1000.0f;
      lastControlMs = now;
      control(now, dt);
    }
    busLock.unlock();
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
  }
}

// Reads "key=<float>" from a query string; false if missing or malformed.
bool queryFloat(const char* query, const char* key, float& out) {
  const size_t keyLen = strlen(key);
  for (const char* p = query; p && *p;) {
    if (strncmp(p, key, keyLen) == 0 && p[keyLen] == '=') {
      char* end = nullptr;
      const float v = strtof(p + keyLen + 1, &end);
      if (end == p + keyLen + 1 || isnan(v)) return false;
      out = clamp(v, -1.0f, 1.0f);
      return true;
    }
    p = strchr(p, '&');
    if (p) p++;
  }
  return false;
}

void appendStatus(String& s, uint32_t now) {
  s += "{\"armed\":";
  s += armed ? "true" : "false";
  s += ",\"streaming\":";
  s += tm4Bus.transmitting() ? "true" : "false";
  s += ",\"canHealthy\":";
  s += tm4Bus.healthy() ? "true" : "false";
  s += ",\"rx\":";
  s += tm4Bus.rxCount();
  s += ",\"tx\":";
  s += tm4Bus.txCount();
  s += ",\"txErr\":";
  s += tm4Bus.txErrors();
  s += ",\"maxGapMs\":";
  s += lastMaxGapMs;
  s += ",\"maxRpm\":";
  s += TM4_DRIVE_MAX_RPM;
  s += ",\"timeoutMs\":";
  s += WEB_COMMAND_TIMEOUT_MS;
  s += ",\"cmdLeft\":";
  s += String(wheelLeft, 3);
  s += ",\"cmdRight\":";
  s += String(wheelRight, 3);
  s += ",\"error\":\"";
  s += lastError;
  s += "\",\"inverters\":[";
  bool first = true;
  for (uint8_t k = 0; k < tm4Bus.count(); k++) {
    Tm4Inverter& inv = tm4Bus.inverter(k);
    const InverterConfig& cfg = inv.config();
    const tm4::Status& st = inv.status();
    if (!first) s += ',';
    first = false;
    s += "{\"name\":\"";
    s += cfg.name;
    s += "\",\"role\":\"";
    s += roleName(cfg.role);
    s += "\",\"present\":";
    s += cfg.present ? "true" : "false";
    s += ",\"healthy\":";
    s += inv.healthy(now) ? "true" : "false";
    s += ",\"phase\":\"";
    s += phaseName(inv.phase());
    s += "\",\"pwmTimeout\":";
    s += inv.pwmTimedOut() ? "true" : "false";
    s += ",\"mains\":";
    s += st.mainsState;
    s += ",\"fault\":";
    s += st.faultCode;
    s += ",\"level\":";
    s += st.faultLevel;
    s += ",\"pwm\":";
    s += st.pwmOutput ? "true" : "false";
    s += ",\"rpm\":";
    s += st.actualRpm;
    s += ",\"dc\":";
    s += String(st.dcBusVoltage, 1);
    s += '}';
  }
  s += "]}";
}

// Applies one request to the shared state and fills the reply (holds busLock).
int handleRequest(char* line, uint32_t now, const char*& reason, String& body) {
  // Request line: "<METHOD> <path>[?query] HTTP/1.x"
  reason = "OK";
  char* sp1 = strchr(line, ' ');
  if (!sp1) { reason = "Bad Request"; body = "{\"error\":\"bad request\"}"; return 400; }
  *sp1 = '\0';
  char* path = sp1 + 1;
  char* sp2 = strchr(path, ' ');
  if (sp2) *sp2 = '\0';
  char* query = strchr(path, '?');
  if (query) *query++ = '\0';

  if (strcmp(line, "OPTIONS") == 0) { reason = "No Content"; return 204; }
  if (strcmp(line, "GET") != 0) {
    reason = "Method Not Allowed";
    body = "{\"error\":\"GET only\"}";
    return 405;
  }

  if (strcmp(path, "/arm") == 0) {
    if (!tm4Bus.transmitting()) {
      lastError = "CAN not started";
    } else {
      armed = true;
      clearDrive();
      tm4Bus.setPowerRequest(true);
      lastError = "";
    }
  } else if (strcmp(path, "/disarm") == 0) {
    disarm("");
  } else if (strcmp(path, "/stop") == 0) {
    clearDrive();
  } else if (strcmp(path, "/drive") == 0) {
    float l = 0.0f, r = 0.0f;
    if (!armed) {
      lastError = "not armed";
    } else if (!tm4Bus.healthy()) {
      clearDrive();
      lastError = "CAN not healthy";
    } else if (!queryFloat(query, "l", l) || !queryFloat(query, "r", r)) {
      reason = "Bad Request";
      body = "{\"error\":\"drive needs l and r in -1..1\"}";
      return 400;
    } else {
      targetLeft = l;
      targetRight = r;
      lastDriveMs = now;
      driveFresh = true;
      lastError = "";
    }
  } else if (strcmp(path, "/status") != 0 && strcmp(path, "/") != 0) {
    reason = "Not Found";
    body = "{\"error\":\"unknown path\"}";
    return 404;
  }

  appendStatus(body, now);
  return 200;
}

// ---- loop()-side I/O (never holds busLock) ---------------------------------

void reply(int code, const char* reason, const String& body) {
  String head;
  head.reserve(256);
  head += "HTTP/1.1 ";
  head += code;
  head += ' ';
  head += reason;
  head += "\r\nContent-Type: application/json\r\n"
          "Access-Control-Allow-Origin: *\r\n"
          "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
          "Access-Control-Allow-Private-Network: true\r\n"
          "Cache-Control: no-store\r\n"
          "Connection: close\r\nContent-Length: ";
  head += body.length();
  head += "\r\n\r\n";
  client.print(head);
  client.print(body);
}

// Picks up at most one client and reads whatever has arrived.
void serviceHttp(uint32_t now) {
  if (!wifiUp) return;
  if (!client) {
    client = server.accept();
    if (!client) return;
    reqLen = 0;
    clientSinceMs = now;
  }

  bool complete = false;
  while (client.available() && !complete) {
    const char c = (char)client.read();
    if (reqLen < sizeof(req) - 1) req[reqLen++] = c;
    req[reqLen] = '\0';
    complete = reqLen >= 4 && strcmp(req + reqLen - 4, "\r\n\r\n") == 0;
  }
  // Header block too long for the buffer: answer from the request line anyway.
  if (reqLen >= sizeof(req) - 1) complete = true;

  if (complete) {
    char* eol = strstr(req, "\r\n");
    if (eol) *eol = '\0';
    String body;
    body.reserve(1536);
    const char* reason = "OK";
    busLock.lock();
    const int code = handleRequest(req, now, reason, body);
    busLock.unlock();
    reply(code, reason, body);
    client.stop();
  } else if (now - clientSinceMs > WEB_CLIENT_TIMEOUT_MS || !client.connected()) {
    client.stop();
  }
}

void report(uint32_t now) {
  String s;
  s.reserve(512);
  busLock.lock();
  lastMaxGapMs = maxGapMs;
  maxGapMs = 0;
  s += "web armed=";
  s += armed;
  s += " wifi=";
  s += wifiUp ? WiFi.localIP().toString() : String("down");
  s += " can_healthy=";
  s += tm4Bus.healthy();
  s += " rx=";
  s += tm4Bus.rxCount();
  s += " tx=";
  s += tm4Bus.txCount();
  s += " tx_err=";
  s += tm4Bus.txErrors();
  s += " can_max_gap_ms=";
  s += lastMaxGapMs;
  s += " cmd L=";
  s += String(wheelLeft, 2);
  s += " R=";
  s += String(wheelRight, 2);
  if (lastError[0]) {
    s += " err=";
    s += lastError;
  }
  s += '\n';
  for (uint8_t k = 0; k < tm4Bus.count(); k++) {
    Tm4Inverter& inv = tm4Bus.inverter(k);
    if (!inv.config().present) continue;
    const tm4::Status& st = inv.status();
    s += "  ";
    s += inv.config().name;
    s += ": healthy=";
    s += inv.healthy(now);
    s += " phase=";
    s += phaseName(inv.phase());
    if (inv.pwmTimedOut()) s += "(PWM confirm timeout)";
    s += " mains=";
    s += st.mainsState;
    s += " fault=";
    s += st.faultCode;
    s += " level=";
    s += st.faultLevel;
    s += " pwm=";
    s += st.pwmOutput;
    s += " rpm=";
    s += st.actualRpm;
    s += " dc=";
    s += String(st.dcBusVoltage, 1);
    s += '\n';
  }
  busLock.unlock();
  Serial.print(s);
}

bool startWifi() {
#if WEB_WIFI_AP_MODE
  Serial.print("Starting Wi-Fi access point \"");
  Serial.print(WEB_AP_SSID);
  Serial.println("\"...");
  if (WiFi.beginAP(WEB_AP_SSID, WEB_AP_PASS) != WL_AP_LISTENING) return false;
#else
  Serial.print("Joining Wi-Fi \"");
  Serial.print(WEB_STA_SSID);
  Serial.println("\"...");
  if (WiFi.begin(WEB_STA_SSID, WEB_STA_PASS) != WL_CONNECTED) return false;
#endif
  server.begin();
  return true;
}

void printApiAddress() {
  Serial.print("Website API: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/status");
}

// Station mode: rejoin after the network drops. WiFi.begin() blocks for
// seconds, which is safe here because CAN and the dead-man run in canThread.
void serviceWifi(uint32_t now) {
#if !WEB_WIFI_AP_MODE
  if (wifiUp && WiFi.status() == WL_CONNECTED) return;
  if (wifiUp) {
    wifiUp = false;
    if (client) client.stop();
    Serial.println("Wi-Fi lost.");
  }
  if (lastWifiRetryMs != 0 && now - lastWifiRetryMs < WEB_STA_RETRY_MS) return;
  lastWifiRetryMs = now;
  wifiUp = startWifi();
  if (wifiUp) printApiAddress();
#else
  (void)now;
#endif
}
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  wifiUp = startWifi();
  lastWifiRetryMs = millis();
  if (wifiUp) printApiAddress();
  else Serial.println("Wi-Fi failed to start; CAN runs with motors off.");

  // Streams RPDO1-4 from here on with motors OFF and power request Startup.
  const bool started = tm4Bus.begin(millis());
  tm4Bus.setPowerRequest(false);
  canThread.start(canTask);
  Serial.println(started ? "CAN: FDCAN2 @ 250000 bit/s, streaming, disarmed."
                         : "CAN bus failed to start at 250 kbit/s.");
}

void loop() {
  const uint32_t now = millis();
  serviceWifi(now);
  serviceHttp(now);
  if (now - lastReportMs >= 1000) {
    lastReportMs = now;
    report(now);
  }
  // Yield so same-priority core threads run (the USB 1200-baud upload reset
  // is serviced by one); CAN timing is unaffected, it lives in canThread.
  delay(1);
}
