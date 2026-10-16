/*
 * AstralPool RS485 WiFi REST API - ESP32-C3 SuperMini
 * Version 1.3.14
 * RS485 auto-direction module: GPIO4 (ESP TX) -> RXD, GPIO3 (ESP RX) <- TXD.
 * GND -> GND; VCC voltage must be verified against module specifications.
 * RS485 A+ -> device A; B- -> device B.
 * Requires ModbusMaster library (Doc Walker).
 * pH/Redox register mappings confirmed; hydraulic flow register not yet identified.
 */
#include <WiFi.h>
#include <ESP32Ping.h>
#include <WebServer.h>
#include <ModbusMaster.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "CachedRegister.h"

struct Measurement { bool ok; float value; uint8_t result; };
struct FlowReading { bool ok; bool present; uint16_t raw; uint8_t result; };

constexpr const char* FIRMWARE_VERSION = "1.3.14";
const char* WIFI_SSID = "FUN5";
const char* WIFI_PASSWORD = "atout.be";
constexpr int RS485_RX = 3;
constexpr int RS485_TX = 4;
constexpr uint16_t HTTP_PORT = 80;
// Change this secret before flashing. Only POST /api/restart accepts it.
const char* RESTART_TOKEN = "CHANGE_ME_LONG_RANDOM_SECRET";
constexpr uint32_t WIFI_RETRY_MS = 10000;
constexpr uint32_t WIFI_RESTART_MS = 300000; // status compatibility; target watchdog governs reboot
constexpr uint32_t TARGET_CHECK_INTERVAL_MS = 15000;
constexpr uint32_t TARGET_RESTART_MS = 300000; // 5 minutes of continuous failures
const IPAddress TARGET_IP(192, 168, 10, 100);
uint32_t targetLastCheckAt = 0;
uint32_t targetFailureSince = 0;
bool targetReachable = false;
bool targetEverChecked = false;
uint32_t targetPingFailures = 0;
uint32_t targetPingSuccesses = 0;
constexpr uint32_t POLL_INTERVAL_MS = 1500;
uint32_t wifiLostAt = 0, wifiLastRetryAt = 0, wifiReconnectAttempts = 0;
uint32_t wifiRestarts = 0; // RAM counter; reset after ESP restart
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t lastRequestedRestart = 0; // 1=target unavailable, 2=HTTP
SemaphoreHandle_t busMutex = nullptr;
SemaphoreHandle_t cacheMutex = nullptr;
TaskHandle_t modbusTaskHandle = nullptr;
uint32_t modbusConsecutiveFailures = 0;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 30000;
uint32_t restartAt = 0;
bool restartScheduled = false;


HardwareSerial& RS485 = Serial1;
ModbusMaster modbus;
WebServer server(HTTP_PORT);
uint8_t slaveId = 1;
uint32_t baudRate = 19200;
uint8_t parityMode = 0; // 0=N, 1=E, 2=O
uint32_t requests = 0, successes = 0, timeouts = 0, errors = 0;
uint8_t lastResult = 0;
uint32_t lastDuration = 0;
String lastOperation = "none";


uint32_t serialConfig() {
  if (parityMode == 1) return SERIAL_8E1;
  if (parityMode == 2) return SERIAL_8O1;
  return SERIAL_8N1;
}
void initModbus() {
  RS485.end();
  RS485.begin(baudRate, serialConfig(), RS485_RX, RS485_TX);
  modbus.begin(slaveId, RS485);
}
String quoted(const String& s) {
  String out = "\"";
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c == '"' || c == '\\') out += '\\';
    if (c == '\n' || c == '\r') out += ' ';
    else out += c;
  }
  return out + '"';
}
void sendJson(int code, const String& payload) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json; charset=utf-8", payload);
}
uint32_t parseNum(const String& s, bool& valid) {
  valid = false;
  if (!s.length()) return 0;
  const char* start = s.c_str();
  char* end = nullptr;
  unsigned long v = strtoul(start, &end, 0); // decimal or 0xHEX
  valid = (end != start && *end == '\0' && s[0] != '-');
  return v;
}
String resultName(uint8_t r) {
  switch (r) {
    case 0: return "OK";
    case 0x01: return "Illegal Function";
    case 0x02: return "Illegal Data Address";
    case 0x03: return "Illegal Data Value";
    case 0x04: return "Slave Device Failure";
    case 0xE0: return "Invalid Slave ID";
    case 0xE1: return "Invalid Function";
    case 0xE2: return "Response Timeout";
    case 0xE3: return "Invalid CRC";
    default: return "Modbus error";
  }
}
// A single-threaded WebServer and ModbusMaster serialize all RS485 access.
// Cached values are never presented as fresh after the TTL expires.
// CachedRegister is defined in CachedRegister.h (before Arduino-generated prototypes).
CachedRegister cache[] = {
  {1202,2,{0,0},false,0,0xE2}, {1204,2,{0,0},false,0,0xE2},
  {1302,2,{0,0},false,0,0xE2}, {1304,2,{0,0},false,0,0xE2},
  {1006,1,{0,0},false,0,0xE2}, {1007,1,{0,0},false,0,0xE2}, {1100,1,{0,0},false,0,0xE2}
};
constexpr uint32_t CACHE_TTL_MS = 10000;
uint32_t modbusRetries = 0;
uint32_t lastSuccessAt = 0;
uint32_t lastPollAt = 0;
uint8_t nextPoll = 0;
// The worker owns cache writes. HTTP only receives locked copies.
bool snapshotRegister(uint16_t address, CachedRegister& out) {
  if (!cacheMutex || xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
  bool found = false;
  for (const auto& item : cache) if (item.addr == address) { out = item; found = true; break; }
  xSemaphoreGive(cacheMutex);
  return found;
}
String lastReadAt(uint16_t address);
CachedRegister* cached(uint16_t address) {
  for (auto& item : cache) if (item.addr == address) return &item;
  return nullptr;
}
// ISO 8601 date/time of the LAST SUCCESSFUL Modbus read, not HTTP request time.
// Uses NTP clock and elapsed milliseconds; returns null until NTP is synchronized.
String lastReadTimestamp(const CachedRegister* item) {
  if (!item || !item->valid) return "null";
  time_t now = time(nullptr);
  if (now < 1700000000) return "null";  // Clock not synchronized
  uint32_t elapsed = (uint32_t)(millis() - item->updated);
  time_t observed = now - (time_t)(elapsed / 1000);
  struct tm localTime;
  localtime_r(&observed, &localTime);
  char stamp[40];
  if (strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S%z", &localTime) == 0) return "null";
  String iso(stamp);
  // Convert +0200 to ISO 8601 +02:00
  if (iso.length() >= 5) iso = iso.substring(0, iso.length()-2) + ":" + iso.substring(iso.length()-2);
  return quoted(iso);
}
String lastReadAt(uint16_t address) {
  CachedRegister item;
  return snapshotRegister(address, item) ? lastReadTimestamp(&item) : String("null");
}
String ageMs(uint16_t address) {
  CachedRegister item;
  return snapshotRegister(address, item) && item.valid ? String((uint32_t)(millis()-item.updated)) : String("null");
}
bool fresh(const CachedRegister* item) {
  return item && item->valid && (uint32_t)(millis()-item->updated) <= CACHE_TTL_MS;
}
uint8_t transact(uint8_t fn, uint16_t addr, uint8_t count) {
  requests++;
  lastOperation = "function=" + String(fn) + " address=" + String(addr) + " count=" + String(count);
  uint32_t start = millis();
  uint8_t result = (fn == 3) ? modbus.readHoldingRegisters(addr,count) : modbus.readInputRegisters(addr,count);
  lastDuration = millis()-start;
  lastResult = result;
  if (result == 0) { successes++; lastSuccessAt = millis(); }
  else if (result == 0xE2) timeouts++;
  else errors++;
  return result;
}
uint8_t transactRetry(uint8_t fn, uint16_t addr, uint8_t count) {
  uint8_t code = transact(fn,addr,count);
  if (code == 0xE2) { delay(80); modbusRetries++; code = transact(fn,addr,count); }
  return code;
}
void pollOne() {
  const uint8_t index = nextPoll;
  nextPoll = (nextPoll + 1) % (sizeof(cache)/sizeof(cache[0]));
  const uint16_t addr = cache[index].addr;
  const uint8_t count = cache[index].count;
  if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(10000)) != pdTRUE) return;
  uint8_t code = transactRetry(3,addr,count);
  uint16_t words[2] = {0,0};
  if (code == 0) for (uint8_t i=0;i<count;i++) words[i] = modbus.getResponseBuffer(i);
  xSemaphoreGive(busMutex);
  if (xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    cache[index].code = code;
    if (code == 0) {
      for (uint8_t i=0;i<count;i++) cache[index].words[i] = words[i];
      cache[index].updated=millis(); cache[index].valid=true;
    }
    xSemaphoreGive(cacheMutex);
  }
  if (code == 0) modbusConsecutiveFailures = 0;
  else modbusConsecutiveFailures++;
}
String readRegisters(uint8_t fn, uint16_t addr, uint8_t count) {
  lastOperation = "function=" + String(fn) + " address=" + String(addr) + " count=" + String(count);
  uint32_t start = millis();
  if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(6000)) != pdTRUE)
    return "{\"ok\":false,\"result_code\":-1,\"result\":\"Modbus busy\",\"registers\":[]}";
  uint8_t result = transactRetry(fn, addr, count);
  lastDuration = millis() - start;
  lastResult = result;
  String out = "{\"ok\":" + String(result == 0 ? "true" : "false") +
               ",\"slave\":" + String(slaveId) + ",\"function\":" + String(fn) +
               ",\"address\":" + String(addr) + ",\"count\":" + String(count) +
               ",\"duration_ms\":" + String(lastDuration) +
               ",\"result_code\":" + String(result) +
               ",\"result\":" + quoted(resultName(result)) + ",\"registers\":[";
  if (result == 0) {
    for (uint8_t i = 0; i < count; i++) {
      if (i) out += ',';
      out += String(modbus.getResponseBuffer(i));
    }
  }
  xSemaphoreGive(busMutex);
  return out + "]}";
}
void handleRead() {
  if (!server.hasArg("address")) { sendJson(400, "{\"error\":\"Missing address\"}"); return; }
  bool okAddr, okFn, okCount;
  uint32_t addr = parseNum(server.arg("address"), okAddr);
  uint32_t fn = parseNum((server.hasArg("function") ? server.arg("function") : String("4")), okFn);
  uint32_t count = parseNum((server.hasArg("count") ? server.arg("count") : String("1")), okCount);
  if (!okAddr || addr > 65535 || !okFn || (fn != 3 && fn != 4) ||
      !okCount || count < 1 || count > 32 || addr + count > 65536) {
    sendJson(400, "{\"error\":\"Invalid address/function/count (max count 32)\"}"); return;
  }
  String response = readRegisters((uint8_t)fn, (uint16_t)addr, (uint8_t)count);
  sendJson(response.indexOf("\"ok\":true") >= 0 ? 200 : 502, response);
}
// HYDRO TOUCH CHY0444-AST: FC03, two 16-bit registers per float,
// low word first (CDAB). holding register 1007 tracks the hydraulic flow switch (SPDT4).
Measurement readFloatAt(uint16_t address) {
  Measurement m = {false, NAN, 0xE2};
  CachedRegister item;
  if (!snapshotRegister(address, item)) return m;
  m.result = item.code;
  if (!fresh(&item)) return m;
  uint32_t bits = (uint32_t(item.words[1]) << 16) | item.words[0];
  memcpy(&m.value, &bits, sizeof(m.value));
  m.ok = isfinite(m.value);
  return m;
}
String jsonMeasurement(const Measurement& m, uint8_t decimals) {
  return m.ok ? String(m.value, static_cast<unsigned int>(decimals)) : String("null");
}
// Hydraulic flow switch now wired to SPDT4; 1 = flow detected, 0 = no flow.
FlowReading readFlow() {
  FlowReading f = {false, false, 0, 0xE2};
  CachedRegister item;
  if (!snapshotRegister(1007, item)) return f;
  f.result = item.code;
  if (!fresh(&item)) return f;
  f.raw = item.words[0];
  if (f.raw <= 1) { f.ok=true; f.present=(f.raw==1); }
  return f;
}
String jsonFlow(const FlowReading& f) {
  return f.ok ? String(f.present ? "true" : "false") : String("null");
}
// SPDT1 pump auxiliary contact: candidate register 1006.
FlowReading readPump() {
  FlowReading p = {false, false, 0, 0xE2};
  CachedRegister item;
  if (!snapshotRegister(1006, item)) return p;
  p.result = item.code;
  if (!fresh(&item)) return p;
  p.raw = item.words[0];
  if (p.raw <= 1) { p.ok = true; p.present = (p.raw == 1); }
  return p;
}
void handleMeasurements() {
  Measurement ph = readFloatAt(1202);
  Measurement phSetpoint = readFloatAt(1204);
  Measurement redox = readFloatAt(1302);
  Measurement redoxSetpoint = readFloatAt(1304);
  FlowReading flow = readFlow();
  FlowReading pump = readPump();
  bool ok = ph.ok && phSetpoint.ok && redox.ok && redoxSetpoint.ok;
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"ok\":" + String(ok ? "true" : "false") +
               ",\"slave\":" + String(slaveId) +
               ",\"ph\":" + jsonMeasurement(ph, 3) +
               ",\"ph_setpoint\":" + jsonMeasurement(phSetpoint, 3) +
               ",\"redox\":" + jsonMeasurement(redox, 2) +
               ",\"redox_mv\":" + jsonMeasurement(redox, 2) +
               ",\"redox_setpoint\":" + jsonMeasurement(redoxSetpoint, 2) +
               ",\"ph_last_read_at\":" + lastReadAt(1202) +
               ",\"redox_last_read_at\":" + lastReadAt(1302) +
               ",\"pump_last_read_at\":" + lastReadAt(1006) +
               ",\"flow_last_read_at\":" + lastReadAt(1007) +
               ",\"flow\":" + jsonFlow(flow) +
               ",\"flow_verified\":" + String(flow.ok ? "true" : "false") +
               ",\"flow_register\":1007" +
               ",\"flow_raw\":" + (flow.ok ? String(flow.raw) : String("null")) +
               ",\"flow_modbus_code\":" + String(flow.result) +
               ",\"pump\":" + jsonFlow(pump) +
               ",\"pump_register\":1006" +
               ",\"pump_raw\":" + (pump.ok ? String(pump.raw) : String("null")) +
               ",\"pump_modbus_code\":" + String(pump.result) +
               ",\"unit_redox\":\"mV\"" +
               ",\"cached\":true,\"cache_ttl_ms\":" + String(CACHE_TTL_MS) +
               ",\"ph_age_ms\":" + ageMs(1202) +
               ",\"pump_age_ms\":" + ageMs(1006) +
               ",\"flow_age_ms\":" + ageMs(1007) +
               ",\"modbus_codes\":{\"ph\":" + String(ph.result) +
               ",\"ph_setpoint\":" + String(phSetpoint.result) +
               ",\"redox\":" + String(redox.result) +
               ",\"redox_setpoint\":" + String(redoxSetpoint.result) + "}}";
  sendJson(ok ? 200 : 502, out);
}
// Individual read-only endpoints: each reads only its own measurement and setpoint.
void handlePh() {
  Measurement value = readFloatAt(1202);
  Measurement setpoint = readFloatAt(1204);
  bool ok = value.ok && setpoint.ok;
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"ok\":" + String(ok ? "true" : "false") +
               ",\"ph\":" + jsonMeasurement(value, 3) +
               ",\"ph_last_read_at\":" + lastReadAt(1202) +
               ",\"ph_setpoint\":" + jsonMeasurement(setpoint, 3) +
               ",\"modbus_codes\":{\"ph\":" + String(value.result) +
               ",\"ph_setpoint\":" + String(setpoint.result) + "}}";
  sendJson(ok ? 200 : 502, out);
}
void handleRedox() {
  Measurement value = readFloatAt(1302);
  Measurement setpoint = readFloatAt(1304);
  bool ok = value.ok && setpoint.ok;
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"ok\":" + String(ok ? "true" : "false") +
               ",\"redox\":" + jsonMeasurement(value, 2) +
               ",\"redox_last_read_at\":" + lastReadAt(1302) +
               ",\"redox_mv\":" + jsonMeasurement(value, 2) +
               ",\"redox_setpoint\":" + jsonMeasurement(setpoint, 2) +
               ",\"unit\":\"mV\"" +
               ",\"modbus_codes\":{\"redox\":" + String(value.result) +
               ",\"redox_setpoint\":" + String(setpoint.result) + "}}";
  sendJson(ok ? 200 : 502, out);
}
// Flow switch on SPDT4, holding register 1007.
void handleFlow() {
  FlowReading flow = readFlow();
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"ok\":" + String(flow.ok ? "true" : "false") +
               ",\"flow\":" + jsonFlow(flow) +
               ",\"flow_verified\":" + String(flow.ok ? "true" : "false") +
               ",\"flow_last_read_at\":" + lastReadAt(1007) +
               ",\"flow_register\":1007" +
               ",\"flow_raw\":" + (flow.ok ? String(flow.raw) : String("null")) +
               ",\"modbus_code\":" + String(flow.result) + "}";
  sendJson(flow.ok ? 200 : 502, out);
}
// Pump auxiliary contact on SPDT1: holding register 1006.
void handlePump() {
  FlowReading pump = readPump();
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"ok\":" + String(pump.ok ? "true" : "false") +
               ",\"pump\":" + jsonFlow(pump) +
               ",\"pump_last_read_at\":" + lastReadAt(1006) +
               ",\"pump_register\":1006" +
               ",\"pump_raw\":" + (pump.ok ? String(pump.raw) : String("null")) +
               ",\"modbus_code\":" + String(pump.result) + "}";
  sendJson(pump.ok ? 200 : 502, out);
}
void handleRestart() {
  if (strcmp(RESTART_TOKEN, "CHANGE_ME_LONG_RANDOM_SECRET") == 0) {
    sendJson(503, "{\"ok\":false,\"error\":\"Configure RESTART_TOKEN before enabling restart\"}");
    return;
  }
  if (!server.hasArg("token") || server.arg("token") != RESTART_TOKEN) {
    sendJson(403, "{\"ok\":false,\"error\":\"Invalid token\"}");
    return;
  }
  sendJson(200, "{\"ok\":true,\"restarting\":true}");
  restartAt = millis();
  restartScheduled = true;
}
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiLostAt) Serial.println("WiFi reconnected: " + WiFi.localIP().toString());
    wifiLostAt = 0;
    return;
  }
  uint32_t now = millis();
  if (!wifiLostAt) {
    wifiLostAt = now ? now : 1;
    wifiLastRetryAt = now - WIFI_RETRY_MS;
    Serial.println("WiFi disconnected");
  }
  if ((uint32_t)(now - wifiLastRetryAt) >= WIFI_RETRY_MS) {
    wifiLastRetryAt = now;
    wifiReconnectAttempts++;
    WiFi.reconnect();
    Serial.println("WiFi reconnect attempt " + String(wifiReconnectAttempts));
  }
}
void maintainTargetReachability() {
  const uint32_t now = millis();
  if (targetEverChecked && (uint32_t)(now - targetLastCheckAt) < TARGET_CHECK_INTERVAL_MS) return;
  targetEverChecked = true;
  targetLastCheckAt = now;

  // Loss of WiFi counts as target unreachable. Do not block on ICMP without WiFi.
  const bool reachable = (WiFi.status() == WL_CONNECTED) && Ping.ping(TARGET_IP, 1);
  targetReachable = reachable;
  if (reachable) {
    if (targetFailureSince) Serial.println("Target 192.168.10.100 reachable again");
    targetFailureSince = 0;
    targetPingSuccesses++;
  } else {
    targetPingFailures++;
    if (!targetFailureSince) {
      targetFailureSince = now ? now : 1;
      Serial.println("Target 192.168.10.100 unreachable: starting 5-minute timer");
    }
    if ((uint32_t)(now - targetFailureSince) >= TARGET_RESTART_MS) {
      Serial.println("Target unreachable for 5 minutes; restarting ESP");
      Serial.flush();
      lastRequestedRestart = 1;
      ESP.restart();
    }
  }
}
void handleStatus() {
  // Holding register 1100: manual switch telling whether the machine is enabled.
  // It does NOT mean a pH/redox injection is running: the machine decides that itself.
  // Read on demand; never interpret a Modbus failure as machine disabled.
  CachedRegister machItem;
  const bool hasMachine = snapshotRegister(1100, machItem);
  const bool machineReadOk = hasMachine && fresh(&machItem);
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") +
               ",\"ip\":" + quoted(WiFi.localIP().toString()) +
               ",\"rssi_dbm\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) +
               ",\"wifi_reconnect_attempts\":" + String(wifiReconnectAttempts) +
               ",\"wifi_disconnected_ms\":" + (wifiLostAt ? String((uint32_t)(millis()-wifiLostAt)) : String("0")) +
               ",\"wifi_restart_after_ms\":" + String(WIFI_RESTART_MS) +
               ",\"target_ip\":\"192.168.10.100\"" +
               ",\"target_reachable\":" + (targetEverChecked ? String(targetReachable ? "true" : "false") : String("null")) +
               ",\"target_check_interval_ms\":" + String(TARGET_CHECK_INTERVAL_MS) +
               ",\"target_unreachable_ms\":" + (targetFailureSince ? String((uint32_t)(millis() - targetFailureSince)) : String("0")) +
               ",\"target_restart_after_ms\":" + String(TARGET_RESTART_MS) +
               ",\"target_ping_successes\":" + String(targetPingSuccesses) +
               ",\"target_ping_failures\":" + String(targetPingFailures) +
               ",\"uptime_ms\":" + String(millis()) +
               ",\"boot_count\":" + String(bootCount) +
               ",\"restart_reason\":" + quoted(esp_reset_reason() == ESP_RST_TASK_WDT ? "task_watchdog" :
                  esp_reset_reason() == ESP_RST_WDT ? "watchdog" :
                  esp_reset_reason() == ESP_RST_BROWNOUT ? "brownout" :
                  lastRequestedRestart == 1 ? "target_unreachable" :
                  lastRequestedRestart == 2 ? "remote_api" : "other_or_power_on") +
               ",\"free_heap\":" + String(ESP.getFreeHeap()) +
               ",\"modbus_consecutive_failures\":" + String(modbusConsecutiveFailures) +
               ",\"time_synced\":" + String(time(nullptr) >= 1700000000 ? "true" : "false") +
               ",\"ph_last_read_at\":" + lastReadAt(1202) +
               ",\"redox_last_read_at\":" + lastReadAt(1302) +
               ",\"pump_last_read_at\":" + lastReadAt(1006) +
               ",\"flow_last_read_at\":" + lastReadAt(1007) +
               ",\"slave\":" + String(slaveId) +
               ",\"machine_enabled\":" + (machineReadOk && machItem.words[0] <= 1 ? String(machItem.words[0] == 1 ? "true" : "false") : String("null")) +
               ",\"machine_register\":1100" +
               ",\"baud\":" + String(baudRate) +
               ",\"parity\":" + quoted(parityMode == 0 ? "N" : (parityMode == 1 ? "E" : "O")) +
               ",\"requests\":" + String(requests) +
               ",\"modbus_retries\":" + String(modbusRetries) +
               ",\"modbus_ok\":" + String(successes) +
               ",\"modbus_timeouts\":" + String(timeouts) +
               ",\"last_success_ms\":" + (lastSuccessAt ? String(lastSuccessAt) : String("null")) +
               ",\"data_age_ms\":" + (lastSuccessAt ? String(millis()-lastSuccessAt) : String("null")) +
               ",\"ok\":" + String(successes) +
               ",\"timeouts\":" + String(timeouts) +
               ",\"errors\":" + String(errors) +
               ",\"last_result_code\":" + String(lastResult) +
               ",\"last_duration_ms\":" + String(lastDuration) +
               ",\"last_operation\":" + quoted(lastOperation) + "}";
  sendJson(200, out);
}
void handleConfig() {
  bool valid;
  uint32_t newSlave = slaveId, newBaud = baudRate;
  uint8_t newParity = parityMode;
  if (server.hasArg("slave")) {
    newSlave = parseNum(server.arg("slave"), valid);
    if (!valid || newSlave < 1 || newSlave > 247) { sendJson(400, "{\"error\":\"slave must be 1..247\"}"); return; }
  }
  if (server.hasArg("baud")) {
    newBaud = parseNum(server.arg("baud"), valid);
    if (!valid || (newBaud != 1200 && newBaud != 2400 && newBaud != 4800 && newBaud != 9600 && newBaud != 19200 && newBaud != 38400 && newBaud != 57600 && newBaud != 115200)) {
      sendJson(400, "{\"error\":\"unsupported baud\"}"); return;
    }
  }
  if (server.hasArg("parity")) {
    String p = server.arg("parity"); p.toUpperCase();
    if (p != "N" && p != "E" && p != "O") { sendJson(400, "{\"error\":\"parity must be N/E/O\"}"); return; }
    newParity = (p == "E") ? 1 : ((p == "O") ? 2 : 0);
  }
  if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(6000)) != pdTRUE) {
    sendJson(503,"{\"ok\":false,\"error\":\"Modbus busy\"}"); return;
  }
  slaveId = newSlave; baudRate = newBaud; parityMode = newParity;
  initModbus();
  xSemaphoreGive(busMutex);
  sendJson(200, "{\"ok\":true,\"slave\":" + String(slaveId) + ",\"baud\":" + String(baudRate) + ",\"parity\":" + quoted(parityMode == 0 ? "N" : (parityMode == 1 ? "E" : "O")) + "}");
}

// Discovery uses holding registers (FC03), never writes to the regulator.
// Each request scans at most 100 registers in chunks of 20. Use start=100 etc.
uint16_t swap16(uint16_t x) { return (x >> 8) | (x << 8); }
float decodeFloat(uint16_t a, uint16_t b, uint8_t order) {
  uint8_t bytes[4] = {uint8_t(a >> 8), uint8_t(a), uint8_t(b >> 8), uint8_t(b)};
  uint8_t o[4];
  switch (order) {
    case 1: o[0]=bytes[2];o[1]=bytes[3];o[2]=bytes[0];o[3]=bytes[1];break; // CDAB
    case 2: o[0]=bytes[1];o[1]=bytes[0];o[2]=bytes[3];o[3]=bytes[2];break; // BADC
    case 3: o[0]=bytes[3];o[1]=bytes[2];o[2]=bytes[1];o[3]=bytes[0];break; // DCBA
    default: memcpy(o, bytes, 4); break; // ABCD
  }
  uint32_t bits = (uint32_t(o[0])<<24)|(uint32_t(o[1])<<16)|(uint32_t(o[2])<<8)|o[3];
  float value; memcpy(&value, &bits, sizeof(value)); return value;
}
void handleScan() {
  bool valid;
  uint32_t start = parseNum(server.hasArg("start") ? server.arg("start") : String("0"), valid);
  if (!valid || start > 65535) { sendJson(400,"{\"error\":\"Invalid start\"}"); return; }
  uint32_t count = parseNum(server.hasArg("count") ? server.arg("count") : String("100"), valid);
  if (!valid || count < 1 || count > 100 || start + count > 65536) {
    sendJson(400,"{\"error\":\"count must be 1..100 and within address range\"}");return;
  }
  // Expected displayed values; parameters do not change device settings.
  float expectedPH = server.hasArg("ph") ? server.arg("ph").toFloat() : 7.41f;
  float expectedORP = server.hasArg("redox") ? server.arg("redox").toFloat() : 751.0f;
  if (expectedPH < 0 || expectedPH > 14 || expectedORP < -2000 || expectedORP > 2000) {
    sendJson(400,"{\"error\":\"Invalid expected measurements\"}");return;
  }
  String out = "{\"version\":\"1.2.0\",\"verified_for_this_device\":false,\"function\":3,\"start\":"+String(start)+
    ",\"count\":"+String(count)+",\"expected_ph\":"+String(expectedPH,2)+",\"expected_redox_mv\":"+String(expectedORP,1)+
    ",\"matches\":[";
  bool first=true; uint32_t successful=0, failed=0;
  for (uint32_t base=start; base<start+count; base+=20) {
    uint8_t n=(uint8_t)min(uint32_t(20),start+count-base);
    if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(6000)) != pdTRUE) { failed++; continue; }
    requests++;lastOperation="scan function=3 address="+String(base)+" count="+String(n);
    uint32_t t=millis();uint8_t r=modbus.readHoldingRegisters((uint16_t)base,n);
    lastDuration=millis()-t;lastResult=r;
    uint16_t scanWords[20] = {0};
    if (r == 0) for (uint8_t i=0;i<n;i++) scanWords[i]=modbus.getResponseBuffer(i);
    xSemaphoreGive(busMutex);
    if (r!=0) {failed++; if(r==0xE2)timeouts++;else errors++;continue;}
    successes++;successful++;
    for (uint8_t i=0;i<n;i++) {
      uint16_t v=scanWords[i], w=swap16(v);
      for (int k=0;k<2;k++) {
        int32_t raw=k?w:v;
        for (int d=0;d<=3;d++) {
          float scale=(d==0?1.0f:(d==1?10.0f:(d==2?100.0f:1000.0f)));
          float val=raw/scale;
          bool ph=fabsf(val-expectedPH)<0.035f;
          bool orp=fabsf(val-expectedORP)<1.1f;
          if (ph||orp) {
            if(!first)out+=',';first=false;
            out+="{\"address\":"+String(base+i)+",\"type\":\""+(ph?String("ph"):String("redox"))+"\",\"encoding\":\""+(k?String("uint16_swapped"):String("uint16"))+"\",\"divisor\":"+String((int)scale)+",\"raw\":"+String(v)+",\"value\":"+String(val,3)+"}";
          }
        }
      }
      if (i+1<n) {
        uint16_t b=scanWords[i+1];
        const char* orders[]={"ABCD","CDAB","BADC","DCBA"};
        for (uint8_t o=0;o<4;o++) {
          float f=decodeFloat(v,b,o);
          if (!isfinite(f))continue;
          bool ph=fabsf(f-expectedPH)<0.035f;
          bool orp=fabsf(f-expectedORP)<1.1f;
          if (ph||orp) {
            if(!first)out+=',';first=false;
            out+="{\"address\":"+String(base+i)+",\"type\":\""+(ph?String("ph"):String("redox"))+"\",\"encoding\":\"float32_"+String(orders[o])+"\",\"value\":"+String(f,3)+"}";
          }
        }
      }
    }
    delay(30);
  }
  out+= "],\"blocks_ok\":"+String(successful)+",\"blocks_failed\":"+String(failed)+
    ",\"next_start\":"+String(start+count)+",\"note\":\"Candidates only; confirm by observing changing readings\"}";
  sendJson(200,out);
}

void handleRoot() {
  server.send(200, "text/plain; charset=utf-8",
    "AstralPool RS485 API v1.3.14 (Modbus read-only)\n"
    "GET /api/status\n"
    "GET /api/measurements (pH and redox)\n"
    "GET /api/flow (hydraulic SPDT4: register 1007)\n"
    "GET /api/pump (SPDT1: register 1006)\n"
    "POST /api/restart?token=YOUR_TOKEN\n"
    "GET /api/scan?start=0&count=100&ph=7.41&redox=751\n"
    "GET /api/read?function=4&address=0x51&count=1\n"
    "POST /api/config?slave=2&baud=9600&parity=N\n"
    "Restart requires token; other endpoints trusted LAN only.\n");
}
// Dedicated worker: Modbus timeout never blocks the HTTP event loop.
void modbusWorker(void*) {
  esp_task_wdt_add(nullptr);
  for (;;) {
    pollOne();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}
void setup() {
  Serial.begin(115200);
  bootCount++;
  busMutex = xSemaphoreCreateMutex();
  cacheMutex = xSemaphoreCreateMutex();
  if (!busMutex || !cacheMutex) { Serial.println("Mutex allocation failed"); ESP.restart(); }
  initModbus();
  esp_task_wdt_config_t wdtConfig = {};
  wdtConfig.timeout_ms = WATCHDOG_TIMEOUT_MS;
  wdtConfig.idle_core_mask = 0;
  wdtConfig.trigger_panic = true;
  esp_err_t wdtResult = esp_task_wdt_reconfigure(&wdtConfig);
  if (wdtResult != ESP_OK) wdtResult = esp_task_wdt_init(&wdtConfig);
  if (wdtResult == ESP_OK) esp_task_wdt_add(nullptr);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.setSleep(false); // improve reliability on some ESP32-C3 access points
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300); Serial.print('.');
  }
  Serial.println();
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  // Brussels timezone: CET in winter, CEST in summer; NTP resyncs automatically.
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/read", HTTP_GET, handleRead);
  server.on("/api/measurements", HTTP_GET, handleMeasurements);
  server.on("/api/ph", HTTP_GET, handlePh);
  server.on("/api/redox", HTTP_GET, handleRedox);
  server.on("/api/flow", HTTP_GET, handleFlow);
  server.on("/api/pump", HTTP_GET, handlePump);
  server.on("/api/scan", HTTP_GET, handleScan);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.onNotFound([](){ sendJson(404, "{\"error\":\"Not found\"}"); });
  server.begin();
  if (xTaskCreate(modbusWorker, "modbusWorker", 6144, nullptr, 1, &modbusTaskHandle) != pdPASS)
    Serial.println("ERROR: Modbus worker not started");
}
void loop() {
  // Always service HTTP and WiFi before starting potentially slow RS485 reads.
  server.handleClient();
  maintainWiFi();
  maintainTargetReachability();
  if (restartScheduled && (uint32_t)(millis() - restartAt) >= 300) {
    Serial.println("Remote restart requested");
    Serial.flush();
    lastRequestedRestart = 2;
    ESP.restart();
  }
  esp_task_wdt_reset();
  delay(1);
}
