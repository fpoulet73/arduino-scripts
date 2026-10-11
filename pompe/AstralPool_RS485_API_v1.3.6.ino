/*
 * AstralPool RS485 WiFi REST API - ESP32-C3 SuperMini
 * Version 1.3.6
 * RS485 auto-direction module: GPIO4 (ESP TX) -> RXD, GPIO3 (ESP RX) <- TXD.
 * GND -> GND; VCC voltage must be verified against module specifications.
 * RS485 A+ -> device A; B- -> device B.
 * Requires ModbusMaster library (Doc Walker).
 * pH/Redox register mappings confirmed; hydraulic flow register not yet identified.
 */
#include <WiFi.h>
#include <WebServer.h>
#include <ModbusMaster.h>
#include <math.h>
#include <string.h>

struct Measurement { bool ok; float value; uint8_t result; };
struct FlowReading { bool ok; bool present; uint16_t raw; uint8_t result; };

constexpr const char* FIRMWARE_VERSION = "1.3.6";
const char* WIFI_SSID = "FUN5";
const char* WIFI_PASSWORD = "atout.be";
constexpr int RS485_RX = 3;
constexpr int RS485_TX = 4;
constexpr uint16_t HTTP_PORT = 80;

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
String readRegisters(uint8_t fn, uint16_t addr, uint8_t count) {
  requests++;
  lastOperation = "function=" + String(fn) + " address=" + String(addr) + " count=" + String(count);
  uint32_t start = millis();
  uint8_t result = (fn == 3) ? modbus.readHoldingRegisters(addr, count)
                             : modbus.readInputRegisters(addr, count);
  lastDuration = millis() - start;
  lastResult = result;
  if (result == 0) successes++;
  else if (result == 0xE2) timeouts++;
  else errors++;
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
  sendJson(lastResult == 0 ? 200 : 502, response);
}
// HYDRO TOUCH CHY0444-AST: FC03, two 16-bit registers per float,
// low word first (CDAB). holding register 1007 tracks the pump contact (SPDT4).
Measurement readFloatAt(uint16_t address) {
  Measurement m = {false, NAN, 0};
  readRegisters(3, address, 2);
  m.result = lastResult;
  if (m.result != 0) return m;
  uint16_t low = modbus.getResponseBuffer(0);
  uint16_t high = modbus.getResponseBuffer(1);
  uint32_t bits = (uint32_t(high) << 16) | low;
  memcpy(&m.value, &bits, sizeof(m.value));
  m.ok = isfinite(m.value);
  return m;
}
String jsonMeasurement(const Measurement& m, uint8_t decimals) {
  return m.ok ? String(m.value, static_cast<unsigned int>(decimals)) : String("null");
}
// Pump contact SPDT4: observed 1 with pump ON, 0 with pump OFF.
// NOT a direct measurement of hydraulic flow (SPDT1).
FlowReading readPump() {
  FlowReading f = {false, false, 0, 0};
  readRegisters(3, 1007, 1);
  f.result = lastResult;
  if (f.result != 0) return f;
  f.raw = modbus.getResponseBuffer(0);
  if (f.raw == 0 || f.raw == 1) {
    f.ok = true;
    f.present = (f.raw == 1);
  }
  return f;
}
String jsonPump(const FlowReading& f) {
  return f.ok ? String(f.present ? "true" : "false") : String("null");
}
void handleMeasurements() {
  Measurement ph = readFloatAt(1202);
  Measurement phSetpoint = readFloatAt(1204);
  Measurement redox = readFloatAt(1302);
  Measurement redoxSetpoint = readFloatAt(1304);
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
               ",\"flow\":null,\"flow_verified\":false" +
               ",\"pump\":" + jsonPump(pump) +
               ",\"pump_register\":1007" +
               ",\"pump_raw\":" + (pump.result == 0 ? String(pump.raw) : String("null")) +
               ",\"pump_modbus_code\":" + String(pump.result) +
               ",\"unit_redox\":\"mV\"" +
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
               ",\"redox_mv\":" + jsonMeasurement(value, 2) +
               ",\"redox_setpoint\":" + jsonMeasurement(setpoint, 2) +
               ",\"unit\":\"mV\"" +
               ",\"modbus_codes\":{\"redox\":" + String(value.result) +
               ",\"redox_setpoint\":" + String(setpoint.result) + "}}";
  sendJson(ok ? 200 : 502, out);
}
// Hydraulic flow is wired to SPDT1; its Modbus register is unknown.
// Returning null prevents the pump contact from being mistaken for real flow.
void handleFlow() {
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"ok\":false,\"flow\":null,\"flow_verified\":false," +
               "\"flow_register\":null,\"note\":\"SPDT1 hydraulic flow register not identified\"}";
  sendJson(200, out);
}
// Pump dry contact on SPDT4, read from holding register 1007.
void handlePump() {
  FlowReading pump = readPump();
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"ok\":" + String(pump.ok ? "true" : "false") +
               ",\"pump\":" + jsonPump(pump) +
               ",\"pump_register\":1007" +
               ",\"pump_raw\":" + (pump.result == 0 ? String(pump.raw) : String("null")) +
               ",\"modbus_code\":" + String(pump.result) + "}";
  sendJson(pump.ok ? 200 : 502, out);
}
void handleStatus() {
  // Holding register 1100: injection activation indicator (user identified).
  // Read on demand; never interpret a Modbus failure as injection disabled.
  readRegisters(3, 1100, 1);
  const uint8_t injectionCode = lastResult;
  const bool injectionReadOk = (injectionCode == 0);
  const uint16_t injectionRaw = injectionReadOk ? modbus.getResponseBuffer(0) : 0;
  const String injectionJson = injectionReadOk ? String(injectionRaw) : String("null");
  String out = "{\"version\":" + quoted(FIRMWARE_VERSION) +
               ",\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") +
               ",\"ip\":" + quoted(WiFi.localIP().toString()) +
               ",\"rssi_dbm\":" + String(WiFi.RSSI()) +
               ",\"uptime_ms\":" + String(millis()) +
               ",\"slave\":" + String(slaveId) +
               ",\"injection_active\":" + (injectionReadOk && injectionRaw <= 1 ? String(injectionRaw == 1 ? "true" : "false") : String("null")) +
               ",\"injection_register\":1100" +
               ",\"injection_raw\":" + injectionJson +
               ",\"injection_modbus_code\":" + String(injectionCode) +
               ",\"baud\":" + String(baudRate) +
               ",\"parity\":" + quoted(parityMode == 0 ? "N" : (parityMode == 1 ? "E" : "O")) +
               ",\"requests\":" + String(requests) +
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
  slaveId = newSlave; baudRate = newBaud; parityMode = newParity;
  initModbus();
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
    requests++;lastOperation="scan function=3 address="+String(base)+" count="+String(n);
    uint32_t t=millis();uint8_t r=modbus.readHoldingRegisters((uint16_t)base,n);
    lastDuration=millis()-t;lastResult=r;
    if (r!=0) {failed++; if(r==0xE2)timeouts++;else errors++;continue;}
    successes++;successful++;
    for (uint8_t i=0;i<n;i++) {
      uint16_t v=modbus.getResponseBuffer(i), w=swap16(v);
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
        uint16_t b=modbus.getResponseBuffer(i+1);
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
    "AstralPool RS485 API v1.3.5 (read-only)\n"
    "GET /api/status\n"
    "GET /api/measurements (pH and redox)\n"
    "GET /api/flow (hydraulic SPDT1: unknown, returns null)\n"
    "GET /api/pump (SPDT4: register 1007)\n"
    "GET /api/scan?start=0&count=100&ph=7.41&redox=751\n"
    "GET /api/read?function=4&address=0x51&count=1\n"
    "POST /api/config?slave=2&baud=9600&parity=N\n"
    "No authentication: use only on a trusted LAN.\n");
}
void setup() {
  Serial.begin(115200);
  initModbus();
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300); Serial.print('.');
  }
  Serial.println();
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
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
}
void loop() {
  server.handleClient();
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastRetry = 0;
    if (millis() - lastRetry > 10000) { lastRetry = millis(); WiFi.reconnect(); }
  }
}
