#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NimBLEDevice.h>
#include <Preferences.h>

// ESP32-C3 Super Mini: install ESP32 boards (Espressif) and NimBLE-Arduino 2.x.
// The BLE command protocol is NOT known yet: this firmware discovers devices
// and GATT services but deliberately does not send unverified LED commands.
static const char* WIFI_SSID = "FUN5";
static const char* WIFI_PASSWORD = "atout.be";
static const char* DEVICE_NAME = "MELK#OC21";
static const char* API_KEY = "CHANGE_ME_TO_A_LONG_RANDOM_KEY";

WebServer server(80);
Preferences prefs;
struct LedDevice {
  String mac;
  String name;
  int rssi = -127;
  bool seen = false;
};
LedDevice leds[2];
String lastScanError;
unsigned long lastScanMs = 0;

String jsonEscape(const String &s) {
  String out;
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if ((uint8_t)c >= 32) out += c;
  }
  return out;
}
void reply(int status, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(status, "application/json; charset=utf-8", body);
}
bool authorized() {
  if (server.hasHeader("X-API-Key") && server.header("X-API-Key") == API_KEY) return true;
  if (server.hasArg("key") && server.arg("key") == API_KEY) return true; // Jeedom HTTP plugin fallback; avoid exposing URL in logs
  reply(401, "{\"error\":\"unauthorized\"}");
  return false;
}
String deviceJson(int i) {
  return "{\"id\":" + String(i + 1) + ",\"mac\":\"" + jsonEscape(leds[i].mac) +
    "\",\"name\":\"" + jsonEscape(leds[i].name) + "\",\"seen\":" +
    String(leds[i].seen ? "true" : "false") + ",\"rssi\":" + String(leds[i].rssi) + "}";
}
void sendDevices() {
  if (!authorized()) return;
  reply(200, "{\"devices\":[" + deviceJson(0) + "," + deviceJson(1) + "]}");
}

// Discovery is intentionally manual: an active scan can delay the HTTP response.
void scanDevices() {
  if (!authorized()) return;
  for (auto &led : leds) { led.seen = false; led.rssi = -127; }
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  NimBLEScanResults results = scan->getResults(5000, false);
  int matches = 0;
  for (int i = 0; i < results.getCount(); ++i) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    if (!dev || !dev->haveName()) continue;
    String name = String(dev->getName().c_str());
    if (name != DEVICE_NAME) continue;
    String mac = String(dev->getAddress().toString().c_str());
    mac.toLowerCase();
    ++matches;
    int slot = -1;
    for (int j = 0; j < 2; ++j) if (leds[j].mac == mac) slot = j;
    if (slot < 0) for (int j = 0; j < 2; ++j) if (leds[j].mac.isEmpty()) { slot = j; break; }
    if (slot < 0) continue;
    leds[slot].mac = mac;
    leds[slot].name = name;
    leds[slot].seen = true;
    leds[slot].rssi = dev->getRSSI();
    prefs.putString((String("mac") + String(slot)).c_str(), mac);
  }
  scan->clearResults();
  lastScanMs = millis();
  reply(200, "{\"matchedAdvertisements\":" + String(matches) + ",\"devices\":[" + deviceJson(0) + "," + deviceJson(1) + "]}");
}

void inspectGatt() {
  if (!authorized()) return;
  if (!server.hasArg("id")) { reply(400, "{\"error\":\"missing id=1 or id=2\"}"); return; }
  int id = server.arg("id").toInt();
  if (id < 1 || id > 2 || leds[id - 1].mac.isEmpty()) {
    reply(404, "{\"error\":\"unknown device; run /api/scan first\"}"); return;
  }
  NimBLEClient* client = NimBLEDevice::createClient();
  client->setConnectTimeout(8);
  NimBLEAddress address(leds[id - 1].mac.c_str(), BLE_ADDR_PUBLIC);
  if (!client->connect(address, true)) {
    NimBLEDevice::deleteClient(client);
    reply(502, "{\"error\":\"BLE connection failed (device may use random address)\"}"); return;
  }
  String result = "{\"id\":" + String(id) + ",\"mac\":\"" + jsonEscape(leds[id - 1].mac) + "\",\"services\":[";
  bool firstService = true;
  for (auto* svc : client->getServices(true)) {
    if (!firstService) result += ",";
    firstService = false;
    result += "{\"uuid\":\"" + String(svc->getUUID().toString().c_str()) + "\",\"characteristics\":[";
    bool firstChar = true;
    for (auto* chr : svc->getCharacteristics(true)) {
      if (!firstChar) result += ",";
      firstChar = false;
      result += "{\"uuid\":\"" + String(chr->getUUID().toString().c_str()) +
        "\",\"read\":" + String(chr->canRead() ? "true" : "false") +
        ",\"write\":" + String(chr->canWrite() ? "true" : "false") +
        ",\"writeNoResponse\":" + String(chr->canWriteNoResponse() ? "true" : "false") +
        ",\"notify\":" + String(chr->canNotify() ? "true" : "false") + "}";
    }
    result += "]}";
  }
  result += "]}";
  client->disconnect();
  NimBLEDevice::deleteClient(client);
  reply(200, result);
}
void notImplemented() {
  if (!authorized()) return;
  reply(501, "{\"error\":\"BLE command protocol not yet identified; no command sent\",\"next\":\"GET /api/gatt?id=1 and id=2\"}");
}
void setup() {
  Serial.begin(115200);
  delay(500);
  prefs.begin("dual-led", false);
  for (int i = 0; i < 2; ++i) {
    leds[i].mac = prefs.getString((String("mac") + String(i)).c_str(), "");
    leds[i].name = DEVICE_NAME;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; ++i) { delay(500); Serial.print('.'); }
  Serial.println();
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  NimBLEDevice::init("JeedomDualBLE");
  const char* headers[] = {"X-API-Key"};
  server.collectHeaders(headers, 1);
  server.on("/api/status", HTTP_GET, []() {
    if (!authorized()) return;
    reply(200, "{\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") +
      ",\"ip\":\"" + WiFi.localIP().toString() + "\",\"ble\":\"initialized\",\"controlReady\":false}");
  });
  server.on("/api/devices", HTTP_GET, sendDevices);
  server.on("/api/scan", HTTP_GET, scanDevices);
  server.on("/api/gatt", HTTP_GET, inspectGatt);
  server.on("/api/led/1/on", HTTP_POST, notImplemented);
  server.on("/api/led/1/off", HTTP_POST, notImplemented);
  server.on("/api/led/2/on", HTTP_POST, notImplemented);
  server.on("/api/led/2/off", HTTP_POST, notImplemented);
  server.on("/api/led/1/brightness", HTTP_POST, notImplemented);
  server.on("/api/led/2/brightness", HTTP_POST, notImplemented);
  server.onNotFound([]() { reply(404, "{\"error\":\"not found\"}"); });
  server.begin();
  Serial.println("REST server ready on port 80");
}
void loop() {
  server.handleClient();
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 10000) { WiFi.reconnect(); lastAttempt = millis(); }
  }
  delay(2);
}
