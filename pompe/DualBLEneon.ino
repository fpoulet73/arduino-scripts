#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ESP32-C3, NimBLE-Arduino 2.x. HTTP never waits for BLE scanning or writes.
// API returns 202 (queued), NOT a confirmation that the lamp changed state.
static const char* FIRMWARE_VERSION = "1.5.0";
static const char* WIFI_SSID = "FUN5";
static const char* WIFI_PASSWORD = "atout.be";
static const char* API_KEY = "CHANGE_ME_TO_A_LONG_RANDOM_KEY";
static const char* TARGET_MACS[2] = {"be:16:57:00:36:9a", "be:16:57:00:31:4d"};
static const NimBLEUUID SERVICE_UUID("fff0");
static const NimBLEUUID WRITE_UUID("fff3");
static const uint32_t RETRY_MS = 6000;

WebServer server(80);
QueueHandle_t commandQueue;
SemaphoreHandle_t stateMutex;

enum CommandType : uint8_t { CMD_ON, CMD_OFF, CMD_BRIGHTNESS };
struct LedCommand { uint32_t id; CommandType type; uint8_t brightness; };
struct BleState {
  bool connected = false;
  int activeIndex = -1;
  int rssi[2] = {-127, -127};
  uint32_t scans = 0, attempts = 0, connections = 0, failovers = 0;
  uint32_t sent = 0, failed = 0, lastWriteMs = 0, lastCommandId = 0;
  bool lastCommandOk = false;
  char phase[24] = "starting";
  char error[100] = "";
} state;
uint32_t nextCommandId = 0;

void setPhase(const char* phase, const char* error = nullptr) {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  snprintf(state.phase, sizeof(state.phase), "%s", phase);
  if (error) snprintf(state.error, sizeof(state.error), "%s", error);
  xSemaphoreGive(stateMutex);
}
void sendJson(int code, const String& body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json; charset=utf-8", body);
}
bool authorized() {
  if (server.hasHeader("X-API-Key") && server.header("X-API-Key") == API_KEY) return true;
  sendJson(401, "{\"error\":\"unauthorized\"}");
  return false;
}
String jsonEscape(const char* s) {
  String out;
  while (*s) {
    char c = *s++;
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if ((uint8_t)c >= 32) out += c;
  }
  return out;
}
void statusHandler() {
  if (!authorized()) return;
  BleState snap;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  snap = state;
  xSemaphoreGive(stateMutex);
  String json = "{\"version\":\"" + String(FIRMWARE_VERSION) + "\"";
  json += ",\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  json += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += ",\"bleConnected\":" + String(snap.connected ? "true" : "false");
  json += ",\"controlReady\":" + String(snap.connected ? "true" : "false");
  json += ",\"bleState\":\"" + jsonEscape(snap.phase) + "\"";
  json += ",\"activeMac\":\"" + String(snap.activeIndex >= 0 ? TARGET_MACS[snap.activeIndex] : "") + "\"";
  json += ",\"mac1Rssi\":" + String(snap.rssi[0]);
  json += ",\"mac2Rssi\":" + String(snap.rssi[1]);
  json += ",\"scanCount\":" + String(snap.scans);
  json += ",\"connectionAttempts\":" + String(snap.attempts);
  json += ",\"successfulConnections\":" + String(snap.connections);
  json += ",\"failovers\":" + String(snap.failovers);
  json += ",\"successfulWrites\":" + String(snap.sent);
  json += ",\"failedWrites\":" + String(snap.failed);
  json += ",\"lastWriteMs\":" + String(snap.lastWriteMs);
  json += ",\"lastCommandId\":" + String(snap.lastCommandId);
  json += ",\"lastCommandOk\":" + String(snap.lastCommandOk ? "true" : "false");
  json += ",\"queued\":" + String(uxQueueMessagesWaiting(commandQueue));
  json += ",\"lastError\":\"" + jsonEscape(snap.error) + "\"}";
  sendJson(200, json);
}
void enqueueCommand(CommandType type) {
  if (!authorized()) return;
  uint8_t brightness = 0;
  if (type == CMD_BRIGHTNESS) {
    if (!server.hasArg("value")) { sendJson(400, "{\"error\":\"missing value 1..100\"}"); return; }
    String value = server.arg("value");
    if (!value.length()) { sendJson(400, "{\"error\":\"value must be 1..100\"}"); return; }
    for (size_t i = 0; i < value.length(); ++i) {
      if (value[i] < '0' || value[i] > '9') { sendJson(400, "{\"error\":\"invalid brightness\"}"); return; }
    }
    int v = value.toInt();
    if (v < 1 || v > 100) { sendJson(400, "{\"error\":\"value must be 1..100\"}"); return; }
    brightness = (uint8_t)v;
  }
  LedCommand cmd = {++nextCommandId, type, brightness};
  if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
    sendJson(503, "{\"error\":\"BLE command queue full\"}"); return;
  }
  String body = "{\"queued\":true,\"commandId\":" + String(cmd.id);
  body += ",\"command\":\"" + String(type == CMD_ON ? "on" : type == CMD_OFF ? "off" : "brightness") + "\"";
  if (type == CMD_BRIGHTNESS) body += ",\"brightness\":" + String(brightness);
  body += "}";
  sendJson(202, body);
}

// Only this FreeRTOS task owns the BLE client. HTTP handlers only enqueue.
void bleWorker(void*) {
  NimBLEClient* client = nullptr;
  NimBLERemoteCharacteristic* writer = nullptr;
  uint32_t lastAttempt = 0;
  int previousIndex = -1;
  for (;;) {
    if (client && !client->isConnected()) {
      writer = nullptr;
      NimBLEDevice::deleteClient(client);
      client = nullptr;
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      state.connected = false;
      state.activeIndex = -1;
      xSemaphoreGive(stateMutex);
      setPhase("disconnected", "peripheral disconnected");
      lastAttempt = millis();
    }
    if (!client && (lastAttempt == 0 || millis() - lastAttempt >= RETRY_MS)) {
      lastAttempt = millis();
      setPhase("scanning", "");
      NimBLEScan* scan = NimBLEDevice::getScan();
      scan->setActiveScan(true);
      NimBLEScanResults results = scan->getResults(5000, false);
      const NimBLEAdvertisedDevice* found[2] = {nullptr, nullptr};
      int rssi[2] = {-127, -127};
      for (int i = 0; i < results.getCount(); ++i) {
        const NimBLEAdvertisedDevice* dev = results.getDevice(i);
        if (!dev) continue;
        String mac = String(dev->getAddress().toString().c_str());
        for (int j = 0; j < 2; ++j) {
          if (mac.equalsIgnoreCase(TARGET_MACS[j]) && (!found[j] || dev->getRSSI() > rssi[j])) {
            found[j] = dev;
            rssi[j] = dev->getRSSI();
          }
        }
      }
      scan->stop();
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      ++state.scans;
      state.rssi[0] = rssi[0]; state.rssi[1] = rssi[1];
      xSemaphoreGive(stateMutex);
      int first = (rssi[1] > rssi[0]) ? 1 : 0;
      int order[2] = {first, 1 - first};
      bool connected = false;
      for (int n = 0; n < 2 && !connected; ++n) {
        int index = order[n];
        if (!found[index]) continue;
        setPhase("connecting");
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        ++state.attempts;
        if (n == 1) ++state.failovers;
        xSemaphoreGive(stateMutex);
        client = NimBLEDevice::createClient();
        if (!client) break;
        client->setConnectTimeout(12);
        bool ok = client->connect(found[index], false);
        if (ok) {
          NimBLERemoteService* svc = client->getService(SERVICE_UUID);
          writer = svc ? svc->getCharacteristic(WRITE_UUID) : nullptr;
          ok = writer && writer->canWriteNoResponse();
        }
        if (!ok) {
          writer = nullptr;
          if (client->isConnected()) client->disconnect();
          NimBLEDevice::deleteClient(client);
          client = nullptr;
          continue;
        }
        connected = true;
        previousIndex = index;
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        state.connected = true;
        state.activeIndex = index;
        ++state.connections;
        xSemaphoreGive(stateMutex);
        setPhase("connected", "");
        Serial.printf("[BLE] connected %s RSSI=%d\n", TARGET_MACS[index], rssi[index]);
      }
      scan->clearResults();
      if (!connected) setPhase("waiting_retry", "connection failed on available controllers");
    }
    LedCommand cmd;
    if (client && writer && client->isConnected() && xQueueReceive(commandQueue, &cmd, pdMS_TO_TICKS(30)) == pdTRUE) {
      uint8_t packet[9] = {0x7E, 0x04, 0x04, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xEF};
      if (cmd.type == CMD_ON) packet[3] = 0xF0;
      else if (cmd.type == CMD_BRIGHTNESS) { packet[2] = 0x01; packet[3] = cmd.brightness; packet[4] = 0x01; }
      uint32_t start = millis();
      bool ok = writer->writeValue(packet, sizeof(packet), false);
      uint32_t elapsed = millis() - start;
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      state.lastWriteMs = elapsed;
      state.lastCommandId = cmd.id;
      state.lastCommandOk = ok;
      if (ok) ++state.sent; else ++state.failed;
      xSemaphoreGive(stateMutex);
      Serial.printf("[BLE] command=%lu ok=%d writeMs=%lu mac=%s\n", (unsigned long)cmd.id, ok, (unsigned long)elapsed, TARGET_MACS[previousIndex]);
      if (!ok) {
        setPhase("write_failed", "FFF3 write failed");
        writer = nullptr;
        if (client->isConnected()) client->disconnect();
        NimBLEDevice::deleteClient(client);
        client = nullptr;
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        state.connected = false;
        state.activeIndex = -1;
        xSemaphoreGive(stateMutex);
        lastAttempt = millis();
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(350);
  commandQueue = xQueueCreate(8, sizeof(LedCommand));
  stateMutex = xSemaphoreCreateMutex();
  if (!commandQueue || !stateMutex) { Serial.println("FATAL: FreeRTOS allocation failed"); while (true) delay(1000); }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  NimBLEDevice::init("JeedomDualBLE");
  const char* headers[] = {"X-API-Key"};
  server.collectHeaders(headers, 1);
  server.on("/api/status", HTTP_GET, statusHandler);
  server.on("/api/led/on", HTTP_POST, [](){ enqueueCommand(CMD_ON); });
  server.on("/api/led/off", HTTP_POST, [](){ enqueueCommand(CMD_OFF); });
  server.on("/api/led/brightness", HTTP_POST, [](){ enqueueCommand(CMD_BRIGHTNESS); });
  server.onNotFound([](){ sendJson(404, "{\"error\":\"not found\"}"); });
  server.begin();
  xTaskCreate(bleWorker, "ble-worker", 8192, nullptr, 1, nullptr);
  Serial.printf("[HTTP] firmware %s ready\n", FIRMWARE_VERSION);
}
void loop() {
  server.handleClient();
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastWifiRetry = 0;
    if (millis() - lastWifiRetry > 10000) { WiFi.reconnect(); lastWifiRetry = millis(); }
  }
  delay(2);
}
