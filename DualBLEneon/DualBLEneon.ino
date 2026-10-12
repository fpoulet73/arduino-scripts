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
static const char* FIRMWARE_VERSION = "1.8.0";
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

enum CommandType : uint8_t { CMD_ON, CMD_OFF, CMD_BRIGHTNESS, CMD_COLOR, CMD_MODE, CMD_SPEED };
struct LedCommand { uint32_t id; CommandType type; uint8_t value; uint8_t r, g, b; };
struct ControllerState {
  bool connected = false;
  int rssi = -127;
  uint32_t attempts = 0, connections = 0, sent = 0, failed = 0;
  uint32_t lastWriteMs = 0, lastCommandId = 0;
  uint32_t lastConnectedMs = 0;
  uint8_t consecutiveWriteFailures = 0;
  bool lastWriteOk = false;
  char phase[24] = "starting";
};
struct BleState {
  ControllerState controllers[2];
  uint32_t scans = 0, lastCommandId = 0;
  uint32_t scanFailures = 0;
  bool lastCommandOk = false;
  char lastPacketHex[28] = "";
  char error[100] = "";
} state;
uint32_t nextCommandId = 0;

struct BleLink {
  NimBLEClient* client = nullptr;
  NimBLERemoteCharacteristic* writer = nullptr;
  uint32_t nextRetryMs = 0;
  uint8_t consecutiveWriteFailures = 0;
};

void setControllerPhase(int i, const char* phase) {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  snprintf(state.controllers[i].phase, sizeof(state.controllers[i].phase), "%s", phase);
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
  bool any = snap.controllers[0].connected || snap.controllers[1].connected;
  bool both = snap.controllers[0].connected && snap.controllers[1].connected;
  json += ",\"bleConnected\":" + String(any ? "true" : "false");
  json += ",\"controlReady\":" + String(both ? "true" : "false");
  json += ",\"scanCount\":" + String(snap.scans);
  json += ",\"scanFailures\":" + String(snap.scanFailures);
  json += ",\"lastCommandId\":" + String(snap.lastCommandId);
  json += ",\"lastCommandOk\":" + String(snap.lastCommandOk ? "true" : "false");
  json += ",\"lastPacketHex\":\"" + String(snap.lastPacketHex) + "\"";
  json += ",\"queued\":" + String(uxQueueMessagesWaiting(commandQueue));
  json += ",\"controllers\":[";
  for (int i = 0; i < 2; ++i) {
    if (i) json += ",";
    const ControllerState& c = snap.controllers[i];
    json += "{\"mac\":\"" + String(TARGET_MACS[i]) + "\"";
    json += ",\"connected\":" + String(c.connected ? "true" : "false");
    json += ",\"rssi\":" + String(c.rssi);
    json += ",\"state\":\"" + jsonEscape(c.phase) + "\"";
    json += ",\"connectionAttempts\":" + String(c.attempts);
    json += ",\"successfulConnections\":" + String(c.connections);
    json += ",\"successfulWrites\":" + String(c.sent);
    json += ",\"failedWrites\":" + String(c.failed);
    json += ",\"lastWriteMs\":" + String(c.lastWriteMs);
    json += ",\"lastConnectedMs\":" + String(c.lastConnectedMs);
    json += ",\"consecutiveWriteFailures\":" + String(c.consecutiveWriteFailures);
    json += ",\"lastCommandId\":" + String(c.lastCommandId);
    json += ",\"lastWriteOk\":" + String(c.lastWriteOk ? "true" : "false");
    json += "}";
  }
  json += "]}";
  sendJson(200, json);
}
bool parseNumber(const String& text, int minValue, int maxValue, uint8_t& output) {
  if (text.length() == 0 || text.length() > 3) return false;
  for (size_t i = 0; i < text.length(); ++i)
    if (text[i] < '0' || text[i] > '9') return false;
  int n = text.toInt();
  if (n < minValue || n > maxValue) return false;
  output = (uint8_t)n;
  return true;
}
int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
bool parseHexColor(String text, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (text.startsWith("#")) text.remove(0, 1);
  if (text.length() != 6) return false;
  uint8_t bytes[3];
  for (int i = 0; i < 3; ++i) {
    int hi = hexDigit(text[2 * i]);
    int lo = hexDigit(text[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    bytes[i] = (uint8_t)((hi << 4) | lo);
  }
  r = bytes[0]; g = bytes[1]; b = bytes[2];
  return true;
}
void enqueueCommand(CommandType type) {
  if (!authorized()) return;
  LedCommand cmd = {};
  cmd.type = type;
  if (type == CMD_BRIGHTNESS || type == CMD_MODE || type == CMD_SPEED) {
    int low = type == CMD_BRIGHTNESS ? 1 : type == CMD_MODE ? 135 : 1;
    int high = type == CMD_MODE ? 156 : 100;
    if (!server.hasArg("value") || !parseNumber(server.arg("value"), low, high, cmd.value)) {
      sendJson(400, "{\"error\":\"invalid value; brightness 1..100, mode 135..156, speed 1..100\"}");
      return;
    }
  }
  if (type == CMD_COLOR) {
    bool valid = false;
    if (server.hasArg("hex")) {
      valid = parseHexColor(server.arg("hex"), cmd.r, cmd.g, cmd.b);
    } else if (server.hasArg("r") && server.hasArg("g") && server.hasArg("b")) {
      valid = parseNumber(server.arg("r"), 0, 255, cmd.r) &&
              parseNumber(server.arg("g"), 0, 255, cmd.g) &&
              parseNumber(server.arg("b"), 0, 255, cmd.b);
    }
    if (!valid) { sendJson(400, "{\"error\":\"use hex=RRGGBB or r,g,b=0..255\"}"); return; }
  }
  cmd.id = ++nextCommandId;
  if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
    sendJson(503, "{\"error\":\"BLE command queue full\"}"); return;
  }
  const char* names[] = {"on", "off", "brightness", "color", "mode", "speed"};
  String body = "{\"queued\":true,\"commandId\":" + String(cmd.id);
  body += ",\"command\":\"" + String(names[type]) + "\"";
  if (type == CMD_BRIGHTNESS || type == CMD_MODE || type == CMD_SPEED) body += ",\"value\":" + String(cmd.value);
  if (type == CMD_COLOR) {
    body += ",\"r\":" + String(cmd.r) + ",\"g\":" + String(cmd.g) + ",\"b\":" + String(cmd.b);
  }
  body += "}";
  sendJson(202, body);
}

// The worker alone owns both BLE clients. Commands are broadcast sequentially.
void dropLink(BleLink& link, int i) {
  link.writer = nullptr;
  if (link.client) {
    if (link.client->isConnected()) link.client->disconnect();
    NimBLEDevice::deleteClient(link.client);
    link.client = nullptr;
  }
  link.nextRetryMs = millis() + RETRY_MS;
  link.consecutiveWriteFailures = 0;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  state.controllers[i].connected = false;
  xSemaphoreGive(stateMutex);
  setControllerPhase(i, "disconnected");
}
void bleWorker(void*) {
  BleLink links[2];
  for (;;) {
    // Reconnect only missing controllers, independently of HTTP requests.
    bool needsScan = false;
    for (int i = 0; i < 2; ++i) {
      if (links[i].client && !links[i].client->isConnected()) dropLink(links[i], i);
      if (!links[i].client && (int32_t)(millis() - links[i].nextRetryMs) >= 0) needsScan = true;
    }
    if (needsScan) {
      NimBLEScan* scan = NimBLEDevice::getScan();
      scan->setActiveScan(true);
      NimBLEScanResults results = scan->getResults(3000, false);
      const NimBLEAdvertisedDevice* found[2] = {nullptr, nullptr};
      int rssi[2] = {-127, -127};
      for (int j = 0; j < results.getCount(); ++j) {
        const NimBLEAdvertisedDevice* dev = results.getDevice(j);
        if (!dev) continue;
        String mac = String(dev->getAddress().toString().c_str());
        for (int i = 0; i < 2; ++i) {
          if (mac.equalsIgnoreCase(TARGET_MACS[i]) && (!found[i] || dev->getRSSI() > rssi[i])) {
            found[i] = dev;
            rssi[i] = dev->getRSSI();
          }
        }
      }
      scan->stop();
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      ++state.scans;
      for (int i = 0; i < 2; ++i) if (found[i]) state.controllers[i].rssi = rssi[i];
      xSemaphoreGive(stateMutex);
      // Do not clear scan results until both connection attempts finish.
      for (int i = 0; i < 2; ++i) {
        if (links[i].client || (int32_t)(millis() - links[i].nextRetryMs) < 0) continue;
        if (!found[i]) {
          setControllerPhase(i, "not_found");
          links[i].nextRetryMs = millis() + RETRY_MS;
          continue;
        }
        setControllerPhase(i, "connecting");
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        ++state.controllers[i].attempts;
        xSemaphoreGive(stateMutex);
        NimBLEClient* client = NimBLEDevice::createClient();
        if (!client) {
          setControllerPhase(i, "no_client");
          links[i].nextRetryMs = millis() + RETRY_MS;
          continue;
        }
        client->setConnectTimeout(12);
        bool ok = client->connect(found[i], false);
        NimBLERemoteCharacteristic* writer = nullptr;
        if (ok) {
          NimBLERemoteService* svc = client->getService(SERVICE_UUID);
          writer = svc ? svc->getCharacteristic(WRITE_UUID) : nullptr;
          ok = writer && writer->canWriteNoResponse();
        }
        if (!ok) {
          if (client->isConnected()) client->disconnect();
          NimBLEDevice::deleteClient(client);
          setControllerPhase(i, "connect_failed");
          links[i].nextRetryMs = millis() + RETRY_MS;
          continue;
        }
        links[i].client = client;
        links[i].writer = writer;
        links[i].consecutiveWriteFailures = 0;
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        state.controllers[i].connected = true;
        ++state.controllers[i].connections;
        state.controllers[i].lastConnectedMs = millis();
        state.controllers[i].consecutiveWriteFailures = 0;
        xSemaphoreGive(stateMutex);
        setControllerPhase(i, "connected");
        Serial.printf("[BLE] connected %s RSSI=%d\n", TARGET_MACS[i], rssi[i]);
      }
      scan->clearResults();
    }
    LedCommand cmd;
    if (xQueueReceive(commandQueue, &cmd, pdMS_TO_TICKS(30)) == pdTRUE) {
      uint8_t packet[9] = {0x7E, 0x04, 0x04, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xEF};
      size_t packetLen = sizeof(packet);
      if (cmd.type == CMD_ON) packet[3] = 0xF0;
      else if (cmd.type == CMD_BRIGHTNESS) {
        packet[2] = 0x01; packet[3] = cmd.value; packet[4] = 0x01;
      } else if (cmd.type == CMD_COLOR) {
        // Verified LotusLamp MELK RGB frame is NINE bytes; 0x03 is mandatory.
        const uint8_t rgb[9] = {0x7E, 0x07, 0x05, 0x03, cmd.r, cmd.g, cmd.b, 0x10, 0xEF};
        memcpy(packet, rgb, sizeof(rgb));
        packetLen = sizeof(rgb);
      } else if (cmd.type == CMD_MODE) {
        const uint8_t effect[9] = {0x7E, 0x07, 0x03, cmd.value, 0x03, 0xFF, 0xFF, 0x00, 0xEF};
        memcpy(packet, effect, sizeof(packet));
      } else if (cmd.type == CMD_SPEED) {
        const uint8_t speed[9] = {0x7E, 0x07, 0x02, cmd.value, 0xFF, 0xFF, 0xFF, 0x00, 0xEF};
        memcpy(packet, speed, sizeof(packet));
      }
      char packetHex[28] = {};
      for (size_t b = 0; b < packetLen; ++b)
        snprintf(packetHex + b * 2, sizeof(packetHex) - b * 2, "%02X", packet[b]);
      bool allOk = true;
      for (int i = 0; i < 2; ++i) {
        bool ok = false;
        uint32_t elapsed = 0;
        if (links[i].client && links[i].writer && links[i].client->isConnected()) {
          uint32_t start = millis();
          ok = links[i].writer->writeValue(packet, packetLen, false);
          elapsed = millis() - start;
        }
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        ControllerState& c = state.controllers[i];
        c.lastWriteMs = elapsed;
        c.lastCommandId = cmd.id;
        c.lastWriteOk = ok;
        if (ok) { ++c.sent; links[i].consecutiveWriteFailures = 0; }
        else { ++c.failed; if (links[i].consecutiveWriteFailures < 255) ++links[i].consecutiveWriteFailures; }
        c.consecutiveWriteFailures = links[i].consecutiveWriteFailures;
        xSemaphoreGive(stateMutex);
        Serial.printf("[BLE] cmd=%lu mac=%s ok=%d ms=%lu packet=%s\n",
                      (unsigned long)cmd.id, TARGET_MACS[i], ok, (unsigned long)elapsed, packetHex);
        if (!ok) {
          allOk = false;
          // A write-without-response failure is not proof of disconnection.
          // Keep both BLE links alive unless NimBLE reports an actual disconnect.
          if (links[i].client && !links[i].client->isConnected()) dropLink(links[i], i);
        }
      }
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      state.lastCommandId = cmd.id;
      state.lastCommandOk = allOk;
      snprintf(state.lastPacketHex, sizeof(state.lastPacketHex), "%s", packetHex);
      xSemaphoreGive(stateMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
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
  NimBLEDevice::setPower(9); // ESP32-C3 BLE TX power (dBm)
  const char* headers[] = {"X-API-Key"};
  server.collectHeaders(headers, 1);
  server.on("/api/status", HTTP_GET, statusHandler);
  server.on("/api/led/on", HTTP_POST, [](){ enqueueCommand(CMD_ON); });
  server.on("/api/led/off", HTTP_POST, [](){ enqueueCommand(CMD_OFF); });
  server.on("/api/led/brightness", HTTP_POST, [](){ enqueueCommand(CMD_BRIGHTNESS); });
  server.on("/api/led/color", HTTP_POST, [](){ enqueueCommand(CMD_COLOR); });
  server.on("/api/led/mode", HTTP_POST, [](){ enqueueCommand(CMD_MODE); });
  server.on("/api/led/speed", HTTP_POST, [](){ enqueueCommand(CMD_SPEED); });
  server.onNotFound([](){ sendJson(404, "{\"error\":\"not found\"}"); });
  server.begin();
  xTaskCreate(bleWorker, "ble-worker", 10240, nullptr, 1, nullptr);
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
