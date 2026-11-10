/*
  SwarmLink ESP-NOW telemetry reference

  This sketch shares the compact swarmlink.telemetry.compact.v1 JSON envelope
  between nearby ESP32 boards. The Python package expands it into the same
  NodeTelemetry object used by the normal, readable envelope. Every received
  valid-size frame is printed unchanged to USB serial, allowing one receiver
  connected to a laptop/Raspberry Pi to act as a base-station bridge.

  It is deliberately telemetry-only. It neither controls motors nor sends
  flight-controller setpoints. Replace readYourSensors() with your own GPS,
  UWB, IMU, or wheel-odometry integration.

  Board: ESP32 family with Arduino-ESP32 core 2.x or 3.x.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <cstring>
#include <time.h>

// All nodes using ESP-NOW must use the same Wi-Fi channel. Channel 0 asks the
// radio to keep its current channel; set a fixed value if your deployment does.
static const uint8_t ESPNOW_CHANNEL = 0;
static const uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint32_t BAUDRATE = 115200;
static const uint32_t PUBLISH_INTERVAL_MS = 250;
static const size_t MAX_PACKET_BYTES = 240;  // ESP-NOW v1 payload limit is 250 bytes.

// Keep names concise so the full JSON packet always fits the ESP-NOW frame.
static const char *NODE_ID = "rover-01";
static const char *VEHICLE_KIND = "rover";  // drone, rover, sensor, boat, or unknown
static bool espNowReady = false;

struct TelemetrySample {
  // Local ENU coordinate frame: x=east, y=north, z=up (metres).
  float x_m;
  float y_m;
  float z_m;
  float vx_mps;
  float vy_mps;
  float vz_mps;
  float battery_percent;
  const char *mission_state;
};

/*
  Replace this with sensor code. The default deliberately stays a static bench
  test so the radio path can be checked before GPS/UWB hardware is connected.
*/
TelemetrySample readYourSensors() {
  TelemetrySample sample;
  sample.x_m = 0.0f;
  sample.y_m = 0.0f;
  sample.z_m = 0.0f;
  sample.vx_mps = 0.0f;
  sample.vy_mps = 0.0f;
  sample.vz_mps = 0.0f;
  sample.battery_percent = 100.0f;
  sample.mission_state = "bench-test";
  return sample;
}

// Return real Unix UTC seconds. For actual coordination, synchronize this from
// GPS or NTP in your own setup before sending. A value of zero is fine for a
// packet-format bench test but is not suitable for stale-node decisions.
double unixTimeSeconds() {
  time_t current = time(nullptr);
  return current > 1700000000 ? static_cast<double>(current) : 0.0;
}

bool encodeTelemetry(char *out, size_t capacity, const TelemetrySample &sample) {
  int written = snprintf(
      out, capacity,
      "{\"type\":\"swarmlink.telemetry.compact.v1\",\"n\":\"%s\","
      "\"t\":%.3f,\"p\":[%.3f,%.3f,%.3f],\"v\":[%.3f,%.3f,%.3f],"
      "\"k\":\"%s\",\"b\":%.1f,\"s\":\"%s\"}",
      NODE_ID, unixTimeSeconds(), sample.x_m, sample.y_m, sample.z_m,
      sample.vx_mps, sample.vy_mps, sample.vz_mps, VEHICLE_KIND,
      sample.battery_percent, sample.mission_state);
  return written > 0 && static_cast<size_t>(written) < capacity && static_cast<size_t>(written) <= MAX_PACKET_BYTES;
}

void forwardToUsb(const uint8_t *data, int length) {
  if (length <= 0 || static_cast<size_t>(length) > MAX_PACKET_BYTES) return;
  // JSON has no embedded newline. Writing the original bytes preserves the
  // versioned packet exactly, then the Python base station reads one line.
  Serial.write(data, length);
  Serial.write('\n');
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  (void)info;
  forwardToUsb(data, length);
}
#else
void onDataReceived(const uint8_t *mac, const uint8_t *data, int length) {
  (void)mac;
  forwardToUsb(data, length);
}
#endif

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("# swarmlink: esp-now send failed");
}

bool setUpEspNow() {
  WiFi.mode(WIFI_STA);
  if (ESPNOW_CHANNEL != 0) WiFi.setChannel(ESPNOW_CHANNEL);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onDataReceived);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  return esp_now_is_peer_exist(BROADCAST_MAC) || esp_now_add_peer(&peer) == ESP_OK;
}

void setup() {
  Serial.begin(BAUDRATE);
  delay(300);
  if (!setUpEspNow()) {
    Serial.println("# swarmlink: ESP-NOW setup failed");
    return;
  }
  espNowReady = true;
  Serial.println("# swarmlink: ESP-NOW telemetry bridge ready");
}

void loop() {
  if (!espNowReady) {
    delay(1000);
    return;
  }
  static uint32_t lastPublishMs = 0;
  if (millis() - lastPublishMs < PUBLISH_INTERVAL_MS) return;
  lastPublishMs = millis();

  char packet[MAX_PACKET_BYTES + 1];
  TelemetrySample sample = readYourSensors();
  if (!encodeTelemetry(packet, sizeof(packet), sample)) {
    Serial.println("# swarmlink: telemetry packet is invalid or too large");
    return;
  }
  esp_err_t result = esp_now_send(BROADCAST_MAC, reinterpret_cast<uint8_t *>(packet), strlen(packet));
  if (result != ESP_OK) Serial.printf("# swarmlink: esp-now send error %d\n", result);
}
