#include <ADG706.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Time.h>
#include <PicoMQTT.h>
#include <WiFi.h>

#include <string>
#include <unordered_map>
#include <vector>

// RTC instance
ESP32Time rtc;

// WiFi Credentials
const char *WIFI_SSID = "Galaxy";
const char *WIFI_PASS = "kartik2001";

// mux configuration
ADG706 mux1(0, 1, 2, 3);
ADG706 mux2(0, 1, 2, 3);
ADG706 mux3(0, 1, 2, 3);
ADG706 mux4(0, 1, 2, 3, 4, 5, 6);

// configaration variable
std::vector<int> config;
std::vector<int> frequecies;
std::string sensortype;
int datapoints;
bool start = false;

// std::vector<std::pair<int, int>> RBRH = {{1, 1}, {3, 1}};
// std::vector<std::pair<int, int>> RBRL = {{2, 3}, {4, 3}};
// std::vector<std::pair<int, int>> LBLH = {{1, 2}, {3, 2}};
// std::vector<std::pair<int, int>> LBLL = {{2, 4}, {4, 4}};
// std::vector<std::pair<int, int>> UBRH = {{1, 1}, {3, 1}};
// std::vector<std::pair<int, int>> UBLH = {{2, 2}, {4, 2}};
// std::vector<std::pair<int, int>> BBRL = {{1, 3}, {3, 3}};
// std::vector<std::pair<int, int>> BBLL = {{2, 6}, {4, 6}};
// std::vector<std::pair<int, int>> FFBB = {{1, 0}, {2, 0}, {3, 0}, {4, 0}};
std::unordered_map<int, std::vector<int>> cofig;

// MQTT Configuration
PicoMQTT::Server mqtt;

void activate_right_body_mux() {
  mux1.selectChannel(1);
  mux3.selectChannel(1);
  mux2.selectChannel(3);
  mux4.selectChannel(3);
}

void activate_left_body() {
  mux1.selectChannel(1);
  mux3.selectChannel(2);
  mux2.selectChannel(4);
  mux4.selectChannel(4);
}

void activate_upper_body() {
  mux1.selectChannel(1);
  mux3.selectChannel(1);
  mux3.selectChannel(2);
  mux4.selectChannel(3);
}

void activate_lower_body() {
  mux1.selectChannel(3);
  mux3.selectChannel(3);
  mux2.selectChannel(6);
  mux4.selectChannel(6);
}

void activate_full_body() {
  mux1.selectChannel(0);
  mux2.selectChannel(0);
  mux3.selectChannel(0);
  mux4.selectChannel(0);
}

bool deserializeMessage(const char *topic, Stream &stream) {
  Serial.printf("Received message in topic '%s':\n", topic);
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, stream);
  if (error) {
    Serial.println("Failed to deserialize JSON");
    return false;
  }

  JsonArray configArray = doc["config"].as<JsonArray>();
  config.clear();
  for (JsonVariant v : configArray) {
    config.push_back(v.as<int>());
  }

  frequecies.clear();
  JsonArray freqArray = doc["frequecy"].as<JsonArray>();
  for (JsonVariant v : freqArray) {
    frequecies.push_back(v.as<int>());
  }

  datapoints = doc["datapoints"];

  sensortype = doc["sensorType"].as<std::string>();

  Serial.println("Deserialization successful");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  mux1.begin();
  mux2.begin();

  Serial.println("ADG706 MUX Initialized");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("Connecting...");

  while (WiFi.status() != WL_CONNECTED) {
    if (WiFi.status() == WL_CONNECT_FAILED) {
      Serial.println("Failed to connect to WIFI. Please verify credentials.");
    }
    delay(5000);
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());

  mqtt.subscribe("global/time", [](const char *topic, const char *payload) {
    unsigned long unixTime = strtoul(payload, nullptr, 10);
  });

  mqtt.subscribe("global/command_devices", [](const char *topic, Stream &stream) {
    if (!deserializeMessage(topic, stream)) {
      Serial.println("Failed to process message");
    } else {
      if (sensortype == "bioImpedance") {
        start = true;
      }
    }
  });

  rtc.setTime(1740146300, 0);

  mqtt.begin();
}

void loop() {
  mqtt.loop();

  while (start) {
  }
}
