#include <Arduino.h>
#include <ESP32Time.h>
#include <PicoMQTT.h>
#include <WiFi.h>
#include <vector>
#include <unordered_map>
#include <string>

// RTC instance
ESP32Time rtc;

// WiFi Credentials
const char *WIFI_SSID = "Galaxy";
const char *WIFI_PASS = "kartik2001";

// configaration variable

std::vector<int> frequecies;
std::vector<std::string> UpperBody {}
std::unordered_map<int, std::vector<int>> cofig;

// MQTT Configuration
PicoMQTT::Server mqtt;

void setup()
{
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("Connecting...");

  while (WiFi.status() != WL_CONNECTED)
  {
    if (WiFi.status() == WL_CONNECT_FAILED)
    {
      Serial.println("Failed to connect to WIFI. Please verify credentials.");
    }
    delay(5000);
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());

  mqtt.subscribe("global/time", [](const char *topic, const char *payload)
                 { unsigned long unixTime = strtoul(payload, nullptr, 10); });

  rtc.setTime(1740146300, 0);

  mqtt.begin();
}

void loop()
{
  mqtt.loop();

  delay(2000);
  Serial.println(rtc.getEpoch());
}
