#include <Arduino.h>
#include <WiFi.h>
#include <PicoMQTT.h>
#include "time.h"
#include <ESP32Time.h>

// RTC instance
ESP32Time rtc;

// NTP Server
const char *ntpServer = "pool.ntp.org"; // Alternative: "time.google.com"
const long gmtOffset_sec = 19800;       // GMT+5:30 (India Standard Time)
const int daylightOffset_sec = 0;

// WiFi Credentials
const char *WIFI_SSID = "Galaxy";
const char *WIFI_PASS = "kartik2001";

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

  mqtt.begin();

  // Get Timestamp (with retry)
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;

  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 10) // Retry up to 10 times
  {
    Serial.println("Waiting for NTP time sync...");
    delay(1000);
    retry++;
  }

  if (retry >= 10)
  {
    Serial.println("Failed to obtain time after retries.");
    return;
  }

  // Get epoch time and set RTC
  time_t now;
  time(&now);
  rtc.setTime(now); // Sync RTC with NTP time
  Serial.println("RTC Time Set from NTP");

  // Print the current RTC time
  unsigned long currentTime = rtc.getLocalEpoch();
  Serial.println(currentTime);
}

void loop()
{
  mqtt.loop();
}
