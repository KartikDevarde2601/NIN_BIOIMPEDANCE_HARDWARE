#include <Arduino.h>
#include <ESP32Time.h>
#include <PicoMQTT.h>
#include <WiFi.h>

#include "time.h"

// RTC instance
ESP32Time rtc;

// NTP Server
const char *ntpServer = "pool.ntp.org";  // Alternative: "time.google.com"
const long gmtOffset_sec = 19800;        // GMT+5:30 (India Standard Time)
const int daylightOffset_sec = 0;

// WiFi Credentials
const char *WIFI_SSID = "Galaxy";
const char *WIFI_PASS = "kartik2001";

// MQTT Configuration
PicoMQTT::Server mqtt;

void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("Connecting...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Waiting for WiFi...");
    delay(5000);
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());

  mqtt.begin();

  // Get Timestamp (with retry)
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;

  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 10) {
    Serial.println("Waiting for NTP time sync...");
    delay(1000);
    retry++;
  }

  if (retry >= 10) {
    Serial.println("Failed to obtain time after retries.");
    return;
  }

  // Debugging NTP time
  Serial.print("NTP Time obtained: ");
  Serial.println(asctime(&timeinfo));

  // Get epoch time and set RTC correctly
  rtc.setTime(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour,
              timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
  
  Serial.println("RTC Time Set from NTP");

  // Print the current RTC time
  Serial.print("RTC Current Time: ");
  Serial.println(rtc.getEpoch());
}

void loop() { mqtt.loop(); }
