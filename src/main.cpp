#include <ADG706.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PicoMQTT.h>
#include <WiFi.h>

#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "BodyImpedance.h"
#include "ad5940.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "nvs_flash.h"

#define WIFI_SSID "wifi@iiith"
#define EAP_IDENTITY "kartik.devarde@ihub-data.iiit.ac.in"
#define EAP_PASSWORD "Kartik@2001"

static const char *TAG = "IIITH-IHUB-WiFi-MM";
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG, "WiFi Started, Connecting...");
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGW(TAG, "Disconnected, Retrying...");
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Connected! IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
    Serial.print("Assigned IP Address: ");
    Serial.println(ip4addr_ntoa((const ip4_addr_t *)&event->ip_info.ip));
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

bool wifi_init() {
  wifi_event_group = xEventGroupCreate();
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  ESP_LOGI(TAG, "Initializing WiFi...");
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  wifi_config_t wifi_config = {};
  strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG, "Waiting for connection...");
  EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    Serial.println("WiFi Connected");
    return true;
  } else {
    Serial.println("WiFi Connection Failed");
    return false;
  }
}

// mux configuration module
ADG706 mux1(4, 5, 6, 7);
ADG706 mux2(15, 18, 45, 46);
ADG706 mux3(35, 36, 37, 38);
ADG706 mux4(39, 40, 41, 42);

// // mux configuration board
// ADG706 mux1(4, 5, 6, 7);
// ADG706 mux2(35, 36, 37, 38);
// ADG706 mux3(45, 46, 47, 48);
// ADG706 mux4(8, 9, 10, 3);

// define varible for AD5940
#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];
int VECLIMITCOUNTER = 0;
float freqAD;
std::string currentConfig;
const int MAXVECLIMIT = 20;

// configaration variable
std::vector<std::string> config;
std::vector<int> frequecies;
std::string sensortype;
int datapoints;
bool collectBioimpedance = false;
uint32_t datacount = 0;

std::map<std::string, std::function<void()>> funcMap;

// MQTT Configuration
PicoMQTT::Client mqtt("10.2.216.208");

void activate_right_body_mux() {
  mux1.selectChannel(1);
  mux3.selectChannel(1);
  mux2.selectChannel(1);
  mux4.selectChannel(1);
  printf("activate_right_body_mux\n");
  delay(10);
}

void activate_left_body_mux() {
  mux1.selectChannel(3);
  mux3.selectChannel(3);
  mux2.selectChannel(6);
  mux4.selectChannel(6);
  printf("activate_left_body_mux\n");
  delay(10);
}

void activate_upper_body_mux() {
  mux1.selectChannel(1);
  mux3.selectChannel(1);
  mux2.selectChannel(2);
  mux4.selectChannel(2);
  printf("activate_upper_body_mux\n");
  delay(10);
}

void activate_lower_body_mux() {
  mux1.selectChannel(2);
  mux3.selectChannel(2);
  mux2.selectChannel(5);
  mux4.selectChannel(5);
  printf("activate_lower_body_mux\n");
  delay(10);
}

void SendDataToMobile(JsonDocument &payload, const char *topic) {
  auto publish = mqtt.begin_publish(topic, measureJson(payload));
  serializeJson(payload, publish);
  publish.send();
}

void SendCommandToMobile(const char *payload, const char *topic) {
  // Serial.printf("Publishing command in topic '%s': %s\n", topic, payload);

  mqtt.publish(topic, payload);
}

static int32_t AD5940PlatformCfg(void) {
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;   // AppBIACfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT
                             // result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg); /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg); /* Enable FIFO here */

  /* Step3. Interrupt controller */

  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT,
                 bTRUE); /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH,
                 bTRUE); /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC | GP5_SYNC | GP4_SYNC | GP2_TRIG | GP1_SYNC | GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin4 | AGPIO_Pin5 | AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;

  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940BIAStructInit(float SF, int dlimit) {
  AppBIACfg_Type *pBIACfg;

  AppBIAGetCfg(&pBIACfg);

  pBIACfg->SeqStartAddr = 0;
  pBIACfg->MaxSeqLen = 512; /** @todo add checker in function */

  pBIACfg->RcalVal = 10000.0;
  pBIACfg->DftNum = DFTNUM_8192;
  pBIACfg->NumOfData = dlimit; /* Never stop until you stop it manually by AppBIACtrl() function */
  pBIACfg->BiaODR = 20;        /* ODR(Sample Rate) 20Hz */
  pBIACfg->FifoThresh = 4;     /* 4 */
  pBIACfg->SinFreq = SF;
  pBIACfg->ADCSinc3Osr = ADCSINC3OSR_2;
}

void AD5940_Main() {
  datacount = 0;
  uint32_t temp;

  AD5940PlatformCfg();
  AD5940BIAStructInit(freqAD, datapoints); /* Configure your parameters in this function */
  AppBIAInit(
      AppBuff,
      APPBUFF_SIZE); /* Initialize BIA application. Provide a buffer, which is used to store sequencer commands */
  AppBIACtrl(BIACTRL_START,
             0); /* Control BIA measurement to start. Second parameter has no meaning with this command. */

  JsonDocument doc;

  struct BioPhaseData {
    float bioImpedance;
    float phaseAngle;
  };

  // Create a fixed-size array to store the sensor data.
  BioPhaseData sensorData[datapoints];
  // Index to track the current count of data entries.
  uint32_t sensorDataIndex = 0;

  doc["sensor"] = "BioImpedance";
  doc["freq"] = freqAD;
  doc["config"] = currentConfig;
  while (1) {
    // Check if interrupt flag which will be set when interrupt occurred.
    if (AD5940_GetMCUIntFlag()) {
      AD5940_ClrMCUIntFlag();  // Clear this flag
      temp = APPBUFF_SIZE;
      AppBIAISR(AppBuff, &temp);  // Deal with it and provide a buffer to store data we got

      fImpPol_Type *pImp = (fImpPol_Type *)AppBuff;
      AppBIACtrl(BIACTRL_GETFREQ, &freqAD);

      /* Process data */
      for (int i = 0; i < temp; i++) {
        datacount++;  // Update the total data count
        BioPhaseData tempData;
        tempData.bioImpedance = pImp[i].Magnitude;
        tempData.phaseAngle = pImp[i].Phase * 180 / MATH_PI;
        // Store data if we haven't exceeded the fixed size
        if (sensorDataIndex < datapoints) {
          sensorData[sensorDataIndex++] = tempData;
        }
        VECLIMITCOUNTER++;
      }
    }

    // When the accumulated data count in this batch reaches the limit, send it.
    if (VECLIMITCOUNTER >= MAXVECLIMIT) {
      JsonArray dataArray = doc.createNestedArray("data");
      for (uint32_t i = 0; i < sensorDataIndex; i++) {
        JsonObject entry = dataArray.createNestedObject();
        entry["bioImpedance"] = sensorData[i].bioImpedance;
        entry["phaseAngle"] = sensorData[i].phaseAngle;
      }
      SendDataToMobile(doc, "mobile/bio/data");
      // Serial.printf("batch:::::");
      // serializeJson(doc, Serial);
      // Serial.println();  // Newline for readability

      // Reset for the next batch
      delay(50);
      VECLIMITCOUNTER = 0;
      doc.clear();
      sensorDataIndex = 0;
    }

    // When the total number of datapoints is reached, shut down.
    if (datacount >= datapoints) {
      AppBIAInit(0, 0);
      AppBIACtrl(BIACTRL_SHUTDOWN, 0);

      if (VECLIMITCOUNTER > 0) {
        JsonArray dataArray = doc.createNestedArray("data");
        for (uint32_t i = 0; i < sensorDataIndex; i++) {
          JsonObject entry = dataArray.createNestedObject();
          entry["bioImpedance"] = sensorData[i].bioImpedance;
          entry["phaseAngle"] = sensorData[i].phaseAngle;
        }
        // SendDataToMobile(doc, "mobile/bio/data");
        // Serial.printf("batch:::::");
        // serializeJson(doc, Serial);
        // Serial.println();  // Newline for readability
        delay(50);
        VECLIMITCOUNTER = 0;
        doc.clear();
        sensorDataIndex = 0;
      }
      printf("{\"type\":\"end\"}\n");
      break;
    }
  }
}

// bool deserializeMessage(const char *topic, Stream &stream) {
//   Serial.printf("Received message in topic '%s':\n", topic);
//   JsonDocument doc;
//   DeserializationError error = deserializeJson(doc, stream);
//   if (error) {
//     Serial.println("Failed to deserialize JSON");
//     return false;
//   }

//   JsonArray configArray = doc["config"].as<JsonArray>();
//   config.clear();
//   for (JsonVariant v : configArray) {
//     config.push_back(v.as<std::string>());
//   }

//   frequecies.clear();
//   JsonArray freqArray = doc["frequecy"].as<JsonArray>();
//   for (JsonVariant v : freqArray) {
//     frequecies.push_back(v.as<int>());
//   }

//   datapoints = doc["datapoints"];

//   sensortype = doc["sensorType"].as<std::string>();

//   Serial.println("Deserialization successful");
//   return true;
// }

bool deserializeStringMessage(std::string input) {
  // Split using ':'
  config.clear();
  frequecies.clear();
  std::vector<std::string> tokens;
  std::stringstream ss(input);
  std::string token;

  while (std::getline(ss, token, ':')) {
    tokens.push_back(token);
  }

  if (tokens.size() < 4) {
    return false;
  }

  std::string sensorType = tokens[0];
  std::string configStr = tokens[1];    // "fullbody,rightbody"
  std::string frequiesStr = tokens[2];  // "100,200,300,500"
  std::string datapointsStr = tokens[3];

  std::stringstream configStream(configStr);
  while (std::getline(configStream, token, ',')) {
    config.push_back(token);
  }

  std::stringstream freqStream(frequiesStr);
  while (std::getline(freqStream, token, ',')) {
    frequecies.push_back(std::stoi(token));
  }

  datapoints = 60;

  sensortype = sensorType;

  return true;
}

void wifiEvent(WiFiEvent_t event) {
  if (event == SYSTEM_EVENT_STA_DISCONNECTED) {
    Serial.println("WiFi disconnected!");
    collectBioimpedance = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  bool wifi_connected = wifi_init();
  if (!wifi_connected) {
    Serial.println("Failed to connect to WiFi");
    return;
  }

  mux1.begin();
  mux2.begin();
  mux3.begin();
  mux4.begin();

  Serial.println("ADG706 MUX Initialized");

  funcMap["RIGHTBODY"] = activate_right_body_mux;
  funcMap["LEFTBODY"] = activate_left_body_mux;
  funcMap["UPPERBODY"] = activate_upper_body_mux;
  funcMap["LOWERBODY"] = activate_lower_body_mux;
  // put your setup code here, to run once:
  AD5940_MCUResourceInit(NULL);

  Serial.println("MCU Initialised");

  // reset AD5940
  Serial.println("Attempting to reset AD5940...");
  AD5940_HWReset();
  Serial.println("AD5940 reset!");

  // initialise AD5940 by writing the startup sequence
  Serial.println("Attempting to initialise AD5940...");
  AD5940_Initialize();
  Serial.println("AD5940 initialised!\n");
  delay(50);
  Serial.println("BIA init!");

  mqtt.subscribe("esp/bio/data", [](const char *topic, const char *payload) {
    Serial.printf("Received message in topic '%s': %s\n", topic, payload);
    if (!deserializeStringMessage(payload)) {
      Serial.printf("Failed to process message");
    } else {
      Serial.printf("success to process message\n");
      if (sensortype == "bioImpedance") {
        collectBioimpedance = true;
      }
    }
  });

  mqtt.subscribe("esp/bio/command", [](const char *topic, const char *payload) {
    Serial.printf("Received message in topic '%s': %s\n", topic, payload);
    String payloadStr = String(payload);
    if (payloadStr.equals("stop")) {
      collectBioimpedance = false;
    }
  });

  mqtt.begin();
}

void loop() {
  mqtt.loop();
  if (collectBioimpedance) {
    for (int i = 0; i < frequecies.size(); i++) {
      for (int j = 0; j < config.size(); j++) {
        freqAD = frequecies[i] * 1000.00;
        currentConfig = config[j];

        if (funcMap.find(currentConfig) != funcMap.end()) {
          funcMap[currentConfig]();
          printf("Current Config: %s\n", currentConfig.c_str());
          printf("Current Freq: %f\n", freqAD);
          AD5940_Main();
        } else {
          //  printf("Input command for Config is wrong, not found in funcMap");
        }
      }
    }
    // collectBioimpedance = false;
    // SendCommandToMobile("completed", "mobile/bio/command");
  }
}
