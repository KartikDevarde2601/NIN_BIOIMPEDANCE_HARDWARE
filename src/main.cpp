#include <ADG706.h>
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "BodyImpedance.h"
#include "ad5940.h"
#include "mbedtls/base64.h"

// *********************** BLE VARS
BLEServer *pServer = NULL;
BLECharacteristic *pSensorDataCharacteristic = NULL;
BLECharacteristic *pCommandCharacteristic = NULL;
BLECharacteristic *pInterruptCharacteristic = NULL;
BLECharacteristic *pCompletedCharacteristic = NULL;

#define SERVICE_UUID_SENSOR "9b3333b4-8307-471b-95d1-17fa46507379"
#define CHARACTERISTIC_SENSOR_DATA "766def80-beba-45d1-bad9-4f80ceba5938"
#define CHARACTERISTIC_UUID_COMMAND "ea8145ec-d810-471a-877e-177ce5841b63"
#define CHARACTERISTIC_UUID_INTERRUPT "9bcec788-0cba-4437-b3b0-b53f0ee37312"
#define CHARACTERISTIC_UUID_COMPLETED "9bcec788-0cba-beba-45d1-b53f0ee37312"

#define CONTROL_COMMAND_INTERRUPT "STOPPED"

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool nextCombination = false;
bool statSensorDataInterrupt = false;
bool collectBioimpedance = false;

// define varible for AD5940
#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];
int VECLIMITCOUNTER = 0;
float freqAD;
std::string currentConfig;
const int MAXVECLIMIT = 20;

struct BioPhaseData {
  float bioImpedance;
  float phaseAngle;
};

struct Payload {
  int freq;
  std::string config;
  std::vector<BioPhaseData> data;
};

// configaration variable
std::vector<std::string> config;
std::vector<int> frequecies;
std::string sensortype;
int datapoints;

uint32_t datacount = 0;

// mux configuration module
ADG706 mux1(4, 5, 6, 7);
ADG706 mux2(15, 18, 45, 46);
ADG706 mux3(35, 36, 37, 38);
ADG706 mux4(39, 40, 41, 42);

// mux configuration board
// ADG706 mux1(4, 5, 6, 7);
// ADG706 mux2(35, 36, 37, 38);
// ADG706 mux3(45, 46, 47, 48);
// ADG706 mux4(8, 9, 10, 3);

// ✅ Serialize the struct into a byte array
std::vector<uint8_t> serializePayload(const Payload &payload) {
  std::vector<uint8_t> buffer;

  // Add frequency (4 bytes)
  buffer.insert(buffer.end(), (uint8_t *)&payload.freq, (uint8_t *)&payload.freq + sizeof(payload.freq));

  // Add config string length (2 bytes) + string data
  uint16_t configLen = payload.config.size();
  buffer.insert(buffer.end(), (uint8_t *)&configLen, (uint8_t *)&configLen + sizeof(configLen));
  buffer.insert(buffer.end(), payload.config.begin(), payload.config.end());

  // Add number of BioPhaseData elements (2 bytes)
  uint16_t dataSize = payload.data.size();
  buffer.insert(buffer.end(), (uint8_t *)&dataSize, (uint8_t *)&dataSize + sizeof(dataSize));

  // Add BioPhaseData elements
  for (const auto &data : payload.data) {
    buffer.insert(buffer.end(), (uint8_t *)&data.bioImpedance,
                  (uint8_t *)&data.bioImpedance + sizeof(data.bioImpedance));
    buffer.insert(buffer.end(), (uint8_t *)&data.phaseAngle, (uint8_t *)&data.phaseAngle + sizeof(data.phaseAngle));
  }

  return buffer;
}

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

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    BLEDevice::startAdvertising();
    Serial.println("Client Connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Client Disconnected");
  }
};

class CommandCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (!value.empty()) {
      std::string receivedData = std::string(value.begin(), value.end());
      bool result = deserializeStringMessage(receivedData);
      if (result) {
        collectBioimpedance = true;
      }
    } else {
      Serial.println("No data received.");
    }
  }
};

class InterruptCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (value == CONTROL_COMMAND_INTERRUPT) {
      statSensorDataInterrupt = true;
    }
  }
};

void SensorDataSetAndNotify(const Payload &payload) {
  auto buffer = serializePayload(payload);
  pSensorDataCharacteristic->setValue(buffer.data(), buffer.size());
  pSensorDataCharacteristic->notify();
}

void notifyCompletion() {
  uint8_t value = 0x01;  // Completion signal
  pCompletedCharacteristic->setValue(&value, 1);
  pCompletedCharacteristic->notify();  // ✅ Notify the connected app
  Serial.println("✅ Data collection completed - Notification sent!");
}

std::unordered_map<std::string, std::function<void()>> funcMap;

int concatenateIntegers(int i, int j) {
  // Handle the case where j is 0
  if (j == 0) {
    return i * 10;
  }

  // Calculate the number of digits in j
  int j_copy = j;
  int digits = 0;
  while (j_copy != 0) {
    j_copy /= 10;
    digits++;
  }

  // Scale i and add j
  int result = i * std::pow(10, digits) + j;
  return result;
}

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

  std::vector<BioPhaseData> sensorData;

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
        sensorData.push_back(tempData);
        VECLIMITCOUNTER++;
      }
    }

    // When the accumulated data count in this batch reaches the limit, send it.
    if (VECLIMITCOUNTER >= MAXVECLIMIT) {
      Payload payload;
      payload.freq = freqAD;
      payload.config = currentConfig;
      payload.data = sensorData;
      SensorDataSetAndNotify(payload);
      sensorData.clear();
      delay(50);
      VECLIMITCOUNTER = 0;
    }

    // When the total number of datapoints is reached, shut down.
    if (datacount >= datapoints) {
      AppBIAInit(0, 0);
      AppBIACtrl(BIACTRL_SHUTDOWN, 0);

      if (VECLIMITCOUNTER > 0) {
        Payload payload;
        payload.freq = freqAD;
        payload.config = currentConfig;
        payload.data = sensorData;
        SensorDataSetAndNotify(payload);
        sensorData.clear();
        delay(50);
        VECLIMITCOUNTER = 0;
      }
      printf("{\"type\":\"end\"}\n");
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Create the BLE Device
  BLEDevice::init("NIN_IMPEDANCE");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID_SENSOR);

  // SensorData Characteristic
  pSensorDataCharacteristic =
      pService->createCharacteristic(CHARACTERISTIC_SENSOR_DATA, BLECharacteristic::PROPERTY_NOTIFY);

  pSensorDataCharacteristic->addDescriptor(new BLE2902());

  // Conpleted  Characteristic
  pCompletedCharacteristic =
      pService->createCharacteristic(CHARACTERISTIC_UUID_COMPLETED, BLECharacteristic::PROPERTY_NOTIFY);

  pCompletedCharacteristic->addDescriptor(new BLE2902());

  // InterruptCallback Characteristic
  pInterruptCharacteristic =
      pService->createCharacteristic(CHARACTERISTIC_UUID_INTERRUPT, BLECharacteristic::PROPERTY_WRITE);

  pInterruptCharacteristic->setCallbacks(new InterruptCallback());

  // Command Characteristic
  pCommandCharacteristic =
      pService->createCharacteristic(CHARACTERISTIC_UUID_COMMAND, BLECharacteristic::PROPERTY_WRITE);
  pCommandCharacteristic->setCallbacks(new CommandCallback());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_SENSOR);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
  Serial.println("Waiting for a client connection to notify...");

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
}

void loop() {
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
          printf("Input command for Config is wrong, not found in funcMap");
        }
      }
    }
    collectBioimpedance = false;
    notifyCompletion();
  }
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}
