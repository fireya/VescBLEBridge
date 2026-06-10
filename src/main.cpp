#include <Arduino.h>
#include <NimBLEDevice.h>

static const char *const LOG_TAG = "BleServer";

int MTU_SIZE = 128;
int PACKET_SIZE = MTU_SIZE - 3;

NimBLEServer *pServer = nullptr;
NimBLECharacteristic *pCharacteristicVescTx = nullptr;
NimBLECharacteristic *pCharacteristicVescRx = nullptr;

bool deviceConnected = false;
bool oldDeviceConnected = false;

#define VESC_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define VESC_CHAR_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define VESC_CHAR_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ===== BLE Callbacks =====
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) {
    ESP_LOGI(LOG_TAG, "Client connected: %s", NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    deviceConnected = true;
    NimBLEDevice::startAdvertising();
  }

  void onDisconnect(NimBLEServer *pServer) {
    ESP_LOGI(LOG_TAG, "Client disconnected - start advertising");
    deviceConnected = false;
    NimBLEDevice::startAdvertising();
  }

  void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc) {
    ESP_LOGI(LOG_TAG, "MTU changed - new size %d", MTU);
    MTU_SIZE = MTU;
    PACKET_SIZE = MTU_SIZE - 3;
  }
};

char tmpbuf[1024];

void dumpBuffer(std::string header, std::string buffer) {
  if (esp_log_level_get(LOG_TAG) < ESP_LOG_DEBUG) return;
  int length = snprintf(tmpbuf, 50, "%s : len = %d / ", header.c_str(), buffer.length());
  for (char i : buffer) {
    length += snprintf(tmpbuf + length, 1024 - length, "%02x ", i);
  }
  ESP_LOGD(LOG_TAG, "%s", tmpbuf);
}

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    ESP_LOGD(LOG_TAG, "onWrite to characteristics: %s", pCharacteristic->getUUID().toString().c_str());
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      if (pCharacteristic->getUUID().equals(pCharacteristicVescRx->getUUID())) {
        dumpBuffer("BLE/UART => VESC: ", rxValue);
        for (int i = 0; i < rxValue.length(); i++) {
          Serial0.write(rxValue[i]); // BLE -> VESC
        }
      }
    }
  }
};

// ===== VESC packet parser for controller passthrough =====
// Собираем полные VESC-фреймы от Serial0(VESC), дублируем на Serial1(Controller)
static enum { V_IDLE, V_WAIT_LEN, V_WAIT_LEN2, V_GATHER } vState = V_IDLE;
static int vLen = 0;
static int vIdx = 0;
static uint8_t vBuf[540];

static void feedVescByte_dup(uint8_t b) {
  switch (vState) {
    case V_IDLE:
      if (b == 0x02 || b == 0x03) {
        vState = V_WAIT_LEN;
        vBuf[0] = b;
        vIdx = 1;
        vLen = 0;
      }
      break;
    case V_WAIT_LEN: {
      if (vBuf[0] == 0x02) {
        vLen = b;
        vBuf[vIdx++] = b;
        vState = V_GATHER;
      } else {
        vLen = (b << 8);
        vBuf[vIdx++] = b;
        vState = V_WAIT_LEN2;
      }
      break;
    }
    case V_WAIT_LEN2: {
      vLen |= b;
      vBuf[vIdx++] = b;
      vState = V_GATHER;
      break;
    }
    case V_GATHER: {
      if (vIdx < (int)sizeof(vBuf)) {
        vBuf[vIdx++] = b;
      }
      int headerBytes = (vBuf[0] == 0x02) ? 1 : 2;
      int totalFrame = 1 + headerBytes + vLen + 2 + 1;
      if (vIdx >= totalFrame) {
        // Duplicate VESC response to controller
        Serial1.write(vBuf, vIdx);
        vState = V_IDLE;
        vIdx = 0;
        vLen = 0;
      }
      break;
    }
  }
}

// ===== Controller -> VESC passthrough (frame-aware) =====
static enum { C_IDLE, C_WAIT_LEN, C_WAIT_LEN2, C_GATHER } cState = C_IDLE;
static int cLen = 0;
static int cIdx = 0;
static uint8_t cBuf[540];

static void feedCtrlByte(uint8_t b) {
  switch (cState) {
    case C_IDLE:
      if (b == 0x02 || b == 0x03) {
        cState = C_WAIT_LEN;
        cBuf[0] = b;
        cIdx = 1;
        cLen = 0;
      }
      break;
    case C_WAIT_LEN: {
      if (cBuf[0] == 0x02) {
        cLen = b;
        cBuf[cIdx++] = b;
        cState = C_GATHER;
      } else {
        cLen = (b << 8);
        cBuf[cIdx++] = b;
        cState = C_WAIT_LEN2;
      }
      break;
    }
    case C_WAIT_LEN2: {
      cLen |= b;
      cBuf[cIdx++] = b;
      cState = C_GATHER;
      break;
    }
    case C_GATHER: {
      if (cIdx < (int)sizeof(cBuf)) {
        cBuf[cIdx++] = b;
      }
      int headerBytes = (cBuf[0] == 0x02) ? 1 : 2;
      int totalFrame = 1 + headerBytes + cLen + 2 + 1;
      if (cIdx >= totalFrame) {
        Serial0.write(cBuf, cIdx); // Controller -> VESC
        cState = C_IDLE;
        cIdx = 0;
        cLen = 0;
      }
      break;
    }
  }
}

// ===== Setup / Loop =====
void setup() {
  Serial.begin(115200);

  // Serial0: ESP32 RX=20, TX=21  <--->  VESC
  Serial0.begin(115200, SERIAL_8N1, 20, 21);

  // Serial1: ESP32 RX=3, TX=4  <--->  Controller receiver (VX3 Pro)
  Serial1.begin(115200, SERIAL_8N1, 3, 4);

  // BLE
  NimBLEDevice::init("VescBLEBridge");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  auto pSecurity = new NimBLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

  BLEService *pService = pServer->createService(VESC_SERVICE_UUID);

  pCharacteristicVescTx = pService->createCharacteristic(
      VESC_CHAR_UUID_TX,
      NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

  pCharacteristicVescRx = pService->createCharacteristic(
      VESC_CHAR_UUID_RX,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

  pCharacteristicVescRx->setCallbacks(new MyCallbacks());

  pService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(VESC_SERVICE_UUID);
  pAdvertising->start();
  ESP_LOGI(LOG_TAG, "waiting a client connection to notify...");

  // Drain stale data
  delay(50);
  while (Serial0.available()) Serial0.read();
  while (Serial1.available()) Serial1.read();
}

std::string vescBuffer;

void loop() {
  // 1. Controller (Serial1) -> VESC (Serial0)
  while (Serial1.available()) {
    feedCtrlByte(Serial1.read());
  }

  // 2. VESC (Serial0) -> BLE + Controller duplicate
  if (Serial0.available()) {
    int oneByte;
    while (Serial0.available()) {
      oneByte = Serial0.read();
      vescBuffer.push_back((char)oneByte);
      // Also feed to VESC dup parser for controller
      feedVescByte_dup((uint8_t)oneByte);
    }

    if (deviceConnected) {
      while (vescBuffer.length() > 0) {
        if (vescBuffer.length() > PACKET_SIZE) {
          dumpBuffer("VESC => BLE/UART", vescBuffer.substr(0, PACKET_SIZE));
          pCharacteristicVescTx->setValue(vescBuffer.substr(0, PACKET_SIZE));
          vescBuffer = vescBuffer.substr(PACKET_SIZE);
        } else {
          dumpBuffer("VESC => BLE/UART", vescBuffer);
          pCharacteristicVescTx->setValue(vescBuffer);
          vescBuffer.clear();
        }
        pCharacteristicVescTx->notify();
        delay(5);
      }
    }
  }

  // 3. Connection state handling
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    ESP_LOGI(LOG_TAG, "start advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}
