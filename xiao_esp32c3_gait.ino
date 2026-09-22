/* ============================================================================
   GaitIMU XIAO ESP32-C3 — 200 Hz BLE IMU streamer (MPU-6050)
   Board: Seeed XIAO ESP32-C3 + MPU-6050 breakout
   Libraries: "esp32" Arduino core by Espressif (bundles ESP32 BLE Arduino + Wire)
   
   Wiring (MPU-6050 -> XIAO ESP32-C3):
   VCC -> 3V3 (MPU-6050 is 3.3 V ONLY — never 5 V)
   GND -> GND
   SDA -> D4 (GPIO6)
   SCL -> D5 (GPIO7)
   AD0 -> GND (I2C address 0x68)
   ========================================================================== */

#include <Wire.h>
#include <string.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ───────── USER CONFIG — set per module before flashing ───────── */
#define MODULE_ID "Torso" // "RightLeg" | "LeftLeg" | "Torso"
/* ──────────────────────────────────────────────────────────────── */
#define DEVICE_NAME "GaitIMU-" MODULE_ID

/* XIAO ESP32-C3 I2C pins: SDA = D4 (GPIO6), SCL = D5 (GPIO7) */
#define I2C_SDA 6
#define I2C_SCL 7
#define I2C_SPEED 400000 // drop to 100000 if the bus is unreliable
#define MPU_ADDR 0x68    // 0x69 if AD0 is tied high

/* Status LED: XIAO's onboard LED is a WS2812 NeoPixel (GPIO8) — not usable
   with digitalWrite. Leave -1, or point it at a free GPIO with an external
   LED (active low if to a transistor / active high to 3V3). */
#define MODULE_LED -1
#define LED_ACTIVE_LOW 1

/* Sampling / packetisation */
#define SAMPLE_RATE_HZ 200
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)

// FIX: Increased from 10 to 20 samples per packet
// This reduces BLE notifications from 20/s to 10/s per module (30/s total across 3 modules)
// Web Bluetooth handles this rate without dropping packets
#define MAX_SAMPLES_PER_PACKET 20
#define SAMPLES_PER_PACKET 20

#define MAX_PACKET_LEN (6 + 12 * MAX_SAMPLES_PER_PACKET)

/* BLE UUIDs — must match index.html */
#define GAIT_SERVICE_UUID "9f000001-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define CMD_CHAR_UUID     "9f000002-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define TIME_CHAR_UUID    "9f000003-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define DATA_CHAR_UUID    "9f000004-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define ID_CHAR_UUID      "9f000005-8c5a-4e1f-9a7b-3d6e5f0a1b2c"

#define CMD_START   0x01
#define CMD_STOP    0x02
#define CMD_SET_SPP 0x03

/* MPU-6050 registers */
#define REG_WHO_AM_I       0x75
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_SMPLRT_DIV     0x19
#define REG_PWR_MGMT_1     0x6B
#define REG_PWR_MGMT_2     0x6C
#define REG_ACCEL_XOUT_H   0x3B

#define VAL_CONFIG         0x01 // DLPF 184 Hz -> internal 1 kHz rate
#define VAL_GYRO_CONFIG    0x10 // ±1000 dps (FS_SEL = 2)
#define VAL_ACCEL_CONFIG   0x10 // ±8 g (AFS_SEL = 2)
#define VAL_SMPLRT_DIV     4    // 1 kHz / (1 + 4) = 200 Hz

/* ── BLE objects ── */
static BLEService* gaitService = nullptr;
static BLECharacteristic* cmdChar = nullptr;
static BLECharacteristic* timeChar = nullptr;
static BLECharacteristic* dataChar = nullptr;
static BLECharacteristic* idChar = nullptr;
static BLEAdvertising* advertising = nullptr;
static bool deviceConnected = false;

/* ── streaming state ── */
bool streaming = false;
uint32_t syncMillis = 0;
uint32_t nextSampleMs = 0;
uint8_t samplesPerPacket = SAMPLES_PER_PACKET;
uint8_t packetSeq = 0;
uint8_t packetFill = 0;
uint32_t packetT0 = 0;
uint8_t packetBuf[MAX_PACKET_LEN];

/* ── stats ── */
uint32_t smpCount = 0;
uint32_t pktCount = 0;
uint32_t i2cErrors = 0;
uint32_t i2cFailStreak = 0;
uint32_t statLast = 0;

/* ══ I2C helpers ══ */
static uint8_t readReg(uint8_t r) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(r);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((int)MPU_ADDR, (int)1);
  if (Wire.available() < 1) return 0;
  uint8_t v = (uint8_t)Wire.read();
  while (Wire.available()) Wire.read();
  return v;
}

static void writeReg(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission(true);
}

static bool hwReadSample(int16_t acc[3], int16_t gyr[3]) {
  uint8_t b[14]; /* ACCEL_X..Z(6) TEMP(2) GYRO_X..Z(6) */
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((int)MPU_ADDR, (int)14);
  if (Wire.available() < 14) return false;
  for (int i = 0; i < 14; i++) b[i] = (uint8_t)Wire.read();
  
  acc[0] = (int16_t)((uint16_t)(b[0] << 8) | b[1]);
  acc[1] = (int16_t)((uint16_t)(b[2] << 8) | b[3]);
  acc[2] = (int16_t)((uint16_t)(b[4] << 8) | b[5]);
  gyr[0] = (int16_t)((uint16_t)(b[8] << 8) | b[9]);
  gyr[1] = (int16_t)((uint16_t)(b[10] << 8) | b[11]);
  gyr[2] = (int16_t)((uint16_t)(b[12] << 8) | b[13]);
  return true;
}

/* ══ MPU-6050 bring-up ══ */
static bool imuInit() {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_SPEED);
  delay(50);
  writeReg(REG_PWR_MGMT_1, 0x80); /* device reset */
  delay(150);
  
  uint8_t who = readReg(REG_WHO_AM_I);
  Serial.print("[IMU] WHO_AM_I = 0x");
  Serial.println(who, HEX);
  if (who != 0x68 && who != 0x70) { /* 0x70 = MPU-6000 family too */
    Serial.println("[IMU] *** MPU-6050 not seen on I2C — check SDA/SCL wiring ***");
    return false;
  }
  
  writeReg(REG_PWR_MGMT_1, 0x01); /* wake, PLL X-gyro clock */
  writeReg(REG_PWR_MGMT_2, 0x00); /* all axes enabled */
  writeReg(REG_SMPLRT_DIV, VAL_SMPLRT_DIV);
  writeReg(REG_CONFIG, VAL_CONFIG);
  writeReg(REG_GYRO_CONFIG, VAL_GYRO_CONFIG);
  writeReg(REG_ACCEL_CONFIG, VAL_ACCEL_CONFIG);
  delay(50);
  
  Serial.print("[IMU] GYRO_CONFIG = 0x");
  Serial.println(readReg(REG_GYRO_CONFIG), HEX);
  Serial.print("[IMU] ACCEL_CONFIG = 0x");
  Serial.println(readReg(REG_ACCEL_CONFIG), HEX);
  return true;
}

/* ══ packet build / send ══ */
static void flushPacket() {
  if (packetFill == 0) return;
  packetBuf[0] = packetSeq++;
  packetBuf[1] = packetFill;
  memcpy(&packetBuf[2], &packetT0, 4);
  dataChar->setValue(packetBuf, 6 + 12 * packetFill);
  
  if (deviceConnected) {
    // FIX: notify() returns void in ESP32 BLE library v3.x, so just call it directly
    dataChar->notify();
  }
  packetFill = 0;
  pktCount++;
}

static void addSample(uint32_t sampleMs) {
  int16_t acc[3], gyr[3];
  if (!hwReadSample(acc, gyr)) {
    i2cErrors++; i2cFailStreak++;
    if (i2cFailStreak > 50) { /* bus wedged — re-initialise it */
      i2cFailStreak = 0;
      Wire.begin(I2C_SDA, I2C_SCL, I2C_SPEED);
    }
    return;
  }
  i2cFailStreak = 0;
  
  if (packetFill == 0) packetT0 = sampleMs - syncMillis;
  
  uint8_t* p = &packetBuf[6 + 12 * packetFill];
  memcpy(p, acc, 6);
  memcpy(p + 6, gyr, 6);
  packetFill++;
  smpCount++;
  
  if (packetFill >= samplesPerPacket) flushPacket();
}

/* ══ BLE callbacks ══ */
class GaitServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    Serial.println("[BLE] central connected");
  }
  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    streaming = false;
    packetFill = 0;
    Serial.println("[BLE] central disconnected — advertising again");
    advertising->start();
  }
};

class GaitCmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    if (v.length() == 0) return;
    switch ((uint8_t)v[0]) {
      case CMD_START:
        if (!streaming) {
          packetFill = 0; packetSeq = 0;
          nextSampleMs = millis() + 2;
          streaming = true;
          Serial.println("[CMD] streaming started");
        }
        break;
      case CMD_STOP:
        if (streaming) {
          streaming = false;
          flushPacket();
          Serial.println("[CMD] streaming stopped");
        }
        break;
      case CMD_SET_SPP: {
        uint8_t n = v.length() >= 2 ? (uint8_t)v[1] : 0;
        flushPacket();
        samplesPerPacket = (n >= 1 && n <= MAX_SAMPLES_PER_PACKET) ? n : 1;
        Serial.print("[CMD] samples/packet = ");
        Serial.println(samplesPerPacket);
        break;
      }
    }
  }
};

class GaitTimeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    if (v.length() != 6) return; /* 6-byte probe or real epoch */
    uint32_t sec; uint16_t ms10;
    memcpy(&sec, v.c_str(), 4);
    memcpy(&ms10, v.c_str() + 4, 2);
    syncMillis = millis();
    uint32_t rel = millis() - syncMillis;
    uint8_t ack[4];
    memcpy(ack, &rel, 4);
    c->setValue(ack, 4);
    c->notify();
    Serial.print("[TIME] epoch ");
    Serial.print(sec);
    Serial.print('.');
    Serial.println(ms10);
  }
};

/* ══ setup / loop ══ */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("GaitIMU XIAO ESP32-C3 node: " DEVICE_NAME);

#if MODULE_LED >= 0
  pinMode(MODULE_LED, OUTPUT);
  digitalWrite(MODULE_LED, LED_ACTIVE_LOW ? HIGH : LOW);
#endif

  bool imuOk = imuInit();
  if (!imuOk) Serial.println("[IMU] *** streaming will produce no samples ***");

  /* BLE setup */
  BLEDevice::init(DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new GaitServerCallbacks());

  gaitService = server->createService(GAIT_SERVICE_UUID);

  idChar = gaitService->createCharacteristic(ID_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ);
  cmdChar = gaitService->createCharacteristic(CMD_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  timeChar = gaitService->createCharacteristic(TIME_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  dataChar = gaitService->createCharacteristic(DATA_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  timeChar->addDescriptor(new BLE2902());
  dataChar->addDescriptor(new BLE2902());

  cmdChar->setCallbacks(new GaitCmdCallbacks());
  timeChar->setCallbacks(new GaitTimeCallbacks());
  idChar->setValue((uint8_t*)MODULE_ID, strlen(MODULE_ID));

  gaitService->start();
  advertising = server->getAdvertising();
  advertising->addServiceUUID(GAIT_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
  Serial.println("Advertising as " DEVICE_NAME " — waiting for a connection…");
}

void loop() {
  if (streaming) {
    /* 200 Hz sampling loop (back-to-back reads while behind schedule) */
    uint32_t now = millis();
    if ((int32_t)(now - nextSampleMs) > 100) nextSampleMs = now;
    while ((int32_t)(nextSampleMs - now) <= 0) {
      addSample(nextSampleMs);
      nextSampleMs += SAMPLE_PERIOD_MS;
    }
  }

#if MODULE_LED >= 0
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 500) {
    lastBlink = millis();
    digitalWrite(MODULE_LED, !digitalRead(MODULE_LED));
  }
#endif

  if (millis() - statLast >= 1000) {
    statLast = millis();
    Serial.print("[STAT] samples="); Serial.print(smpCount);
    Serial.print(" packets="); Serial.print(pktCount);
    Serial.print(" i2cErrors="); Serial.println(i2cErrors);
  }
  delay(1);
}
