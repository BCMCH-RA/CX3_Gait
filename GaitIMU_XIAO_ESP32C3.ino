/* ============================================================================
   GaitIMU XIAO ESP32-C3 — 200 Hz BLE IMU streamer (MPU-6050)
   ========================================================================== */
#include <Wire.h>
#include <string.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ───────── USER CONFIG — CHANGE THIS PER MODULE BEFORE FLASHING ───────── */
#define MODULE_ID "T" // Use "T" for Torso, "L" for LeftLeg, "R" for RightLeg
/* ───────────────────────────────────────────────────────────────────────── */
#define DEVICE_NAME "Gait-" MODULE_ID // Shortened to fit in 31-byte packet

#define I2C_SDA 6
#define I2C_SCL 7
#define I2C_SPEED 400000
#define MPU_ADDR 0x68

#define MODULE_LED -1
#define LED_ACTIVE_LOW 1

#define SAMPLE_RATE_HZ 200
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)
// Packet size cap. This environment delivers data over ATT READS, not
// notifications (on WinRT the ESP32's client bookkeeping stays empty, so
// notify() always no-ops with ERROR_NO_CLIENT — see BLECharacteristic.cpp:852).
// Bigger packets = fewer flushes/sec = the page's read-poll captures a higher
// fraction of the stream. A read needs the value to fit ONE ATT Read Response
// (negotiated MTU-1 = 516 B at MTU 517), so 42 samples = 510 B is the safe
// ceiling. The page raises spp per stream via CMD_SET_SPP.
#define MAX_SAMPLES_PER_PACKET 42
#define SAMPLES_PER_PACKET 15   // boot default (proven 186-B value); page may raise
#define MAX_PACKET_LEN (6 + 12 * MAX_SAMPLES_PER_PACKET)

#define GAIT_SERVICE_UUID "9f000001-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define CMD_CHAR_UUID     "9f000002-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define TIME_CHAR_UUID    "9f000003-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define DATA_CHAR_UUID    "9f000004-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define ID_CHAR_UUID      "9f000005-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define STAT_CHAR_UUID    "9f000006-8c5a-4e1f-9a7b-3d6e5f0a1b2c"

#define CMD_START   0x01
#define CMD_STOP    0x02
#define CMD_SET_SPP 0x03

#define REG_WHO_AM_I       0x75
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_SMPLRT_DIV     0x19
#define REG_PWR_MGMT_1     0x6B
#define REG_PWR_MGMT_2     0x6C
#define REG_ACCEL_XOUT_H   0x3B

#define VAL_CONFIG         0x01
#define VAL_GYRO_CONFIG    0x10
#define VAL_ACCEL_CONFIG   0x10
#define VAL_SMPLRT_DIV     4

static BLEService* gaitService = nullptr;
static BLECharacteristic* cmdChar = nullptr;
static BLECharacteristic* timeChar = nullptr;
static BLECharacteristic* dataChar = nullptr;
static BLECharacteristic* idChar = nullptr;
static BLECharacteristic* statChar = nullptr;
static BLEAdvertising* advertising = nullptr;
static bool deviceConnected = false;

bool streaming = false;
uint32_t syncMillis = 0;
uint32_t nextSampleMs = 0;
uint8_t samplesPerPacket = SAMPLES_PER_PACKET;
uint8_t packetSeq = 0;
uint8_t packetFill = 0;
uint32_t packetT0 = 0;
uint8_t packetBuf[MAX_PACKET_LEN];

uint32_t smpCount = 0;
uint32_t pktCount = 0;
uint32_t i2cErrors = 0;
uint32_t i2cFailStreak = 0;
uint32_t statLast = 0;

static BLEServer* bleServerPtr = nullptr;
static uint16_t lastConnId = 0;
static uint16_t peerMtu = 0;
uint32_t notifyErrCount = 0;
static uint8_t lastCmdByte = 0xFF;
static uint32_t lastNotifyCode = 0;

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
  uint8_t b[14];
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

static bool imuInit() {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_SPEED);
  delay(50);
  writeReg(REG_PWR_MGMT_1, 0x80);
  delay(150);
  
  uint8_t who = readReg(REG_WHO_AM_I);
  Serial.print("[IMU] WHO_AM_I = 0x");
  Serial.println(who, HEX);
  if (who != 0x68 && who != 0x70) {
    Serial.println("[IMU] *** MPU-6050 not seen on I2C — check SDA/SCL wiring ***");
    return false;
  }
  
  writeReg(REG_PWR_MGMT_1, 0x01);
  writeReg(REG_PWR_MGMT_2, 0x00);
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

static void flushPacket() {
  if (packetFill == 0) return;
  packetBuf[0] = packetSeq++;
  packetBuf[1] = packetFill;
  memcpy(&packetBuf[2], &packetT0, 4);
  dataChar->setValue(packetBuf, 6 + 12 * packetFill);
  // Notify is gate-off by default: on this Windows/WinRT stack the library's
  // connected-client list always reads empty (ERROR_NO_CLIENT at
  // BLECharacteristic.cpp:852 even though ATT reads/writes work), so notify()
  // is a guaranteed no-op that just churns the error counter. If a future
  // client ever registers properly (deviceConnected via stack reconcile), we
  // opportunistically notify as well. The page's READ-POLL fallback is the
  // reliable channel and always works.
  if (deviceConnected) dataChar->notify();
  packetFill = 0;
  pktCount++;
}

// Pick the largest samples-per-packet that fits the negotiated ATT MTU.
// Notification payload limit is MTU-3 bytes; each sample takes 12 bytes plus
// a 6-byte header. This makes the stream work at any MTU (e.g. 1 sample per
// packet is left even at MTU=23) while using the full 20 per packet at MTU>=249.
static void adaptSamplesPerPacket() {
  // No MTU info reached the app callbacks in this environment (peerMtu stays 0
  // even though WinRT negotiated ~517) — in that case keep whatever spp the
  // page set; don't stomp it with the MAX default.
  if (peerMtu < 3) return;
  uint16_t maxFit = MAX_SAMPLES_PER_PACKET;
  uint16_t payload = peerMtu - 3;
  if (payload >= 6) maxFit = (uint16_t)((payload - 6) / 12);
  if (maxFit < 1) maxFit = 1;
  if (maxFit > MAX_SAMPLES_PER_PACKET) maxFit = MAX_SAMPLES_PER_PACKET;
  if (maxFit == samplesPerPacket) return;
  if (streaming) flushPacket(); // end the current (possibly oversized) packet first
  samplesPerPacket = (uint8_t)maxFit;
  Serial.print("[MTU] effective samplesPerPacket="); Serial.print(samplesPerPacket);
  Serial.print(" (peerMTU="); Serial.print(peerMtu); Serial.println(")");
}

// Expose live firmware state to the browser (polled via the STATS
// characteristic) so the page can show what the device is doing without any
// Serial Monitor timing. Layout (18 bytes):
//   [0..3] notifyErrCount u32 LE   [4..5] peerMtu u16 LE
//   [6] samplesPerPacket           [7] streaming (0/1)
//   [8..11] pktCount u32 LE        [12] connected (0/1)
//   [13] lastCmdByte (0xFF = none) [14..17] lastNotifyErr code u32 LE
static void publishStats() {
  if (statChar == nullptr) return;
  uint8_t b[18];
  memcpy(b, &notifyErrCount, 4);
  b[4] = (uint8_t)(peerMtu & 0xFF);
  b[5] = (uint8_t)(peerMtu >> 8);
  b[6] = samplesPerPacket;
  b[7] = (uint8_t)(streaming ? 1 : 0);
  memcpy(&b[8], &pktCount, 4);
  b[12] = (uint8_t)(deviceConnected ? 1 : 0);
  b[13] = lastCmdByte;
  memcpy(&b[14], &lastNotifyCode, 4);
  statChar->setValue(b, 18);
}

static void addSample(uint32_t sampleMs) {
  int16_t acc[3], gyr[3];
  if (!hwReadSample(acc, gyr)) {
    i2cErrors++; i2cFailStreak++;
    if (i2cFailStreak > 50) {
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

class GaitServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    bleServerPtr = server;
    lastConnId = server->getConnId();
    Serial.print("[BLE] central connected conn=");
    // NOTE: at connect time this is still the pre-exchange default (23); the
    // real value arrives via onMtuChanged once the client performs the ATT MTU
    // exchange. CMD_START re-checks it as a fallback.
    uint16_t mtu = server->getPeerMTU(lastConnId);
    Serial.print(lastConnId, DEC);
    Serial.print(" initialPeerMTU=");
    Serial.println(mtu, DEC);
    if (mtu > 23) { peerMtu = mtu; adaptSamplesPerPacket(); }
  }
  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    streaming = false;
    packetFill = 0;
    peerMtu = 0;
    Serial.println("[BLE] central disconnected — advertising again");
    if (!advertising->start()) {
      Serial.println("[BLE] *** re-advertise FAILED after disconnect! ***");
    }
  }
  // Log the negotiated MTU whenever it changes and re-fit the packet size.
  // Data notifications must fit MTU-3 bytes — with BLEDevice::setMTU(517) and
  // a client that requests a large MTU, 20 samples/packet (246 bytes) fits.
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onMtuChanged(BLEServer*, esp_ble_gatts_cb_param_t* param) override {
    peerMtu = param->mtu.mtu;
    Serial.print("[BLE] peerMTU=");
    Serial.println(peerMtu, DEC);
    adaptSamplesPerPacket();
  }
#endif
#if defined(CONFIG_NIMBLE_ENABLED)
  void onMtuChanged(BLEServer*, ble_gap_conn_desc*, uint16_t mtu) override {
    peerMtu = mtu;
    Serial.print("[BLE] peerMTU=");
    Serial.println(peerMtu, DEC);
    adaptSamplesPerPacket();
  }
#endif
};

// Reports every notification outcome. A 246-byte notification rejected because
// it exceeds the negotiated MTU surfaces here as ERROR_GATT with an ATT error
// code (e.g. ESP_GATT_INVALID_ATT_LEN) — the definitive proof of an MTU issue.
class GaitDataCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic*, Status s, uint32_t code) override {
    if (s == SUCCESS_NOTIFY) return;
    notifyErrCount++;
    lastNotifyCode = code;
    if (notifyErrCount == 1 || notifyErrCount % 500 == 0) {
      Serial.print("[DATA] notify FAIL #"); Serial.print(notifyErrCount, DEC);
      Serial.print(" status="); Serial.print((int)s, DEC);
      Serial.print(" code=0x"); Serial.println(code, HEX);
    }
  }
};

class GaitCmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    // Raw dump of every write received on the command characteristic — lets us
    // prove whether the page's CMD_START (0x01) write actually arrives here.
    Serial.print("[CMD] write len=");
    Serial.print(v.length(), DEC);
    if (v.length() > 0) {
      Serial.print(" b0=0x");
      Serial.println((uint8_t)v[0], HEX);
      if (v.length() > 1) {
        Serial.print("[CMD] .. b1=0x");
        Serial.println((uint8_t)v[1], HEX);
      }
    } else {
      Serial.println();
    }
    if (v.length() == 0) return;
    lastCmdByte = (uint8_t)v[0];
    switch ((uint8_t)v[0]) {
      case CMD_START:
        if (!streaming) {
          // Re-check the negotiated MTU right before streaming so packets are
          // sized to fit whatever MTU the client actually negotiated.
          if (bleServerPtr != nullptr && lastConnId != 0) {
            uint16_t mtu = bleServerPtr->getPeerMTU(lastConnId);
            if (mtu >= 3) { peerMtu = mtu; adaptSamplesPerPacket(); }
          }
          packetFill = 0; packetSeq = 0;
          nextSampleMs = millis() + 2;
          streaming = true;
          Serial.print("[CMD] streaming started (samplesPerPacket=");
          Serial.print(samplesPerPacket, DEC);
          Serial.println(")");
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
    if (v.length() != 6) return;
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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("GaitIMU XIAO ESP32-C3 node: " DEVICE_NAME);
  Serial.println("[VER] firmware build R9 — spp up to 42 (510-B reads), page-set spp preserved when no MTU info");
  Serial.print("[CONF] samples/packet max = ");
  Serial.print(MAX_SAMPLES_PER_PACKET, DEC);
  Serial.print(" (notify payload ");
  Serial.print(6 + 12 * MAX_SAMPLES_PER_PACKET, DEC);
  Serial.println(" B) — fits one ATT Read Response; page sets spp per stream via CMD_SET_SPP");

#if MODULE_LED >= 0
  pinMode(MODULE_LED, OUTPUT);
  digitalWrite(MODULE_LED, LED_ACTIVE_LOW ? HIGH : LOW);
#endif

  bool imuOk = imuInit();
  if (!imuOk) Serial.println("[IMU] *** streaming will produce no samples ***");

  BLEDevice::init(DEVICE_NAME);
  // Raise the local MTU so reads of large packet values survive the ATT
  // exchange. Reads are the data channel in this environment (notify is DOA),
  // and MAX_SAMPLES_PER_PACKET (42 → 510 B value) still fits one 517-MTU read
  // response. The page sets the actual spp per stream via CMD_SET_SPP.
  BLEDevice::setMTU(517);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new GaitServerCallbacks());

  gaitService = server->createService(GAIT_SERVICE_UUID);

  idChar = gaitService->createCharacteristic(ID_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
  cmdChar = gaitService->createCharacteristic(CMD_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  timeChar = gaitService->createCharacteristic(TIME_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  dataChar = gaitService->createCharacteristic(DATA_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  timeChar->addDescriptor(new BLE2902());
  dataChar->addDescriptor(new BLE2902());

  statChar = gaitService->createCharacteristic(STAT_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
  publishStats();

  cmdChar->setCallbacks(new GaitCmdCallbacks());
  timeChar->setCallbacks(new GaitTimeCallbacks());
  dataChar->setCallbacks(new GaitDataCallbacks());
  idChar->setValue((uint8_t*)MODULE_ID, strlen(MODULE_ID));

  gaitService->start();
  advertising = server->getAdvertising();

  // FIX (discovery): advertise the service UUID and name from the PRIMARY
  // packet only — no separate scan response. BLEAdvertising::addServiceUUID()
  // puts the 128-bit UUID in the primary advertisement, and the device name
  // set by BLEDevice::init() is auto-included because it fits:
  // flags(3) + UUID128(18) + name("Gait-X", 8) = 29 of 31 bytes.
  //
  // Do NOT call setScanResponse(true) here (it was in the previous version):
  // it makes the stack move the name into the SCAN-RESPONSE packet. Android/
  // Windows scan actively, fetch scan responses and showed "Gait-T"; but Web
  // Bluetooth on Windows relies on the OS BLE watcher, which often never
  // receives scan responses — so Chrome's `name:` filter matched nothing.
  // The page now filters on the service UUID, which is visible to every
  // scanner regardless of scan-response handling.
  advertising->addServiceUUID(GAIT_SERVICE_UUID);
  if (advertising->start()) {
    Serial.println("Advertising as " DEVICE_NAME " — waiting for a connection…");
  } else {
    Serial.println("[BLE] *** advertising START FAILED — device will NOT be discoverable! Power-cycle and retry. ***");
  }
}

void loop() {
  // Connection-state reconcile, driven by the library's OWN GATT connection
  // counter instead of the app-level connect/disconnect callbacks (which have
  // proven unreliable: sessions where the page was paired and reading stats
  // while the firmware still reported conn=0). Reading the stack's truth every
  // loop means a missed callback can no longer wedge the module in a
  // "connected in spirit, never connected in fact" state.
  static bool prevAttached = false;
  if (bleServerPtr != nullptr) {
    bool attached = bleServerPtr->getConnectedCount() > 0;
    if (attached != deviceConnected) {
      deviceConnected = attached;
      Serial.print("[BLE] stack reconcile: attached=");
      Serial.println(attached ? 1 : 0);
    }
    if (!attached && prevAttached) {
      // The stack says the link dropped. Re-advertise so the module stays
      // discoverable — but do NOT stop streaming: connection bookkeeping is
      // unreliable in this environment (counts flap while reads/writes keep
      // working), and the page drives streaming via CMD_START/CMD_STOP.
      prevAttached = false;
      peerMtu = 0;
      if (advertising != nullptr && !advertising->start()) {
        Serial.println("[BLE] *** re-advertise FAILED (stack reconcile) ***");
      }
    } else if (attached && !prevAttached) {
      prevAttached = true;
      // Fresh attach: pick up the connection id once and grab the MTU in case
      // the connect/MTU events never reached our callbacks. If MTU < 24 here,
      // the client hasn't exchanged yet — CMD_START re-checks it at stream start.
      if (lastConnId == 0) {
        lastConnId = bleServerPtr->getConnId();
        uint16_t mtu = bleServerPtr->getPeerMTU(lastConnId);
        if (mtu > 23) { peerMtu = mtu; adaptSamplesPerPacket(); }
      }
    }
  }

  if (streaming) {
    uint32_t now = millis();
    if ((int32_t)(now - nextSampleMs) > 100) nextSampleMs = now;
    while ((int32_t)(nextSampleMs - now) <= 0) {
      addSample(nextSampleMs);
      nextSampleMs += SAMPLE_PERIOD_MS;
    }
  }

  if (millis() - statLast >= 1000) {
    statLast = millis();
    publishStats();
    Serial.print("[STAT] samples="); Serial.print(smpCount);
    Serial.print(" packets="); Serial.print(pktCount);
    Serial.print(" i2cErrors="); Serial.print(i2cErrors);
    Serial.print(" streaming="); Serial.println(streaming ? 1 : 0);
  }
  delay(1);
}