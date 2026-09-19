/*
 * Seeed nRF52840 Sense IMU BLE Streamer for Gait Assessment Platform
 *
 * - Streams LSM6DS3 IMU data (200 Hz) over BLE via ArduinoBLE in batches
 * - Time synced from browser via BLE (Unix epoch -> IST UTC+5:30)
 * - Optimized packet transmission: 8 samples/packet @ 25 packets/sec = 200 Hz
 * - Battery monitoring via LC709203F
 * - Device roles: Right Leg, Left Leg, Torso (set via DEVICE_ROLE)
 *
 * Hardware: Seeed nRF52840 Sense (LSM6DS3 IMU, LC709203F Battery Monitor)
 * Board: Seeed nRF52840 Sense (Arduino Board Manager: Seeed nRF52 Boards)
 * Libraries:
 *   - ArduinoBLE (built-in)
 *   - LSM6DS3 (Arduino Library Manager)
 *   - Adafruit LC709203F (Arduino Library Manager)
 */

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Adafruit_LC709203F.h>
#include <Wire.h>

// ─── Device Role Configuration ──────────────────────────────────────
// Set this per device: 0 = Right Leg, 1 = Left Leg, 2 = Torso
#define DEVICE_ROLE 0

#if DEVICE_ROLE == 0
  #define DEVICE_NAME "Gait-RightLeg"
  #define ROLE_STR "Right Leg"
#elif DEVICE_ROLE == 1
  #define DEVICE_NAME "Gait-LeftLeg"
  #define ROLE_STR "Left Leg"
#else
  #define DEVICE_NAME "Gait-Torso"
  #define ROLE_STR "Torso"
#endif

// ─── BLE UUIDs ──────────────────────────────────────────────────────
const char* SERVICE_UUID      = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* IMU_CHAR_UUID     = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* CMD_CHAR_UUID     = "19b10002-e8f2-537e-4f6c-d104768a1214";
const char* STATUS_CHAR_UUID  = "19b10003-e8f2-537e-4f6c-d104768a1214";
const char* BATTERY_CHAR_UUID = "19b10004-e8f2-537e-4f6c-d104768a1214";

// ─── BLE Command Bytes ──────────────────────────────────────────────
#define CMD_START_RECORD      0x01
#define CMD_STOP_RECORD       0x02
#define CMD_SYNC_TIME         0x03
#define CMD_GET_STATUS        0x04

// ─── Packet Format ──────────────────────────────────────────────────
#define IMU_SAMPLES_PER_PACKET  8
#define PACKET_MAGIC           0xBEEF
#define PKT_TYPE_IMU           0x01
#define PKT_TYPE_STATUS        0x02
#define PKT_TYPE_BATTERY       0x03

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;
    uint16_t seq;
    uint8_t  pkt_type;
    uint8_t  device_role;
    uint32_t ts_ms;
    struct {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
    } samples[IMU_SAMPLES_PER_PACKET];
} ImuPacket;

typedef struct {
    uint16_t magic;
    uint16_t seq;
    uint8_t  pkt_type;
    uint8_t  device_role;
    uint32_t ts_ms;
    uint32_t epoch_sec;
    uint8_t  battery_percent;
    float    battery_voltage;
    bool     recording;
    uint32_t total_samples;
} StatusPacket;

typedef struct {
    uint16_t magic;
    uint16_t seq;
    uint8_t  pkt_type;
    uint8_t  device_role;
    uint32_t ts_ms;
    uint8_t  battery_percent;
    float    battery_voltage;
} BatteryPacket;
#pragma pack(pop)

// ─── IMU Scaling (LSM6DS3 defaults: 4g accel, 2000 dps gyro) ────────
#define ACCEL_LSB_PER_G    (32768.0f / 4.0f)      // 4G range
#define GYRO_LSB_PER_DPS   (32768.0f / 2000.0f)   // 2000 dps range

// ─── Ring Buffer (DROP_OLDEST, never blocks producer) ───────────────
#define RING_SIZE 1024

typedef struct {
    int16_t ax, ay, az, gx, gy, gz;
    uint32_t ts_ms;
    uint32_t epoch_ms;
} SampleEntry;

typedef struct {
    SampleEntry buf[RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} SampleRingBuffer;

static SampleRingBuffer bleRing;

// ─── Global State ───────────────────────────────────────────────────
BLEService imuService(SERVICE_UUID);
BLECharacteristic imuChar(IMU_CHAR_UUID, BLERead | BLENotify, sizeof(ImuPacket));
BLECharacteristic cmdChar(CMD_CHAR_UUID, BLEWriteWithoutResponse, 5);
BLECharacteristic statusChar(STATUS_CHAR_UUID, BLERead | BLENotify, sizeof(StatusPacket));
BLECharacteristic batteryChar(BATTERY_CHAR_UUID, BLERead | BLENotify, sizeof(BatteryPacket));

static volatile bool bleConnected = false;
static volatile bool recording = false;
static uint16_t bleSeq = 0;
static volatile uint32_t totalSamples = 0;

static uint32_t bleTimeOffset = 0;
static uint32_t bleTimeMillis = 0;
static bool timeSet = false;

static LSM6DS3 imu(I2C_MODE, 0x6A);
static Adafruit_LC709203F fuelGauge;

// ─── BLE Event Handlers (global functions for mbed core) ────────────
void onBleConnect(BLEDevice central) {
    bleConnected = true;
    Serial.print("BLE connected: ");
    Serial.println(central.address());
    BLE.setConnectionInterval(6, 6);
}

void onBleDisconnect(BLEDevice central) {
    bleConnected = false;
    if (recording) {
        recording = false;
        Serial.println("Recording stopped due to disconnect");
    }
    Serial.println("BLE disconnected, restarting advertising");
    BLE.advertise();
}

void onCmdWrite(BLEDevice central, BLECharacteristic characteristic) {
    uint8_t len = characteristic.valueLength();
    if (len < 1) return;
    const uint8_t* data = characteristic.value();
    uint8_t cmd = data[0];

    switch (cmd) {
        case CMD_START_RECORD:
            if (!recording) {
                ringClear(&bleRing);
                bleSeq = 0;
                recording = true;
                totalSamples = 0;
                Serial.println("CMD: Start recording");
            }
            break;
        case CMD_STOP_RECORD:
            if (recording) {
                recording = false;
                Serial.println("CMD: Stop recording");
            }
            break;
        case CMD_SYNC_TIME:
            if (len >= 5) {
                uint32_t epoch;
                memcpy(&epoch, &data[1], 4);
                bleTimeOffset = epoch;
                bleTimeMillis = millis();
                timeSet = true;
                Serial.print("Time synced: ");
                Serial.println(epoch);
            }
            break;
        case CMD_GET_STATUS:
            sendStatusPacket();
            break;
    }
}

// ─── IST Time (UTC+5:30) ────────────────────────────────────────────
static uint32_t getCurrentEpoch(void) {
    if (timeSet && bleTimeOffset > 0) {
        return bleTimeOffset + (millis() - bleTimeMillis) / 1000;
    }
    return 0;
}

static void formatIST(char* buf, size_t len) {
    uint32_t epoch = getCurrentEpoch();
    if (epoch < 1000000000) {
        snprintf(buf, len, "IST: --:--:--");
        return;
    }
    uint32_t istEpoch = epoch + 19800;
    time_t t = (time_t)istEpoch;
    struct tm* tm_info = gmtime(&t);
    static const char* dayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    snprintf(buf, len, "IST: %s %02d:%02d:%02d",
             dayNames[tm_info->tm_wday],
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
}

// ─── Ring Buffer Operations ─────────────────────────────────────────
static void ringInit(SampleRingBuffer* r) {
    r->head = 0; r->tail = 0; r->count = 0;
}

static bool ringPush(SampleRingBuffer* r, SampleEntry* s) {
    if (r->count >= RING_SIZE) {
        r->tail = (r->tail + 1) % RING_SIZE;
        r->count--;
    }
    r->buf[r->head] = *s;
    r->head = (r->head + 1) % RING_SIZE;
    r->count++;
    return true;
}

static void ringClear(SampleRingBuffer* r) {
    r->head = 0; r->tail = 0; r->count = 0;
}

static bool ringPop(SampleRingBuffer* r, SampleEntry* s) {
    if (r->count == 0) return false;
    *s = r->buf[r->tail];
    r->tail = (r->tail + 1) % RING_SIZE;
    r->count--;
    return true;
}

static uint16_t ringCount(SampleRingBuffer* r) {
    return r->count;
}

// ─── Send Status Packet ─────────────────────────────────────────────
static void sendStatusPacket() {
    if (!bleConnected) return;
    StatusPacket pkt;
    pkt.magic = PACKET_MAGIC;
    pkt.seq = bleSeq++;
    pkt.pkt_type = PKT_TYPE_STATUS;
    pkt.device_role = DEVICE_ROLE;
    pkt.ts_ms = millis();
    pkt.epoch_sec = getCurrentEpoch();
    pkt.battery_percent = fuelGauge.cellPercent();
    pkt.battery_voltage = fuelGauge.cellVoltage();
    pkt.recording = recording;
    pkt.total_samples = totalSamples;
    statusChar.writeValue((uint8_t*)&pkt, sizeof(pkt));
}

// ─── Send Battery Packet ────────────────────────────────────────────
static void sendBatteryPacket() {
    if (!bleConnected) return;
    BatteryPacket pkt;
    pkt.magic = PACKET_MAGIC;
    pkt.seq = bleSeq++;
    pkt.pkt_type = PKT_TYPE_BATTERY;
    pkt.device_role = DEVICE_ROLE;
    pkt.ts_ms = millis();
    pkt.battery_percent = fuelGauge.cellPercent();
    pkt.battery_voltage = fuelGauge.cellVoltage();
    batteryChar.writeValue((uint8_t*)&pkt, sizeof(pkt));
}

// ─── BLE Initialization ─────────────────────────────────────────────
static void BLE_Init(void) {
    if (!BLE.begin()) {
        Serial.println("BLE init failed!");
        while (1);
    }

    BLE.setDeviceName(DEVICE_NAME);
    BLE.setLocalName(DEVICE_NAME);
    BLE.setAdvertisedService(imuService);

    imuService.addCharacteristic(imuChar);
    imuService.addCharacteristic(cmdChar);
    imuService.addCharacteristic(statusChar);
    imuService.addCharacteristic(batteryChar);

    BLE.addService(imuService);

    BLE.setEventHandler(BLEConnected, onBleConnect);
    BLE.setEventHandler(BLEDisconnected, onBleDisconnect);
    cmdChar.setEventHandler(BLEWritten, onCmdWrite);

    BLE.advertise();

    Serial.print("BLE started as '");
    Serial.print(DEVICE_NAME);
    Serial.println("'");
}

// ─── IMU Task (200 Hz) ──────────────────────────────────────────────
static void imuTask(void) {
    static uint32_t lastSample = 0;
    const uint32_t sampleInterval = 5;

    uint32_t now = micros();
    if (now - lastSample >= sampleInterval * 1000) {
        lastSample = now;

        int16_t ax = imu.readRawAccelX();
        int16_t ay = imu.readRawAccelY();
        int16_t az = imu.readRawAccelZ();
        int16_t gx = imu.readRawGyroX();
        int16_t gy = imu.readRawGyroY();
        int16_t gz = imu.readRawGyroZ();

        uint32_t now_ms = millis();
        uint32_t epoch_ms = 0;
        if (timeSet && bleTimeOffset > 0) {
            epoch_ms = (bleTimeOffset * 1000) + (now_ms - bleTimeMillis);
        }

        SampleEntry entry;
        entry.ax = ax; entry.ay = ay; entry.az = az;
        entry.gx = gx; entry.gy = gy; entry.gz = gz;
        entry.ts_ms = now_ms;
        entry.epoch_ms = epoch_ms;

        ringPush(&bleRing, &entry);
        totalSamples++;
    }
}

// ─── BLE Notify Task (~25 Hz, 8 samples/packet = 200 Hz) ────────────
static void bleNotifyTask(void) {
    static uint32_t lastNotify = 0;
    const uint32_t notifyInterval = 40;

    if (!bleConnected || !recording) return;

    uint32_t now = millis();
    if (now - lastNotify < notifyInterval) return;
    lastNotify = now;

    SampleEntry batch[IMU_SAMPLES_PER_PACKET];
    uint8_t count = 0;
    while (count < IMU_SAMPLES_PER_PACKET && ringPop(&bleRing, &batch[count])) {
        count++;
    }

    if (count == 0) return;

    ImuPacket pkt;
    pkt.magic = PACKET_MAGIC;
    pkt.seq = bleSeq++;
    pkt.pkt_type = PKT_TYPE_IMU;
    pkt.device_role = DEVICE_ROLE;
    pkt.ts_ms = batch[0].ts_ms;

    for (uint8_t i = 0; i < count; i++) {
        pkt.samples[i].ax = batch[i].ax;
        pkt.samples[i].ay = batch[i].ay;
        pkt.samples[i].az = batch[i].az;
        pkt.samples[i].gx = batch[i].gx;
        pkt.samples[i].gy = batch[i].gy;
        pkt.samples[i].gz = batch[i].gz;
    }

    imuChar.writeValue((uint8_t*)&pkt, 8 + (size_t)count * 12);
}

// ─── Battery Monitor Task (1 Hz) ────────────────────────────────────
static void batteryTask(void) {
    static uint32_t lastBattery = 0;
    if (millis() - lastBattery >= 1000) {
        lastBattery = millis();
        sendBatteryPacket();
    }
}

// ─── Setup ──────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    Serial.println("\n=== Seeed nRF52840 Sense IMU BLE Streamer ===");
    Serial.print("Device Role: ");
    Serial.println(ROLE_STR);

    // I2C for IMU and Battery
    Wire.begin();
    Wire.setClock(400000);

    // Initialize IMU
    if (imu.begin() != 0) {
        Serial.println("IMU init failed!");
        while (1);
    }
    imu.settings.accelRange = 4;
    imu.settings.gyroRange = 2000;
    imu.settings.accelSampleRate = 200;
    imu.settings.gyroSampleRate = 200;
    Serial.println("IMU initialized: 4g, 2000dps, 200Hz");

    // Initialize Battery Monitor
    if (!fuelGauge.begin()) {
        Serial.println("Battery monitor init failed!");
        while (1);
    }
    fuelGauge.setPackSize(LC709203F_APA_500MAH);
    fuelGauge.setAlarmVoltage(3.3);
    Serial.println("Battery monitor initialized");

    // Initialize Ring Buffer
    ringInit(&bleRing);

    // Initialize BLE
    BLE_Init();

    Serial.println("Setup complete. Advertising...");
}

// ─── Main Loop ──────────────────────────────────────────────────────
void loop() {
    BLE.poll();

    imuTask();
    bleNotifyTask();
    batteryTask();

    // Periodic status send (every 5 seconds)
    static uint32_t lastStatus = 0;
    if (bleConnected && millis() - lastStatus >= 5000) {
        lastStatus = millis();
        sendStatusPacket();
    }
}