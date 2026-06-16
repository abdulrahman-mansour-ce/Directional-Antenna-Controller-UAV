#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// ─── Pin Definitions ──────────────────────────────────
#define BMP_SDA 8
#define BMP_SCL 9
#define MMC_SDA 4
#define MMC_SCL 5
#define GPS_RX  16
#define GPS_TX  17
#define SWITCH_A  38
#define SWITCH_B  39
#define SEALEVELPRESSURE_HPA 1013.25

// ─── Timing Definitions ───────────────────────────────
// Sensor data is refreshed every 100 ms.
// Antenna switching is checked every 100 ms.
// Bluetooth/Serial report is sent every 5000 ms.
#define SENSOR_PERIOD_MS 100
#define SWITCH_PERIOD_MS 100
#define REPORT_PERIOD_MS 5000

// ─── BLE UUIDs ────────────────────────────────────────
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ─── Sensor Objects ───────────────────────────────────
Adafruit_BMP3XX bmp;
SFE_MMC5983MA   mag;
TinyGPSPlus     gps;
HardwareSerial  gpsSerial(1);
TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

// ─── BLE Objects ──────────────────────────────────────
BLEServer*         pServer           = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;

// ─── Setup State Machine ──────────────────────────────
enum SetupState { WAIT_CONNECT, CALIBRATING, WAIT_GCS, RUNNING };
SetupState setupState = WAIT_CONNECT;

// ─── Persistent Storage ───────────────────────────────
Preferences prefs;

// ─── Antenna Offset ───────────────────────────────────
float BOARD_OFFSET = 0.0;

// ─── Antenna Hysteresis ───────────────────────────────
#define HYSTERESIS_DEG 5.0f
String currentAntenna = "FRONT";

// ─── Shared Data Struct ───────────────────────────────
struct SensorData {
  float heading;
  float boardLat;
  float boardLon;
  float temperature;
  float altitude;
  float bearingToGCS;
  float relativeAngle;
  bool  gpsValid;
};

// ─── GCS Location ─────────────────────────────────────
float GCS_LAT = 0.0;
float GCS_LON = 0.0;
bool  gcsSet  = false;

// ─── Timing Variables ─────────────────────────────────
unsigned long lastSendTime   = 0;   // Used for calibration heading messages
unsigned long lastSensorTime = 0;   // Used for sensor reading
unsigned long lastSwitchTime = 0;   // Used for fast antenna switching
unsigned long lastReportTime = 0;   // Used for slow Bluetooth/Serial report

SensorData data;


// ═════════════════════════════════════════════════════
//  FUNCTION 1 — COMMAND HANDLER
//  Input : Bluetooth link, messages containing GCS location update
//  Output: Command output, location of GCS
// ═════════════════════════════════════════════════════

void bleSend(String msg) {
  if (deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue(msg.c_str());
    pTxCharacteristic->notify();
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    setupState = CALIBRATING;

    Serial.println("[BLE] Phone connected — starting calibration.");

    delay(300);

    bleSend("=== STEP 1: COMPASS CALIBRATION ===");
    bleSend("Restored offset from last session: " + String(BOARD_OFFSET, 1) + " deg");
    bleSend("Slowly rotate the board 360 degrees.");
    bleSend("Send a number like -3 or +5 to adjust offset.");
    bleSend("Reply OK when heading looks correct.");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    setupState = WAIT_CONNECT;

    Serial.println("[BLE] Disconnected — offset preserved. Restarting advertising...");

    pServer->startAdvertising();
  }
};

bool parseGCS(String incoming, float& lat, float& lon) {
  int commaIndex = incoming.indexOf(',');

  if (commaIndex == -1) return false;

  lat = incoming.substring(0, commaIndex).toFloat();
  lon = incoming.substring(commaIndex + 1).toFloat();

  if (lat == 0.0 && lon == 0.0) return false;
  if (lat < -90.0 || lat > 90.0) return false;
  if (lon < -180.0 || lon > 180.0) return false;

  return true;
}

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String incoming = pCharacteristic->getValue().c_str();
    incoming.trim();

    // ── Step 1: Calibration ───────────────────────────
    if (setupState == CALIBRATING) {
      if (incoming.equalsIgnoreCase("OK")) {
        setupState = WAIT_GCS;

        bleSend("Calibration confirmed! Final offset: " + String(BOARD_OFFSET, 1) + " deg");

        delay(200);

        bleSend("=== STEP 2: GCS LOCATION ===");
        bleSend("Send ground station coordinates as: LAT,LON");
        bleSend("Example: 24.688000,46.722000");

        if (gcsSet) {
          bleSend("Previous GCS still loaded: " + String(GCS_LAT, 6) + "," + String(GCS_LON, 6));
          bleSend("Send KEEP to reuse it, or send new LAT,LON.");
        }
      }
      else if (incoming.equalsIgnoreCase("KEEP")) {
        bleSend("KEEP is only valid after calibration is confirmed. Send OK first.");
      }
      else {
        bool isZero = (incoming == "0" || incoming == "0.0" ||
                       incoming == "+0" || incoming == "-0");

        float adjustment = incoming.toFloat();

        if (adjustment != 0.0 || isZero) {
          BOARD_OFFSET = fmod(BOARD_OFFSET + adjustment + 360.0, 360.0);

          prefs.putFloat("offset", BOARD_OFFSET);

          bleSend("Offset adjusted by " + String(adjustment, 1) + " deg");
          bleSend("New offset: " + String(BOARD_OFFSET, 1) + " deg");
          bleSend("Keep adjusting or reply OK to confirm.");
        }
        else {
          bleSend("Not recognised. Send a number like -3, or OK to confirm.");
        }
      }

      return;
    }

    // ── Step 2: GCS Coordinates ───────────────────────
    if (setupState == WAIT_GCS) {
      if (incoming.equalsIgnoreCase("KEEP") && gcsSet) {
        setupState = RUNNING;

        bleSend("Reusing GCS: " + String(GCS_LAT, 6) + ", " + String(GCS_LON, 6));
        bleSend("=== SETUP COMPLETE — Antenna tracking started ===");

        return;
      }

      float lat, lon;

      if (parseGCS(incoming, lat, lon)) {
        GCS_LAT = lat;
        GCS_LON = lon;
        gcsSet  = true;
        setupState = RUNNING;

        String confirm = "GCS SET: " + String(GCS_LAT, 6) + ", " + String(GCS_LON, 6);

        Serial.println(confirm);
        bleSend(confirm);
        bleSend("=== SETUP COMPLETE — Antenna tracking started ===");
      }
      else {
        bleSend("ERROR: Invalid coordinates. Format must be LAT,LON (e.g. 24.688000,46.722000)");
      }

      return;
    }

    // ── Running: Allow GCS Update ─────────────────────
    if (setupState == RUNNING) {
      if (incoming == "-1") {
        setupState = WAIT_GCS;

        bleSend("=== GCS UPDATE ===");
        bleSend("Send new coordinates as: LAT,LON");
        bleSend("Example: 24.688000,46.722000");
        bleSend("Or send KEEP to keep current: " + String(GCS_LAT, 6) + "," + String(GCS_LON, 6));

        return;
      }

      float lat, lon;

      if (parseGCS(incoming, lat, lon)) {
        GCS_LAT = lat;
        GCS_LON = lon;

        String confirm = "GCS UPDATED: " + String(GCS_LAT, 6) + ", " + String(GCS_LON, 6);

        Serial.println(confirm);
        bleSend(confirm);
      }
      else {
        bleSend("ERROR: Format must be LAT,LON (e.g. 24.688000,46.722000)");
      }
    }
  }
};

void setupBluetooth() {
  BLEDevice::init("ESP32-S3_BLE");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("[BLE] Started — Device: ESP32-S3_BLE");
}


// ═════════════════════════════════════════════════════
//  FUNCTION 2 — GEO LOCATION STATE MEASUREMENT
//  Input : GPS data, heading data, altitude data
//  Output: UAV_state: lat, lon, altitude, heading
// ═════════════════════════════════════════════════════

#define EMA_ALPHA 0.50f

float emaX      = 0.0f;
float emaY      = 0.0f;
bool  emaSeeded = false;

SensorData readSensors() {
  SensorData d = {};

  if (bmp.performReading()) {
    d.temperature = bmp.temperature;
    d.altitude    = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }

  uint32_t rawX = 0, rawY = 0, rawZ = 0;

  mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

  float magX = ((float)rawX - 131072.0f) / 16384.0f;
  float magY = ((float)rawY - 131072.0f) / 16384.0f;

  if (!emaSeeded) {
    emaX      = magX;
    emaY      = magY;
    emaSeeded = true;
  }
  else {
    emaX = EMA_ALPHA * magX + (1.0f - EMA_ALPHA) * emaX;
    emaY = EMA_ALPHA * magY + (1.0f - EMA_ALPHA) * emaY;
  }

  float heading = degrees(atan2(emaY, emaX));

  if (heading < 0) {
    heading += 360.0f;
  }

  d.heading = fmod(heading - BOARD_OFFSET + 360.0f, 360.0f);

  d.gpsValid = gps.location.isValid();

  if (d.gpsValid) {
    d.boardLat = gps.location.lat();
    d.boardLon = gps.location.lng();
  }

  return d;
}


// ═════════════════════════════════════════════════════
//  FUNCTION 3 — COMPUTATION ALGORITHM
//  Input : UAV_state + GCS coordinates
//  Output: Relative deg
// ═════════════════════════════════════════════════════

float computeRelativeDeg(SensorData& d) {
  if (!gcsSet) {
    d.relativeAngle = -1.0f;
    return d.relativeAngle;
  }

  if (!d.gpsValid) {
    d.relativeAngle = -2.0f;
    return d.relativeAngle;
  }

  float dLon = radians(GCS_LON - d.boardLon);
  float lat1 = radians(d.boardLat);
  float lat2 = radians(GCS_LAT);

  float x = sin(dLon) * cos(lat2);

  float y = cos(lat1) * sin(lat2) -
            sin(lat1) * cos(lat2) * cos(dLon);

  d.bearingToGCS = degrees(atan2(x, y));

  if (d.bearingToGCS < 0) {
    d.bearingToGCS += 360.0f;
  }

  d.relativeAngle = fmod(d.bearingToGCS - d.heading + 360.0f, 360.0f);

  return d.relativeAngle;
}


// ═════════════════════════════════════════════════════
//  FUNCTION 4 — ANTENNA SELECTION SUBSYSTEM
//  Input : Relative deg
//  Output: Selection line 1 and Selection line 2
// ═════════════════════════════════════════════════════

void applyAntenna(const String& sector) {
  if (sector == "FRONT") {
    digitalWrite(SWITCH_A, LOW);
    digitalWrite(SWITCH_B, LOW);
  }
  else if (sector == "RIGHT") {
    digitalWrite(SWITCH_A, HIGH);
    digitalWrite(SWITCH_B, LOW);
  }
  else if (sector == "BACK") {
    digitalWrite(SWITCH_A, LOW);
    digitalWrite(SWITCH_B, HIGH);
  }
  else {
    digitalWrite(SWITCH_A, HIGH);
    digitalWrite(SWITCH_B, HIGH);
  }
}

String rawSector(float relativeDeg) {
  if (relativeDeg >= 315.0f || relativeDeg < 45.0f) {
    return "FRONT";
  }

  if (relativeDeg < 135.0f) {
    return "RIGHT";
  }

  if (relativeDeg < 225.0f) {
    return "BACK";
  }

  return "LEFT";
}

float sectorCentre(const String& sector) {
  if (sector == "FRONT") return 0.0f;
  if (sector == "RIGHT") return 90.0f;
  if (sector == "BACK")  return 180.0f;

  return 270.0f;
}

String antennaSelectionSubsystem(float relativeDeg) {
  if (relativeDeg == -1.0f) {
    return "NO_GCS";
  }

  if (relativeDeg == -2.0f) {
    return "NO_GPS";
  }

  String candidate = rawSector(relativeDeg);

  if (candidate != currentAntenna) {
    float centre = sectorCentre(candidate);

    float fromCentre = fmod(fabs(relativeDeg - centre) + 360.0f, 360.0f);

    if (fromCentre > 180.0f) {
      fromCentre = 360.0f - fromCentre;
    }

    float penetration = 45.0f - fromCentre;

    if (penetration >= HYSTERESIS_DEG) {
      currentAntenna = candidate;
      applyAntenna(currentAntenna);
    }
  }

  return currentAntenna;
}


// ═════════════════════════════════════════════════════
//  OUTPUT
//  Input : UAV_state + selected antenna
//  Output: Command output over Bluetooth / Serial
// ═════════════════════════════════════════════════════

void sendOutput(SensorData& d, const String& antenna) {
  String msg;

  if (antenna == "NO_GCS") {
    msg = "Waiting for GCS coordinates...";
  }
  else if (antenna == "NO_GPS") {
    msg = "No GPS fix yet...";
  }
  else {
    msg += "Antenna : " + antenna                      + "\n";
    msg += "Heading : " + String(d.heading,       1)   + " deg\n";
    msg += "Bearing : " + String(d.bearingToGCS,  1)   + " deg\n";
    msg += "Rel Ang : " + String(d.relativeAngle, 1)   + " deg\n";
    msg += "Lat     : " + String(d.boardLat,      6)   + "\n";
    msg += "Lon     : " + String(d.boardLon,      6)   + "\n";
    msg += "Temp    : " + String(d.temperature,   2)   + " C\n";
    msg += "Alt     : " + String(d.altitude,      2)   + " m\n";
  }

  Serial.println(msg);
  bleSend(msg);
}


// ═════════════════════════════════════════════════════
//  MAIN LOOP FUNCTION 1 — READING BLUETOOTH
//  Module : Command Handler
//  Input  : Bluetooth link / command input
//  Output : Command output + location of GCS
// ═════════════════════════════════════════════════════

void CommandHandler() {
  if (setupState == CALIBRATING && millis() - lastSendTime >= 1000) {
    lastSendTime = millis();

    bleSend("Heading: " + String(data.heading, 1) +
            " deg  |  Send a number to adjust or OK to confirm");
  }
}


// ═════════════════════════════════════════════════════
//  MAIN LOOP FUNCTION 2 — READING DATA SENSOR
//  Module : Geo location state measurement
//  Input  : GPS data, heading data, altitude data
//  Output : UAV_state: lat, lon, altitude, heading
// ═════════════════════════════════════════════════════

void readingDataSensor() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (millis() - lastSensorTime >= SENSOR_PERIOD_MS) {
    lastSensorTime = millis();

    data = readSensors();
  }
}


// ═════════════════════════════════════════════════════
//  MAIN LOOP FUNCTION 3 — COMPUTATION
//  Module : Computation algorithm
//  Input  : UAV_state + location of GCS
//  Output : Relative deg
// ═════════════════════════════════════════════════════

void computation() {
  if (setupState == RUNNING) {
    computeRelativeDeg(data);
  }
}


// ═════════════════════════════════════════════════════
//  MAIN LOOP FUNCTION 4 — OUTPUT
//  Module : Antenna selection subsystem + output
//  Input  : Relative deg
//  Output : Selection line 1 & 2 + command output
// ═════════════════════════════════════════════════════

void output() {
  if (setupState != RUNNING) {
    return;
  }

  // Fast path:
  // Command antenna-sector switching every 100 ms.
  // This satisfies the ≤ 500 ms sensing-to-switching requirement.
  if (millis() - lastSwitchTime >= SWITCH_PERIOD_MS) {
    lastSwitchTime = millis();

    antennaSelectionSubsystem(data.relativeAngle);
  }

  // Slow path:
  // Send Bluetooth/Serial status report every 5 seconds.
  // This does not delay antenna switching.
  if (millis() - lastReportTime >= REPORT_PERIOD_MS) {
    lastReportTime = millis();

    String selectedAntenna = antennaSelectionSubsystem(data.relativeAngle);

    sendOutput(data, selectedAntenna);
  }
}


// ═════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  prefs.begin("tracker", false);
  BOARD_OFFSET = prefs.getFloat("offset", 0.0);

  Serial.println("[NVS] Loaded compass offset: " + String(BOARD_OFFSET, 1) + " deg");

  I2C_0.begin(BMP_SDA, BMP_SCL);

  if (!bmp.begin_I2C(0x77, &I2C_0)) {
    Serial.println("[BMP390] Not found! Check wiring on SDA=8, SCL=9.");

    while (1) {
      delay(10);
    }
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("[BMP390] OK");

  I2C_1.begin(MMC_SDA, MMC_SCL);

  if (!mag.begin(I2C_1)) {
    Serial.println("[MMC5983MA] Not found! Check wiring on SDA=4, SCL=5.");

    while (1) {
      delay(10);
    }
  }

  mag.softReset();

  Serial.println("[MMC5983MA] OK");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("[GPS] UART started on RX=16, TX=17");

  pinMode(SWITCH_A, OUTPUT);
  pinMode(SWITCH_B, OUTPUT);

  digitalWrite(SWITCH_A, LOW);
  digitalWrite(SWITCH_B, LOW);

  setupBluetooth();

  Serial.println("=== Ready — waiting for BLE connection ===");
}


// ═════════════════════════════════════════════════════
//  MAIN LOOP 
// ═════════════════════════════════════════════════════

void loop() {
  CommandHandler();
  readingDataSensor();
  computation();
  output();
}