#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>
#include "BluetoothSerial.h"

// =====================================================
// HARDWARE
// =====================================================

#define MPU_SDA 21
#define MPU_SCL 22
#define MPU_ADDRESS 0x68

#define GPS_BAUD 9600
#define GPS_RX 16
#define GPS_TX 17

#define SPKR 13
#define LED 14
#define PUSH_BTN 27

// =====================================================
// OBJECTS
// =====================================================

MPU6050 mpu(MPU_ADDRESS);
TinyGPSPlus gps;

HardwareSerial GPS_PORT(2);

int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
BluetoothSerial SerialBT;

// =====================================================
// IMPACT SETTINGS
// =====================================================

const float IMPACT_THRESHOLD_G = 2.5;

// For ±16G:
// 2048 LSB = 1G
const float ACCEL_SCALE = 2048.0;

// =====================================================
// GPS
// =====================================================

double lat = 0.0;
double lon = 0.0;

// =====================================================
// TIMING
// =====================================================

unsigned long lastRefresh = 0;
const unsigned long refreshInterval = 5000;

bool flagImpact = false;

// =====================================================
// MPU INITIALIZATION
// =====================================================

bool initializeMPU() {

  // Serial.println();
  // Serial.println("Initializing MPU6050-compatible sensor...");

  // Initialize the MPU
  mpu.initialize();

  delay(100);

  // Read WHO_AM_I manually
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0) {
    Serial.println("ERROR: Cannot communicate with MPU!");
    return false;
  }

  Wire.requestFrom(MPU_ADDRESS, (uint8_t)1);

  if (Wire.available() == 0) {
    Serial.println("ERROR: WHO_AM_I read failed!");
    return false;
  }

  uint8_t whoAmI = Wire.read();

  // Serial.print("WHO_AM_I = 0x");

  // if (whoAmI < 0x10) {
  //   Serial.print("0");
  // }

  // Serial.println(whoAmI, HEX);

  // if (whoAmI == 0x70) {
  //   Serial.println("Compatible MPU detected.");
  // } 
  // else if (whoAmI == 0x68) {
  //   Serial.println("Standard MPU6050 detected.");
  // } 
  // else {
  //   Serial.println("Unknown MPU-compatible device.");
  // }

  // -------------------------------------------------
  // Wake sensor
  // PWR_MGMT_1 = 0x6B
  // -------------------------------------------------

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);

  // -------------------------------------------------
  // Configure accelerometer ±16G
  //
  // ACCEL_CONFIG = 0x1C
  // AFS_SEL = 3 -> ±16G
  // -------------------------------------------------

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x18);
  Wire.endTransmission();

  delay(50);

  // Serial.println("Accelerometer configured to ±16G.");

  return true;
}

// =====================================================
// IMPACT DETECTION
// =====================================================

bool detectImpact() {

  int16_t axRaw;
  int16_t ayRaw;
  int16_t azRaw;
  int16_t gxRaw;
  int16_t gyRaw;
  int16_t gzRaw;

  mpu.getMotion6(
    &axRaw,
    &ayRaw,
    &azRaw,
    &gxRaw,
    &gyRaw,
    &gzRaw
  );

  // Convert raw data to G
  float axG = (float)axRaw / ACCEL_SCALE;
  float ayG = (float)ayRaw / ACCEL_SCALE;
  float azG = (float)azRaw / ACCEL_SCALE;

  // Total acceleration
  float totalG = sqrt(
    axG * axG +
    ayG * ayG +
    azG * azG
  );

  Serial.print(totalG, 3);
  Serial.print("\n");

  // Serial.print("AX: ");
  // Serial.print(axG, 2);

  // Serial.print(" G | AY: ");
  // Serial.print(ayG, 2);

  // Serial.print(" G | AZ: ");
  // Serial.print(azG, 2);

  // Serial.print(" G | TOTAL: ");
  // Serial.print(totalG, 2);

  // Serial.println(" G");

  // Impact detection
  if (totalG >= IMPACT_THRESHOLD_G) {

    // Serial.println();
    // Serial.println("================================");
    // Serial.println("       !!! IMPACT !!!");
    // Serial.print("G FORCE = ");
    // Serial.print(totalG, 2);
    // Serial.println(" G");
    // Serial.println("================================");
    // Serial.println();

    lcd.clear();
    lcd.setCursor(0, 0);
    // print message
    lcd.print("Impact detected!");
    lcd.setCursor(0, 1);
    lcd.print(totalG, 3);

    SerialBT.print("G: ");
    SerialBT.print(totalG, 3);
    SerialBT.println(" ");

    digitalWrite(SPKR, HIGH);
    digitalWrite(LED, HIGH);

    return true;
  }

  return false;
}

// =====================================================
// GPS PROCESSING
// =====================================================

void processGPSData() {

  while (GPS_PORT.available()) {
    gps.encode(GPS_PORT.read());
  }

  if (gps.location.isUpdated()) {

    lat = gps.location.lat();
    lon = gps.location.lng();
  }
}

// =====================================================
// GPS PRINT
// =====================================================

void printGPSData() {

  if (gps.location.isValid()) {

    Serial.print("GPS Latitude: ");
    Serial.println(lat, 6);

    Serial.print("GPS Longitude: ");
    Serial.println(lon, 6);

  } else {

    Serial.println("GPS Searching for satellites...");
  }
}

// =====================================================
// SMS PLACEHOLDER
// =====================================================

void sendSMS(const char* message) {

  // Serial.println();
  // Serial.println("=========================================");
  // Serial.println("          ACCIDENT ALERT");
  // Serial.println("=========================================");

  // Serial.print("Message: ");
  // Serial.println(message);

  if (gps.location.isValid()) {

    Serial.print("Location: ");
    Serial.print("https://maps.google.com/?q=");
    Serial.print(lat, 6);
    Serial.print(",");
    Serial.println(lon, 6);

  } else {

    Serial.println("Location unavailable.");
  }

  // Serial.println("=========================================");
  // Serial.println();
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  // Serial.println();
  // Serial.println("=========================================");
  // Serial.println("          SMART HELMET SYSTEM");
  // Serial.println("=========================================");

  // I2C
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(100000);

  // GPS
  GPS_PORT.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );

  // Serial.println("GPS started.");

  // MPU
  if (!initializeMPU()) {

    Serial.println("MPU initialization failed.");

    while (1) {
      delay(1000);
    }
  }

  // Serial.println();
  // Serial.println("SMART HELMET SYSTEM READY!");
  // Serial.println();

  pinMode(SPKR, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(PUSH_BTN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  SerialBT.begin("Smart_Helmet");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // GPS
  processGPSData();

  // MPU
  if (detectImpact()) {
    flagImpact = true;
  }

  // Accident handling
  if (millis() - lastRefresh >= refreshInterval) {

    lastRefresh = millis();

    if (flagImpact) {

      if (gps.location.isValid()) {

        sendSMS("Accident Detected!");
        printGPSData();

      } else {

        sendSMS("Accident Detected! GPS Lock pending...");
      }

      flagImpact = false;
    }
  }

  // Serial.println(digitalRead(PUSH_BTN), 1);

  if (!digitalRead(PUSH_BTN)) {
    digitalWrite(SPKR, LOW);
    digitalWrite(LED, LOW);
    lcd.clear();
  }

  delay(20);
}
