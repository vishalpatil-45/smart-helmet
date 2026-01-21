/*
  Smart Helmet - Final ESP32 Firmware (BLE Version)

  Features:
  - Accident detection using MPU6050
  - Helmet worn detection (IR)
  - Drowsiness detection (IR)
  - Alcohol detection (MQ-3)
  - Head gesture detection (gyro)
  - BLE communication with Android app (future-ready)
  - Local buzzer alert 

  NOTE:
  This firmware sends sensor data to an Android app via BLE.
  The Android app handles calls, GPS, voice assistant, and emergency actions.
*/

#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <math.h>

#define ALCOHOL_PIN 34
#define HELMET_PIN 25
#define SLEEP_PIN  26
#define BUZZER_PIN 18

// mpu6050
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

//BLE
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdef123456"

BLECharacteristic *helmetCharacteristic;

//thresholds
#define ALCOHOL_THRESHOLD   1000
#define ACCIDENT_THRESHOLD  2.8
#define GESTURE_THRESHOLD   180


bool helmetWorn = false;
bool alcoholDetected = false;
bool drowsy = false;
bool accidentDetected = false;

//setup
void setup() {
  Serial.begin(115200);

  pinMode(HELMET_PIN, INPUT);
  pinMode(SLEEP_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // MPU6050 init
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // BLE init
  BLEDevice::init("SmartHelmet");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);

  helmetCharacteristic = service->createCharacteristic(
                           CHARACTERISTIC_UUID,
                           BLECharacteristic::PROPERTY_NOTIFY
                         );
  helmetCharacteristic->addDescriptor(new BLE2902());

  service->start();
  BLEDevice::getAdvertising()->start();

  Serial.println("Smart Helmet BLE Ready");
}

// HEAD GESTURE
String detectGesture() {
  if (gy > GESTURE_THRESHOLD) return "RIGHT";   // Pick call
  if (gy < -GESTURE_THRESHOLD) return "LEFT";   // Reject call
  if (gx > GESTURE_THRESHOLD) return "NOD";     // Voice assistant
  return "NONE";
}


void loop() {

  // Read sensors
  alcoholDetected = analogRead(ALCOHOL_PIN) > ALCOHOL_THRESHOLD;
  helmetWorn = !digitalRead(HELMET_PIN);
  drowsy = !digitalRead(SLEEP_PIN);

  // Read MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  float totalAccel = sqrt(axg * axg + ayg * ayg + azg * azg);
  accidentDetected = totalAccel > ACCIDENT_THRESHOLD;

  // Buzzer alert
  if (drowsy || accidentDetected) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Create JSON packet
  StaticJsonDocument<256> doc;
  doc["helmet"] = helmetWorn;
  doc["alcohol"] = alcoholDetected;
  doc["drowsy"] = drowsy;
  doc["accident"] = accidentDetected;
  doc["gesture"] = detectGesture();
  doc["ax"] = axg;
  doc["ay"] = ayg;
  doc["az"] = azg;

  char payload[256];
  serializeJson(doc, payload);

  // Send via BLE
  helmetCharacteristic->setValue(payload);
  helmetCharacteristic->notify();

  // Debug output
  Serial.println(payload);

  delay(400);
}
