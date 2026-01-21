/*
  Smart Helmet - Version 1 (LEGACY - ESP32 Only)

  Features:
  - Alcohol detection using MQ-3 sensor
  - Helmet worn detection using IR sensor
  - Drowsiness detection using IR sensor
  - Accident detection using MPU6050 (accelerometer + gyroscope)
  - Sends SMS alerts via Circuit Digest Cloud API
  - Prints all sensor status on Serial Monitor
  - Buzzer & LED alerts for local notification

  NOTE:
  This version uses Wi-Fi + SMS alerts.
  It has been superseded by the BLE-based version (V2)
  for Android app integration.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// --- PIN DEFINITIONS ---
const int alcoholSensorPin = 34; // MQ-3 analog pin
const int wearSensorPin = 25;    // IR sensor for helmet detection
const int sleepSensorPin = 26;   // IR sensor for drowsiness detection
const int buzzerPin = 18;
const int ledPin = 19;

// --- Wi-Fi credentials ---
const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";

// --- SMS API credentials ---
const char* apiKey = "YOUR_API_KEY";
const char* templateID = "115";
const char* mobileNumber = "91XXXXXXXXXX";
const char* var1 = "RIDER";

// --- Drowsiness detection variables ---
const int drowsinessThreshold = 5;
const unsigned long drowsinessTimeWindow = 10000;
const unsigned long resetTime = 5000;
unsigned long sleepTimestamps[drowsinessThreshold];
int sleepIndex = 0;
bool isDrowsy = false;
unsigned long lastSleepDetectionTime = 0;

// --- Alcohol detection ---
bool alcoholDetected = false;

// --- Helmet wear status ---
int wearStatus = 0;

// --- MPU6050 setup ---
MPU6050 mpu;
int16_t accX_raw, accY_raw, accZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
const float accidentThreshold = 2.5;
bool accidentDetected = false;

void setup() {
  Serial.begin(115200);

  pinMode(wearSensorPin, INPUT);
  pinMode(sleepSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  Serial.println("\nConnecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  Serial.println("Smart Helmet V1 Initialized");
}

// --- Function to send SMS ---
void sendSMS(const char* messageVar) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String apiUrl = "http://www.circuitdigest.cloud/send_sms?ID=" + String(templateID);
    String payload = "{\"mobiles\":\"" + String(mobileNumber) +
                     "\",\"var1\":\"" + String(var1) +
                     "\",\"var2\":\"" + String(messageVar) + "\"}";

    http.begin(apiUrl);
    http.addHeader("Authorization", String(apiKey));
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode == 200) {
      Serial.println("SMS sent successfully");
    } else {
      Serial.print("SMS failed, HTTP code: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("Wi-Fi not connected");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  int alcoholValue = analogRead(alcoholSensorPin);
  wearStatus = !digitalRead(wearSensorPin);
  int sleepSignal = !digitalRead(sleepSensorPin);

  alcoholDetected = (alcoholValue > 1000);

  if (sleepSignal) {
    lastSleepDetectionTime = currentMillis;
    sleepTimestamps[sleepIndex] = currentMillis;
    sleepIndex = (sleepIndex + 1) % drowsinessThreshold;

    int count = 0;
    for (int i = 0; i < drowsinessThreshold; i++) {
      if (currentMillis - sleepTimestamps[i] <= drowsinessTimeWindow) {
        count++;
      }
    }
    isDrowsy = (count >= drowsinessThreshold);
  }

  if (currentMillis - lastSleepDetectionTime > resetTime) {
    isDrowsy = false;
  }

  mpu.getAcceleration(&accX_raw, &accY_raw, &accZ_raw);
  mpu.getRotation(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

  accX = accX_raw / 16384.0;
  accY = accY_raw / 16384.0;
  accZ = accZ_raw / 16384.0;

  gyroX = gyroX_raw / 131.0;
  gyroY = gyroY_raw / 131.0;
  gyroZ = gyroZ_raw / 131.0;

  float totalAccel = sqrt(accX * accX + accY * accY + accZ * accZ);
  accidentDetected = (totalAccel > accidentThreshold);

  if (isDrowsy || alcoholDetected || accidentDetected) {
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, HIGH);

    if (accidentDetected) sendSMS("Accident Detected!");
    else if (alcoholDetected) sendSMS("Alcohol Detected!");
    else if (isDrowsy) sendSMS("Drowsiness Detected!");

    delay(1000);
  } else {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
  }

  Serial.print("Helmet: "); Serial.print(wearStatus ? "Worn" : "Not Worn");
  Serial.print(" | Alcohol: "); Serial.print(alcoholDetected ? "Detected" : "Clear");
  Serial.print(" | Drowsy: "); Serial.print(isDrowsy ? "Yes" : "No");
  Serial.print(" | Accident: "); Serial.println(accidentDetected ? "Yes" : "No");

  delay(500);
}
