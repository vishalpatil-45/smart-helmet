/*
  Smart Helmet - ESP32 with SMS Alerts and MPU6050 Accident Detection
  Features:
  - Alcohol detection using MQ-3 sensor
  - Helmet worn detection using IR sensor
  - Drowsiness detection using IR sensor
  - Accident detection using MPU6050 (accelerometer + gyroscope)
  - Sends SMS alerts via Circuit Digest Cloud API
  - Prints all sensor status on Serial Monitor
  - Buzzer & LED alerts for local notification
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
const char* ssid = "your_wifi_ssid";    // Wi-Fi name
const char* password = "password";     // Wi-Fi password

// --- SMS API credentials ---
const char* apiKey = "api_key";  // Circuit Digest Cloud API key
const char* templateID = "115";       // Template ID for "Unauthorized Access / Alert"
const char* mobileNumber = "91xxxxxxxxxx"; // Recipient's mobile number
const char* var1 = "RIDER";           // SMS variable 1

// --- Drowsiness detection variables ---
const int drowsinessThreshold = 5;
const unsigned long drowsinessTimeWindow = 10000; // 10 seconds
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
const float accidentThreshold = 2.5; // g-force threshold for accident
bool accidentDetected = false;

void setup() {
  Serial.begin(115200);

  // --- Pin configuration ---
  pinMode(wearSensorPin, INPUT);
  pinMode(sleepSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  // --- Connect to Wi-Fi ---
  Serial.println("\nConnecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // --- Initialize MPU6050 ---
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  Serial.println("Helmet System Initialized.");
}

// --- Function to send SMS using Circuit Digest Cloud API ---
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
      Serial.println("SMS sent successfully!");
    } else {
      Serial.print("Failed to send SMS. HTTP code: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("Wi-Fi not connected!");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // --- Read sensors ---
  int alcoholValue = analogRead(alcoholSensorPin);
  wearStatus = !digitalRead(wearSensorPin);
  int sleepSignal = !digitalRead(sleepSensorPin);

  // --- Alcohol detection ---
  alcoholDetected = (alcoholValue > 1000); // adjust threshold

  // --- Drowsiness detection ---
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

  // --- MPU6050 Accident detection ---
  mpu.getAcceleration(&accX_raw, &accY_raw, &accZ_raw);
  mpu.getRotation(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

  accX = accX_raw / 16384.0;
  accY = accY_raw / 16384.0;
  accZ = accZ_raw / 16384.0;

  gyroX = gyroX_raw / 131.0;
  gyroY = gyroY_raw / 131.0;
  gyroZ = gyroZ_raw / 131.0;

  float totalAccel = sqrt(accX*accX + accY*accY + accZ*accZ);
  accidentDetected = (totalAccel > accidentThreshold);

  // --- Local alerts and SMS ---
  if (isDrowsy || alcoholDetected || accidentDetected) {
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, HIGH);

    // Decide message for SMS
    if (accidentDetected) {
      sendSMS("Accident Detected!");
    } else if (alcoholDetected) {
      sendSMS("Alcohol Detected!");
    } else if (isDrowsy) {
      sendSMS("Drowsiness Detected!");
    }

    delay(1000); // prevent SMS spamming
  } else {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
  }

  // --- Serial monitor output ---
  Serial.print("Helmet: "); Serial.print(wearStatus ? "Worn" : "Not Worn");
  Serial.print(" | Alcohol: "); Serial.print(alcoholDetected ? "Detected" : "Clear");
  Serial.print(" | Drowsy: "); Serial.print(isDrowsy ? "Yes" : "No");
  Serial.print(" | Accident: "); Serial.print(accidentDetected ? "Yes" : "No");
  Serial.print(" | AccX: "); Serial.print(accX);
  Serial.print(" AccY: "); Serial.print(accY);
  Serial.print(" AccZ: "); Serial.print(accZ);
  Serial.print(" GyroX: "); Serial.print(gyroX);
  Serial.print(" GyroY: "); Serial.print(gyroY);
  Serial.print(" GyroZ: "); Serial.println(gyroZ);

  delay(500);
}
