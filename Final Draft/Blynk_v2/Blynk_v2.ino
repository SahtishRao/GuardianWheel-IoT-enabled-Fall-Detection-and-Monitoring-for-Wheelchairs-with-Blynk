// Blynk Credentials
#define BLYNK_TEMPLATE_ID "TMPL6be0iNS5f"
#define BLYNK_TEMPLATE_NAME "Final Year Project"
#define BLYNK_AUTH_TOKEN "E097MoFQuAeCSBIEX3YW0TXMFSW9wudw"
#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "HX711.h"

// WiFi Credentials
char ssid[] = "SR14";
char pass[] = "siusiusiu";

// MPU6050 Configuration
MPU6050 mpu;

// HX711 Scale Configuration
#define DT_PIN D4
#define SCK_PIN D3
HX711 scale;
float calibration_factor = 100;
float baselineWeight = 0;
float weightDropThreshold = 0;

// TinyGPS++ and SoftwareSerial objects
static const int RXPin = D6; // NodeMCU RX connected to GPS TXD
static const int TXPin = D5; // NodeMCU TX connected to GPS RXD
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// Timer Variables
unsigned long trigger1Time = 0;
unsigned long trigger2Time = 0;
unsigned long trigger3Time = 0;
unsigned long trigger4Time = 0;
const unsigned long trigger2Timeout = 3000;
const unsigned long trigger3Timeout = 3000;
const unsigned long trigger4Timeout = 2000;
const unsigned long gpsTimeout = 5000;
unsigned long lastGpsDataMillis = 0;
unsigned long previousMillis = 0;
const unsigned long dataUpdateInterval = 1000;

// Threshold Variables
float accelThreshold = 1.8;
float gyroThreshold = 240.0;
float tiltThreshold = 70.0;
float baselineTiltAngle = 0.0;
float percentageThreshold = 25.0;

// Trigger Variables
int triggerCheck = 1;
int alarmRaise = 1;
bool fallDetected = false;
bool isGpsConnected = false;

// Pin Definitions
#define BUTTON_PIN D7
#define BUZZER_PIN D8
#define LED_PIN D0

// Calibration Offsets
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Variables to track GPS data
float lastLatitude = 0.0;
float lastLongitude = 0.0;

// BlynkTimer object
BlynkTimer timer;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Setup...");

  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 Ready!");
    calibrateSensor();
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  // Initialize HX711 Scale
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Measure baseline weight
  Serial.println("Measuring baseline weight...");
  digitalWrite(LED_PIN, HIGH);
  delay(5000);
  baselineWeight = scale.get_units(10);
  Serial.print("Baseline Weight: ");
  Serial.println(baselineWeight);
  Serial.print("Weight Drop Threshold: ");
  weightDropThreshold = baselineWeight * 0.4;
  Serial.println(weightDropThreshold);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Initialize GPS serial communication
  gpsSerial.begin(GPSBaud);
  timer.setInterval(1000L, sendGPSData);
  timer.setInterval(1000L, sendWeightToBlynk); // Call every 2 seconds

}

void loop() {
  Blynk.run();
  timer.run();

  if (fallDetected) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      resetSystem();
      delay(300);
    }
    return;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= dataUpdateInterval) {
    previousMillis = currentMillis;
    sendSensorDataToBlynk();
  }

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    lastGpsDataMillis = millis();
    isGpsConnected = true;
  }

  monitorSensors();
}

void sendWeightToBlynk() {
    float currentWeight = scale.get_units(3)/1000; // Reduced samples for faster response
    Blynk.virtualWrite(V22, currentWeight);   // Send weight to Blynk
    Serial.print("Weight (kg): ");
    Serial.println(currentWeight);
}

void sendGPSData() {
  unsigned long currentMillis = millis();

  if ((currentMillis - lastGpsDataMillis > gpsTimeout)) {
    if (isGpsConnected) {
      Serial.println("GPS module disconnected or no real-time data.");
      isGpsConnected = false;
      Blynk.virtualWrite(V14, 0);
      Blynk.virtualWrite(V15, 255);
      Blynk.virtualWrite(V0, lastLatitude);
      Blynk.virtualWrite(V1, lastLongitude);
      sendGoogleMapsURL(lastLatitude, lastLongitude);
      Serial.print("Last Known Location: ");
      Serial.print(lastLatitude, 6);
      Serial.print(",");
      Serial.println(lastLongitude, 6);
    }
    return;
  }

  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    lastLatitude = latitude;
    lastLongitude = longitude;
    Blynk.virtualWrite(V14, 255);
    Blynk.virtualWrite(V15, 0);
    Blynk.virtualWrite(V0, latitude);
    Blynk.virtualWrite(V1, longitude);
    sendGoogleMapsURL(latitude, longitude);
    Serial.print("Location: ");
    Serial.print(latitude, 6);
    Serial.print(",");
    Serial.println(longitude, 6);
  } else {
    Serial.println("Waiting for valid GPS signal...");
  }
}

void sendGoogleMapsURL(float latitude, float longitude) {
  String googleMapsURL = "https://www.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6);
  Blynk.setProperty(V16, "url", googleMapsURL);
  Serial.print("Google Maps URL: ");
  Serial.println(googleMapsURL);
}

void sendSensorDataToBlynk() {
  int16_t gx, gy, gz, ax, ay, az;
  mpu.getRotation(&gx, &gy, &gz);
  mpu.getAcceleration(&ax, &ay, &az);

  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  Blynk.virtualWrite(V3, accelX);
  Blynk.virtualWrite(V4, accelY);
  Blynk.virtualWrite(V5, accelZ);
  Blynk.virtualWrite(V6, gyroX);
  Blynk.virtualWrite(V7, gyroY);
  Blynk.virtualWrite(V8, gyroZ);

}

void activateAlarm() {
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  Blynk.virtualWrite(V21, 255);
  Blynk.logEvent("fall_detected", "Fall detected! Immediate assistance required.");
}

void resetSystem() {
  fallDetected = false;
  triggerCheck = 1;
  alarmRaise = 1;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  Blynk.virtualWrite(V17, 0);
  Blynk.virtualWrite(V18, 0);
  Blynk.virtualWrite(V19, 0);
  Blynk.virtualWrite(V20, 0);
  Blynk.virtualWrite(V21, 0);
  Blynk.logEvent("system_reseted", "System has been reset due to a false alarm.");
  Serial.println("System Reset by Button Press or Timeout");
}

void monitorSensors() {
  int16_t gx, gy, gz, ax, ay, az;
  mpu.getRotation(&gx, &gy, &gz);
  mpu.getAcceleration(&ax, &ay, &az);

  float gyroX = (gx / 131.0) - gyroOffsetX;
  float gyroY = (gy / 131.0) - gyroOffsetY;
  float gyroZ = (gz / 131.0) - gyroOffsetZ;

  float accelX = (ax / 16384.0) - accelOffsetX;
  float accelY = (ay / 16384.0) - accelOffsetY;
  float accelZ = (az / 16384.0) - accelOffsetZ;

  if (triggerCheck == 1) {
    float angularVelocity = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
    if (angularVelocity > gyroThreshold) {
      Blynk.virtualWrite(V17, 255);
      trigger1Time = millis();
      triggerCheck++;
      alarmRaise++;
    }
  }

  if (triggerCheck == 2) {
    unsigned long currentTime = millis();
    float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    if (totalAccel > accelThreshold) {
      Blynk.virtualWrite(V18, 255);
      trigger2Time = millis();
      baselineTiltAngle = calculateTiltAngle(accelX, accelY, accelZ);
      triggerCheck++;
      alarmRaise++;
    } else if (currentTime - trigger1Time > trigger2Timeout) {
      Blynk.virtualWrite(V18, 0);
      triggerCheck++;
    }
  }

  if (triggerCheck == 3) {
    unsigned long currentTime = millis();
    float tiltAngle = calculateTiltAngle(accelX, accelY, accelZ);
    float percentageChange = abs(tiltAngle - baselineTiltAngle) / baselineTiltAngle * 100;
    if (percentageChange > percentageThreshold) {
      Blynk.virtualWrite(V19, 255);
      triggerCheck++;
      alarmRaise++;
    } else if (currentTime - trigger2Time > trigger3Timeout) {
      Blynk.virtualWrite(V19, 0);
      triggerCheck++;
    }
  }

  if (triggerCheck == 4) {
    unsigned long currentTime = millis();
    float weight = scale.get_units(10);
    float weightDrop = baselineWeight - weight;
    if (weightDrop > weightDropThreshold) {
      Blynk.virtualWrite(V20, 255);
      alarmRaise++;
      triggerCheck++;
    } else if (currentTime - trigger3Time > trigger4Timeout) {
      Blynk.virtualWrite(V20, 0);
      resetSystem();
    }
  }

  if (triggerCheck == 5 && alarmRaise < 5) {
    resetSystem();
  }

  if (alarmRaise == 5) {
    fallDetected = true;
    activateAlarm();
  }
}

float calculateTiltAngle(float accelX, float accelY, float accelZ) {
  return acos(accelZ / sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
}

void calibrateSensor() {
  int numSamples = 100;
  int16_t gx, gy, gz, ax, ay, az;
  for (int i = 0; i < numSamples; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    mpu.getAcceleration(&ax, &ay, &az);
    gyroOffsetX += gx / 131.0;
    gyroOffsetY += gy / 131.0;
    gyroOffsetZ += gz / 131.0;
    accelOffsetX += ax / 16384.0;
    accelOffsetY += ay / 16384.0;
    accelOffsetZ += az / 16384.0;
    delay(50);
  }
  gyroOffsetX /= numSamples;
  gyroOffsetY /= numSamples;
  gyroOffsetZ /= numSamples;
  accelOffsetX /= numSamples;
  accelOffsetY /= numSamples;
  accelOffsetZ /= numSamples;
  Serial.println("Calibration Complete!");
} 