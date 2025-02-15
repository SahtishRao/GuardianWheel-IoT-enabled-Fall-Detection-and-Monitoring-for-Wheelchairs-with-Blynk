// #define BLYNK_TEMPLATE_ID "TMPL6eeYKbXKQ"
// #define BLYNK_TEMPLATE_NAME "TestV2"
// #define BLYNK_AUTH_TOKEN "xFi6R1fhNRPPtX67AREzaHOzwnX51tUE"
#define BLYNK_TEMPLATE_ID "TMPL6be0iNS5f"
#define BLYNK_TEMPLATE_NAME "Final Year Project"
#define BLYNK_AUTH_TOKEN "E097MoFQuAeCSBIEX3YW0TXMFSW9wudw"

#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "HX711.h"


// MPU6050 Configuration
MPU6050 mpu;

// HX711 Scale Configuration
#define DT_PIN D4   // Data pin (DT) connected to NodeMCU pin D4
#define SCK_PIN D3  // Clock pin (SCK) connected to NodeMCU pin D3
HX711 scale;
float calibration_factor = 100; // Calibrated factor for the scale
float baselineWeight = 0;       // Initial baseline weight (kg)
float weightDropThreshold = 0; // Drastic drop threshold (40% of baseline)

// WiFi Credentials
char ssid[] = "SR14";
char pass[] = "siusiusiu";

// Timer Variables
unsigned long trigger1Time = 0;
unsigned long trigger2Time = 0;
unsigned long trigger3Time = 0;
unsigned long trigger4Time = 0;
const unsigned long trigger2Timeout = 3000;
const unsigned long trigger3Timeout = 3000;
const unsigned long trigger4Timeout = 2000;

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

// Pin Definitions
#define BUTTON_PIN D7
#define BUZZER_PIN D8
#define LED_PIN D0

// Calibration Offsets
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Timer for Blynk data updates
const unsigned long dataUpdateInterval = 1000; // 1 second
unsigned long previousMillis = 0;




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
    calibrateSensor(); // Run calibration routine
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  // Initialize HX711 Scale
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare(); // Reset scale to 0
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
}

void loop() {
  Blynk.run();

  if (fallDetected) {
    // Check Button Press for Reset
    if (digitalRead(BUTTON_PIN) == LOW) {
      resetSystem();
      delay(300); // Debounce delay
    }
    return; // Stop monitoring after fall detection
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= dataUpdateInterval) {
    previousMillis = currentMillis;
    sendSensorDataToBlynk();
  }

  // Original sensor and trigger logic remains here
  monitorSensors();
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

  // Update Blynk Virtual Pins
  Blynk.virtualWrite(V3, accelX);
  Blynk.virtualWrite(V4, accelY);
  Blynk.virtualWrite(V5, accelZ);
  Blynk.virtualWrite(V6, gyroX);
  Blynk.virtualWrite(V7, gyroY);
  Blynk.virtualWrite(V8, gyroZ);
}

void activateAlarm() {
  digitalWrite(BUZZER_PIN, HIGH); // Turn buzzer ON
  Serial.print("Buzzer On :");
  digitalWrite(LED_PIN, HIGH);    // Turn LED ON
  Serial.print("Led On :");

  Blynk.virtualWrite(V21, 255);   // Turn on Fall Detected LED
  
}

void resetSystem() {
  fallDetected = false;
  triggerCheck = 1;
  alarmRaise = 1;

  digitalWrite(BUZZER_PIN, LOW); // Turn buzzer OFF
  digitalWrite(LED_PIN, LOW);    // Turn LED OFF
  
  // Turn off all trigger LEDs
  Blynk.virtualWrite(V17, 0);
  Blynk.virtualWrite(V18, 0);
  Blynk.virtualWrite(V19, 0);
  Blynk.virtualWrite(V20, 0);
  Blynk.virtualWrite(V21, 0); // Turn off Fall Detected LED

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
      Blynk.virtualWrite(V17, 255); // Turn on Trigger 1 LED
      trigger1Time = millis();
      Serial.print("Trigger 1 Activated: ");
      Serial.print(angularVelocity);
      triggerCheck++;
      alarmRaise++;
    }
  }

  if (triggerCheck == 2) {
    unsigned long currentTime = millis();
    float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    if (totalAccel > accelThreshold) {
      Blynk.virtualWrite(V18, 255); // Turn on Trigger 2 LED
      trigger2Time = millis();
      baselineTiltAngle = calculateTiltAngle(accelX, accelY, accelZ);
      Serial.print("Trigger 2 Activated: Baseline Tilt Angle: "); 
      Serial.print(baselineTiltAngle);
      Serial.print(" Total Acceleration: ");
      Serial.print(totalAccel);
      triggerCheck++;
      alarmRaise++;
    } else if (millis() - trigger1Time > trigger2Timeout) {
      Blynk.virtualWrite(V18, 0); // Turn off Trigger 2 LED if timeout
      Serial.print("Trigger 2 Not Detected: Total Acceleration: ");
      Serial.print(totalAccel);
      trigger2Time = millis();
      triggerCheck++;
    }
  }

  if (triggerCheck == 3) {
    unsigned long currentTime = millis();
    float tiltAngle = calculateTiltAngle(accelX, accelY, accelZ);
    float percentageChange = abs(tiltAngle - baselineTiltAngle) / baselineTiltAngle * 100;

    if (percentageChange > percentageThreshold) {
      Blynk.virtualWrite(V19, 255); // Turn on Trigger 3 LED
      Serial.print("Trigger 3 Activated: Tilt Angle: ");
      Serial.print(tiltAngle);
      Serial.print(" Baseline Tilt Angle: ");
      Serial.print(baselineTiltAngle);
      Serial.print(" Percentage Change: ");
      Serial.print(percentageChange);
      triggerCheck++;
      alarmRaise++;
    } else if (currentTime - trigger2Time > trigger3Timeout) {
      Blynk.virtualWrite(V19, 0); // Turn on Trigger 3 LED
      Serial.print("Trigger 3 Not Detected: Tilt Angle: ");
      Serial.print(tiltAngle);
      Serial.print(" Percentage Change: ");
      Serial.print(percentageChange);
      triggerCheck++;
    }
  }

  if (triggerCheck == 4) {
    unsigned long currentTime = millis();
    float weight = scale.get_units(10);
    Serial.print("Weight Reading: ");
    Serial.print(weight);

    float weightDrop = baselineWeight - weight;
    Serial.print("Weight Drop: ");
    Serial.print(weightDrop);

    if (weightDrop > weightDropThreshold) {
      Blynk.virtualWrite(V20, 255); // Turn on Trigger 4 LED
      Serial.println("Trigger 4 Activated: Significant Weight Decrease Detected!");
      alarmRaise++;
      triggerCheck++;
    } else if (currentTime - trigger3Time > trigger4Timeout) {
      Blynk.virtualWrite(V20, 0); // Turn off Trigger 4 LED if timeout
      Serial.println("Trigger 4 Not Detected: System Resetting");
      resetSystem();
    }
  }

  if (triggerCheck == 5 && alarmRaise < 5) {
    Serial.println("Buzzer Off: No Fall Detected");
    resetSystem();
  }

  if (alarmRaise == 5) {
    fallDetected = true;
    activateAlarm();
    Serial.println("Fall Detected");
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
