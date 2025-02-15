#define BLYNK_TEMPLATE_ID "TMPL6be0iNS5f"
#define BLYNK_TEMPLATE_NAME "Final Year Project"
#define BLYNK_AUTH_TOKEN "E097MoFQuAeCSBIEX3YW0TXMFSW9wudw"
#define BLYNK_PRINT Serial
#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "HX711.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

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

// GPS Configuration
static const int RXPin = D6; // NodeMCU RX connected to GPS TXD
static const int TXPin = D5; // NodeMCU TX connected to GPS RXD
static const uint32_t GPSBaud = 9600;

// TinyGPS++ and SoftwareSerial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// BlynkTimer object
BlynkTimer timer;

// Variables to track GPS data
float lastLatitude = 0.0;  // Last known latitude
float lastLongitude = 0.0; // Last known longitude
unsigned long lastGpsDataMillis = 0; // Time when last valid GPS data was received
const unsigned long gpsTimeout = 5000; // Timeout in milliseconds (5 seconds)
bool isGpsConnected = false; // Flag to track GPS connection status

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Setup...");

 // Initialize GPS serial communication
  gpsSerial.begin(GPSBaud);

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
  // Set up a timer to send GPS data every second
  timer.setInterval(1000L, sendGPSData);
  timer.setInterval(1000L, sendWeightToBlynk);
}

void loop() {
  Blynk.run();
  timer.run();

  // Continuously read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    lastGpsDataMillis = millis(); // Update last valid data time
    isGpsConnected = true; // Mark GPS as connected
  }

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

void sendGPSData()
{
  unsigned long currentMillis = millis();

  // Step 1: Check for GPS Disconnection
  if ((currentMillis - lastGpsDataMillis > gpsTimeout)) {
    if (isGpsConnected) {
      Serial.println("GPS module disconnected or no real-time data.");
      isGpsConnected = false; // Mark GPS as disconnected

      // Turn off GPS Connected LED (V14) and turn on Disconnected LED (V15)
      Blynk.virtualWrite(V14, 0); // Turn OFF GPS connected LED
      Blynk.virtualWrite(V15, 255); // Turn ON GPS disconnected LED

      // Send the last known location to Blynk
      Blynk.virtualWrite(V0, lastLatitude);   // Send Last Latitude to Virtual Pin V0
      Blynk.virtualWrite(V1, lastLongitude); // Send Last Longitude to Virtual Pin V1

      // Update Web Page Image Button (V16) with last known location
      sendGoogleMapsURL(lastLatitude, lastLongitude);

      // Print the last known location to Serial Monitor
      Serial.print("Last Known Location: ");
      Serial.print(lastLatitude, 6);
      Serial.print(",");
      Serial.println(lastLongitude, 6);
    }
    return; // Stop further execution until GPS is reconnected
  }

  // Step 2: GPS Location is Valid
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Update the last known location
    lastLatitude = latitude;
    lastLongitude = longitude;

    // Turn on GPS Connected LED (V14) and turn off Disconnected LED (V15)
    Blynk.virtualWrite(V14, 255); // Turn ON GPS connected LED
    Blynk.virtualWrite(V15, 0);   // Turn OFF GPS disconnected LED

    // Send data to Blynk Virtual Pins
    Blynk.virtualWrite(V0, latitude);   // Send Latitude to Virtual Pin V0
    Blynk.virtualWrite(V1, longitude);  // Send Longitude to Virtual Pin V1

    // Update Web Page Image Button (V16) with the current location
    sendGoogleMapsURL(latitude, longitude);

    // Print data to Serial Monitor for debugging
    Serial.print("Location: ");
    Serial.print(latitude, 6);
    Serial.print(",");
    Serial.println(longitude, 6);
  } else {
    Serial.println("Waiting for valid GPS signal...");
  }
}

void sendGoogleMapsURL(float latitude, float longitude) {
  // Construct Google Maps URL
  String googleMapsURL = "https://www.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6);

  // Dynamically update the URL of the Web Page Image Button (V16)
  Blynk.setProperty(V16, "url", googleMapsURL);

  // Print URL to Serial Monitor
  Serial.print("Google Maps URL: ");
  Serial.println(googleMapsURL);
}

void sendWeightToBlynk() {
  float currentWeight = scale.get_units(10); // Get the current weight
  Blynk.virtualWrite(V9, currentWeight); // Write the weight to Virtual Pin V9
  Serial.print("Current Weight: ");
  Serial.println(currentWeight);
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
