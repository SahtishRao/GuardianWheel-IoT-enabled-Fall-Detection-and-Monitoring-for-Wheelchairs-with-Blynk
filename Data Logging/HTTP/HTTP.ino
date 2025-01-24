#include <ESP8266WiFi.h>
#include <FS.h> // File system library for SPIFFS
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <MPU6050.h>
#include "HX711.h"

// Wi-Fi Credentials
const char* ssid = "SR14";
const char* password = "siusiusiu";

// MPU6050 Configuration
MPU6050 mpu;

// HX711 Scale Configuration
#define DT_PIN D4   // Data pin (DT) connected to NodeMCU pin D4
#define SCK_PIN D3  // Clock pin (SCK) connected to NodeMCU pin D3
HX711 scale;
float calibration_factor = 100; // Calibrated factor for the scale
float zero_error = 0;           // Offset for scale
float baselineWeight = 0;       // Initial baseline weight (kg)
float weightDropThreshold = 0; // Drastic drop threshold (80% of baseline)

// Web Server
ESP8266WebServer server(80);

// CSV Logging Variables
String logBuffer = ""; // Buffer to hold one row of CSV data
int valueCount = 0;    // Counter for columns (26 max)

// Timer Variables (from Beta code)
unsigned long trigger1Time = 0;
unsigned long trigger2Time = 0;
unsigned long trigger3Time = 0;
unsigned long trigger4Time = 0; // Time when Trigger 4 was detected
unsigned long trigger5Timeout = 5000; // Timeout for Trigger 5 (5 seconds)
const unsigned long trigger2Timeout = 2000; // Timeout for Trigger 2 detection (2 seconds)
const unsigned long trigger3Timeout = 2000; // Timeout for Trigger 3 detection (2 seconds)
const unsigned long trigger4Timeout = 2000; // Timeout for Trigger 4 detection (2 seconds)

// Threshold Variables (from Beta code)
float accelThreshold = 2.0;
float gyroThreshold = 250.0;
float freeFallThreshold = 1.0;
float tiltThreshold = 60.0;
float baselineTiltAngle = 0.0; // Tilt angle before Trigger 3

// Trigger Variables (from Beta code)
int triggerCheck = 1;
int alarmRaise = 1;
bool fallDetected = false;

// Pin Definitions (from Beta code)
#define BUTTON_PIN D7
#define BUZZER_PIN D8
#define LED_PIN D0

// Calibration Offsets
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

void setup() {
  Serial.begin(115200);

  // Wi-Fi Connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount SPIFFS");
    return;
  }

  // MPU6050 Initialization
  Wire.begin();
  delay(2000);
  mpu.initialize();
  if (mpu.testConnection()) {
    calibrateSensor();
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
  delay(5000); // Allow the system to stabilize
  baselineWeight = scale.get_units(10); // Take the average of 10 readings
  Serial.print("Baseline Weight: ");
  Serial.println(baselineWeight);
  Serial.print("Weight Drop Threshold: ");
  weightDropThreshold = baselineWeight*0.4 ;
  Serial.println(weightDropThreshold);

  // Setup Pins
  
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Setup Web Server
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<h1>Beta Fall Detection Logger</h1><a href='/download'>Download Data</a><br><a href='/clear'>Clear Data</a>");
  });

  server.on("/download", HTTP_GET, []() {
    File file = SPIFFS.open("/data.csv", "r");
    if (!file) {
      server.send(500, "text/plain", "Failed to open file");
      return;
    }
    server.streamFile(file, "text/csv");
    file.close();
  });

  server.on("/clear", HTTP_GET, []() {
    SPIFFS.remove("/data.csv");
    server.send(200, "text/plain", "CSV file cleared");
  });

  server.begin();
  Serial.println("Server started");
}

void loop() {
  server.handleClient();

  if (fallDetected) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      resetSystem();
      delay(300);
    }
    return;
  }

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
      trigger1Time = millis();
      Serial.print("Trigger 1 Activated :");
      Serial.print(angularVelocity );
      Serial.print(":");
      logValue("Trigger 1 Activated");
      logValue(String(angularVelocity));
      //writeLogToCSV();
      triggerCheck++;
      alarmRaise++;
    }
  }

  if (triggerCheck == 2) {
    delay(2000);
    unsigned long currentTime = millis();
    float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    
    if (totalAccel < freeFallThreshold) {
      trigger2Time = millis();
      Serial.print("Trigger 2 Activated:");
      Serial.print(totalAccel);
      Serial.print(":");
      logValue("Trigger 2 Activated");
      logValue(String(totalAccel));
      //writeLogToCSV();
      triggerCheck++;
      alarmRaise++;
    } else if (millis() - trigger1Time > trigger2Timeout || totalAccel > accelThreshold) {
      Serial.print("Trigger 2 Not Detected:");
      logValue("Trigger 2 Not Detected");
      logValue(String(totalAccel));
      Serial.print(totalAccel);
      Serial.print(":");
      //writeLogToCSV();
      trigger2Time = millis(); // Ensure timing consistency
      triggerCheck++;
    }
  }

  if (triggerCheck == 3) {
    unsigned long currentTime = millis();
    float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    if (totalAccel > accelThreshold) {
      trigger3Time = millis();
      baselineTiltAngle = calculateTiltAngle(accelX, accelY, accelZ);
      Serial.print("Trigger 3 Activated:");
      Serial.print("Baseline Tilt Angle: "); 
      Serial.print(baselineTiltAngle);
      Serial.print(":");
      Serial.print(totalAccel );
      Serial.print(":");
      logValue("Trigger 3 Activated");
      logValue(String(totalAccel));
      logValue(String(baselineTiltAngle));
      //writeLogToCSV();
      triggerCheck++;
      alarmRaise++;
    } else if (millis() - trigger2Time > trigger3Timeout) {
      Serial.print("Trigger 3 Not Detected:");
      Serial.print(totalAccel );
      Serial.print(":");
      logValue("Trigger 3 Not Detected");
      logValue(String(totalAccel));
      logValue(String(baselineTiltAngle));
      //writeLogToCSV();
      trigger3Time = millis();
      triggerCheck++;
    }
  }

  if (triggerCheck == 4) {
    unsigned long currentTime = millis();
    float tiltAngle = calculateTiltAngle(accelX, accelY, accelZ);
    if ((tiltAngle - baselineTiltAngle) > tiltThreshold) {
      Serial.print("Trigger 4 Activated:");
      Serial.print(tiltAngle);
      Serial.print(":");
      Serial.print(baselineTiltAngle);
      Serial.print(":");
      logValue("Trigger 4 Activated");
      logValue(String(baselineTiltAngle));
      logValue(String(tiltAngle));
      //writeLogToCSV();
      triggerCheck++;
      alarmRaise++;
    } else if (currentTime - trigger3Time > 2000) {
      Serial.print("Trigger 4 Not Detected:");
      Serial.print(tiltAngle);
      Serial.print(":");
      logValue("Trigger 4 Not Detected");
      logValue(String(baselineTiltAngle));
      logValue(String(tiltAngle));
      triggerCheck++; 
      //writeLogToCSV();
    }
  }

  if (triggerCheck == 5) {
    // Trigger 5: Weight Drop Detection
    unsigned long currentTime = millis();

    // Measure weight
    float weight = scale.get_units(10); // Average 10 readings
    Serial.print("Weight Reading: ");
    Serial.print(weight);
    logValue("Weight Reading");
    logValue(String(weight));

    // Compare current weight to baseline weight
    float weightDrop = baselineWeight - weight; // Calculate weight drop
    Serial.print("Weight Drop: ");
    Serial.print(weightDrop);
    logValue("Weight Drop ");
    logValue(String(weightDrop));


    if (weightDrop > weightDropThreshold) {
        Serial.println("Trigger 5 Activated: Significant Weight Decrease Detected! : ");
        logValue("Trigger 5 Activated: Significant Weight Decrease Detected!");
        alarmRaise++;
        triggerCheck++;
    } else if (currentTime - trigger4Time > trigger5Timeout) {
        Serial.print("Trigger 5 Not Detected: System Resetting :");
        Serial.print("Buzzer Off :");
        Serial.print("Led Off :");
        logValue("Trigger 5 Not Detected: System Resetting.");
        logValue("Buzzer Off ");
        logValue("Led Off ");
        logValue("No Fall Detected");
        resetSystem();
    }
  }

  if (triggerCheck == 6 && alarmRaise < 6) {
    Serial.print("Buzzer Off :");
    Serial.print("Led Off :");
    Serial.print("No Fall Detected :");
    logValue("Buzzer Off :");
    logValue("Led Off :");
    logValue("No Fall Detected");
    //writeLogToCSV();
    resetSystem();
  }

  if (alarmRaise == 6) {
    fallDetected = true;
    activateAlarm();
    Serial.print("Fall Detected :");
    //writeLogToCSV();
  }
}

void logValue(String value) {
  logBuffer += value + ","; // Add value to the buffer with a comma
}

void writeLogToCSV() {
  logBuffer.remove(logBuffer.length() - 1); // Remove the trailing comma
  logBuffer += "\n"; // Add newline to indicate end of row
  File file = SPIFFS.open("/data.csv", "a");
  if (file) {
    file.print(logBuffer); // Write the buffer to the CSV
    file.close();
  }
  logBuffer = ""; // Clear the buffer for the next row
}



void activateAlarm() {
  digitalWrite(BUZZER_PIN, HIGH);
  Serial.print("Buzzer On ");
  //writeLogToCSV();
  digitalWrite(LED_PIN, HIGH);
  Serial.print("Led On");
  logValue("Buzzer On ");
  logValue("Led On ");
  logValue("Fall Detected");
 // writeLogToCSV();
}

void resetSystem() {
  fallDetected = false;
  triggerCheck = 1;
  alarmRaise = 1;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  Serial.println("System Reset by Button Press or Timeout");
  logValue("System Reset by Button Press or Timeout");
  writeLogToCSV();
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
}

float calculateTiltAngle(float accelX, float accelY, float accelZ) {
  return acos(accelZ / sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
}
