#include <Wire.h>
#include <MPU6050.h>
#include "HX711.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// MPU6050 Configuration
MPU6050 mpu;

// HX711 Configuration
#define DT_PIN D4  // HX711 data pin
#define SCK_PIN D3 // HX711 clock pin
HX711 scale;

// GPS Configuration
static const int RXPin = D6; // NodeMCU RX connected to GPS TXD
static const int TXPin = D5; // NodeMCU TX connected to GPS RXD
static const uint32_t GPSBaud = 9600;

// TinyGPS++ and SoftwareSerial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// Button and Buzzer Configuration
#define BUTTON_PIN D7   // Physical button for resetting
#define BUZZER_PIN D8   // Buzzer pin

// Thresholds for Event Detection
float accelThreshold = 2.0;   // Acceleration threshold (g-forces)
float gyroThreshold = 200.0;  // Gyroscope threshold (degrees/second)
float weightThreshold = 10.0; // Weight drop threshold in grams

// Variables
bool fallDetected = false;    // Track fall or critical events
bool gpsLockAcquired = false; // Track GPS lock status
float calibration_factor = 100; // Calibration factor for HX711
float units = 0;                // Current scale readings
float prevUnits = 0;            // Previous scale readings

// Previous readings for MPU6050
float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0, prevTotalAccel = 0;
float prevGyroX = 0, prevGyroY = 0, prevGyroZ = 0;

// Function to apply deadband to scale readings
float applyDeadband(float value, float threshold) {
  return (abs(value) < threshold) ? 0.00 : value;
}

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  Serial.println("Starting Setup...");

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 Ready!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  // Initialize HX711
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare(); // Reset scale to zero
  Serial.println("HX711 initialized.");

  // Initialize GPS
  gpsSerial.begin(GPSBaud);
  Serial.println("GPS initialized.");

  // Initialize Button and Buzzer
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Internal pull-up resistor for the button
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);      // Ensure buzzer is off at the start

  Serial.println("Setup Completed!");
}

void loop() {
  // Continuously read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Check GPS lock status
  if (!gpsLockAcquired) {
    if (gps.location.isValid()) {
      gpsLockAcquired = true;
      Serial.println("GPS lock acquired. Starting fall detection system...");
    } else {
      Serial.println("Waiting for GPS lock...");
      delay(1000); // Wait for 1 second before rechecking
      return; // Skip the rest of the loop until GPS lock is acquired
    }
  }

  // Read MPU6050 Data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Convert raw data to usable values
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Calculate total acceleration magnitude
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // Read HX711 Data
  scale.set_scale(calibration_factor);
  float total = 0;
  int readings = 10;
  for (int i = 0; i < readings; i++) {
    total += scale.get_units();
    delay(10);
  }
  units = total / readings;
  units = applyDeadband(units, 2.5); // Apply deadband

  // Calculate Weight Difference
  float weightDrop = prevUnits - units;

  // Check for Fall or Critical Event
  if ((totalAccel > accelThreshold || 
       abs(gyroX) > gyroThreshold || 
       abs(gyroY) > gyroThreshold || 
       abs(gyroZ) > gyroThreshold || 
       (weightDrop > weightThreshold)) && !fallDetected) {
    fallDetected = true;
    activateBuzzer();
    Serial.println("-----------------------------------------------------------------------------------------------------------");
    Serial.println("Fall or critical event detected!");

    // Print the previous readings
    Serial.println("Previous Readings:");
    Serial.print("Previous Acceleration: "); Serial.print(prevTotalAccel); Serial.println(" g");
    Serial.print("Previous Gyro X: "); Serial.print(prevGyroX);
    Serial.print(", Y: "); Serial.print(prevGyroY);
    Serial.print(", Z: "); Serial.println(prevGyroZ);

    // Print the current readings that caused the event
    if (totalAccel > accelThreshold) {
      Serial.println("Cause: High Acceleration");
      Serial.print("Acceleration: "); Serial.print(totalAccel); Serial.println(" g");
    }
    if (abs(gyroX) > gyroThreshold || abs(gyroY) > gyroThreshold || abs(gyroZ) > gyroThreshold) {
      Serial.println("Cause: High Orientation Change");
      Serial.print("Gyro Readings: X: "); Serial.print(gyroX);
      Serial.print(", Y: "); Serial.print(gyroY);
      Serial.print(", Z: "); Serial.println(gyroZ);
    }
    if (weightDrop > weightThreshold) {
      Serial.println("Cause: Weight Drop Detected");
      Serial.print("Previous Weight: "); Serial.print(prevUnits); Serial.println(" grams");
      Serial.print("Current Weight: "); Serial.print(units); Serial.println(" grams");
      Serial.print("Weight Drop: "); Serial.print(weightDrop); Serial.println(" grams");
    }

    // Print GPS location
    if (gps.location.isValid()) {
      float latitude = gps.location.lat();
      float longitude = gps.location.lng();
      Serial.print("Location: ");
      Serial.print(latitude, 6);
      Serial.print(",");
      Serial.println(longitude, 6);
    } else {
      Serial.println("Waiting for valid GPS signal...");
    }
    Serial.println("-----------------------------------------------------------------------------------------------------------");
  }

  // Check Button Press for Reset
  if (digitalRead(BUTTON_PIN) == LOW) {
    resetSystem();
    delay(300); // Debounce delay
  }

  // Display monitoring state if no event
  if (!fallDetected) {
    Serial.println("Monitoring...");
    Serial.print("Acceleration: "); Serial.print(totalAccel); Serial.println(" g");
    Serial.print("Gyro X: "); Serial.print(gyroX);
    Serial.print(", Y: "); Serial.print(gyroY);
    Serial.print(", Z: "); Serial.println(gyroZ);
    Serial.print("Weight: "); Serial.print(units); Serial.println(" grams");
  }

  // Save the current readings as previous readings for the next loop
  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;
  prevTotalAccel = totalAccel;
  prevGyroX = gyroX;
  prevGyroY = gyroY;
  prevGyroZ = gyroZ;
  prevUnits = units;

  delay(500); // Small delay for stability
}

void activateBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH); // Turn buzzer ON
  Serial.println("Buzzer activated.");
}

void resetSystem() {
  fallDetected = false;
  digitalWrite(BUZZER_PIN, LOW); // Turn buzzer OFF
  Serial.println("System reset by button press.");
}
