#include <Wire.h>
#include <MPU6050.h>

// MPU6050 Configuration
MPU6050 mpu;

// Button and Buzzer Configuration
#define BUTTON_PIN D7   // Physical button for resetting
#define BUZZER_PIN D8   // Buzzer pin

// Thresholds for Event Detection
float accelThreshold = 2.0;   // Acceleration threshold (g-forces)
float gyroThreshold = 200.0;  // Gyroscope threshold (degrees/second)

// Variables
bool fallDetected = false;    // Track fall or critical events

// Previous readings
float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0, prevTotalAccel = 0;
float prevGyroX = 0, prevGyroY = 0, prevGyroZ = 0;

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

  // Initialize Button and Buzzer
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Internal pull-up resistor for the button
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);      // Ensure buzzer is off at the start

  Serial.println("Setup Completed!");
}

void loop() {
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

  // Check for Fall or Critical Event
  if ((totalAccel > accelThreshold || abs(gyroX) > gyroThreshold || abs(gyroY) > gyroThreshold || abs(gyroZ) > gyroThreshold) && !fallDetected) {
    fallDetected = true;
    activateBuzzer();
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
  }

  // Save the current readings as previous readings for the next loop
  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;
  prevTotalAccel = totalAccel;
  prevGyroX = gyroX;
  prevGyroY = gyroY;
  prevGyroZ = gyroZ;

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
