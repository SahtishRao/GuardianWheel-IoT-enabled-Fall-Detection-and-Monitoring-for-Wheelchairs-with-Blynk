#include <Wire.h>
#include <MPU6050.h>

// MPU6050 Configuration
MPU6050 mpu;

// Button, Buzzer, and LED Configuration
#define BUTTON_PIN D7   // Physical button for resetting
#define BUZZER_PIN D8   // Buzzer pin
#define LED_PIN D0      // LED pin

// Thresholds for Event Detection
float accelThreshold = 2.0;   // Acceleration threshold (g-forces)
float gyroThreshold = 250.0;  // Angular velocity threshold (degrees/second)
float freeFallThreshold = 0.2; // Free-fall acceleration threshold (g-forces)

// Variables to store offsets
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Variables
bool fallDetected = false;    // Track fall or critical events
bool trigger1Detected = false; // Track Trigger 1 detection
bool trigger2Detected = false; // Track Trigger 2 detection
unsigned long trigger1Time = 0; // Time when Trigger 1 was detected
const unsigned long trigger2Timeout = 2000; // Timeout for Trigger 2 detection (2 seconds)

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Setup...");

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 Ready!");
    calibrateSensor(); // Run calibration routine
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  // Initialize Button, Buzzer, and LED
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Setup Completed!");
}

void loop() {
  if (fallDetected) {
    // Check Button Press for Reset
    if (digitalRead(BUTTON_PIN) == LOW) {
      resetSystem();
      delay(300); // Debounce delay
    }
    return; // Stop monitoring after fall detection
  }

  // Read MPU6050 Data
  int16_t gx, gy, gz, ax, ay, az;
  mpu.getRotation(&gx, &gy, &gz);
  mpu.getAcceleration(&ax, &ay, &az);

  // Apply calibration offsets
  float gyroX = (gx / 131.0) - gyroOffsetX;
  float gyroY = (gy / 131.0) - gyroOffsetY;
  float gyroZ = (gz / 131.0) - gyroOffsetZ;

  float accelX = (ax / 16384.0) - accelOffsetX;
  float accelY = (ay / 16384.0) - accelOffsetY;
  float accelZ = (az / 16384.0) - accelOffsetZ;

  // Trigger 1 Validation
  if (!trigger1Detected) {
    detectTrigger1(gyroX, gyroY, gyroZ);
  }

  // Trigger 2 Validation (only if Trigger 1 is detected)
  if (trigger1Detected && !trigger2Detected) {
    detectTrigger2(accelX, accelY, accelZ);
  }

  // Confirm Fall if both Trigger 1 and Trigger 2 are detected
  if (trigger1Detected && trigger2Detected) {
    fallDetected = true;
    activateAlarm();
    Serial.println("Fall Detected: Alarm Activated!");
  }
}

void detectTrigger1(float gyroX, float gyroY, float gyroZ) {
  // Calculate total angular velocity
  float angularVelocity = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  if (angularVelocity > gyroThreshold && !trigger1Detected) {
    trigger1Detected = true;
    trigger1Time = millis();
    Serial.println("Trigger 1 Activated: Sudden Angular Motion Detected!");
    Serial.print("Angular Velocity: "); Serial.println(angularVelocity);

    // Determine fall direction with refined logic
    if (abs(gyroX) > abs(gyroY) && abs(gyroX) > abs(gyroZ)) {
      if (gyroX > gyroThreshold) {
        Serial.println("Fall Direction: Front Fall");
      } else if (gyroX < -gyroThreshold) {
        Serial.println("Fall Direction: Back Fall");
      }
    } else if (abs(gyroY) > abs(gyroX) && abs(gyroY) > abs(gyroZ)) {
      if (gyroY > gyroThreshold) {
        Serial.println("Fall Direction: Right Fall");
      } else if (gyroY < -gyroThreshold) {
        Serial.println("Fall Direction: Left Fall");
      }
    } else {
      Serial.println("Fall Direction: No Sharp Movement Detected");
    }
  }
}

void detectTrigger2(float accelX, float accelY, float accelZ) {
  unsigned long currentTime = millis();
  float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  if (totalAccel < freeFallThreshold) {
    trigger2Detected = true;
    Serial.println("Trigger 2 Activated: Free Fall Detected!");
  } else if (currentTime - trigger1Time > trigger2Timeout) {
    Serial.println("Trigger 2 Not Detected: System Resetting.");
    resetSystem();
  }
}

void activateAlarm() {
  digitalWrite(BUZZER_PIN, HIGH); // Turn buzzer ON
  digitalWrite(LED_PIN, HIGH);    // Turn LED ON
  Serial.println("Alarm Activated!");
}

void resetSystem() {
  fallDetected = false;
  trigger1Detected = false;
  trigger2Detected = false;
  digitalWrite(BUZZER_PIN, LOW); // Turn buzzer OFF
  digitalWrite(LED_PIN, LOW);    // Turn LED OFF
  Serial.println("System Reset by Button Press or Timeout.");
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
  Serial.print("Gyro Offsets -> X: "); Serial.print(gyroOffsetX);
  Serial.print(", Y: "); Serial.print(gyroOffsetY);
  Serial.print(", Z: "); Serial.println(gyroOffsetZ);
  Serial.print("Accel Offsets -> X: "); Serial.print(accelOffsetX);
  Serial.print(", Y: "); Serial.print(accelOffsetY);
  Serial.print(", Z: "); Serial.println(accelOffsetZ);
}
