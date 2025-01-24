#include <Wire.h>
#include <MPU6050.h>

// MPU6050 Configuration
MPU6050 mpu;

// Button, Buzzer, and LED Configuration
#define BUTTON_PIN D7   // Physical button for resetting
#define BUZZER_PIN D8   // Buzzer pin
#define LED_PIN D0      // LED pin

// Thresholds for Event Detection
float gyroThreshold = 250.0;  // Angular velocity threshold (degrees/second)

// Variables to store offsets
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Variables
bool fallDetected = false;    // Track fall or critical events
bool trigger1Activated = false; // Track Trigger 1 state

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

  checkForTrigger1();
}

void checkForTrigger1() {
  // Read MPU6050 Data
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Apply calibration offsets
  float gyroX = (gx / 131.0) - gyroOffsetX;
  float gyroY = (gy / 131.0) - gyroOffsetY;
  float gyroZ = (gz / 131.0) - gyroOffsetZ;

  // Calculate total angular velocity
  float angularVelocity = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  // Detect Sudden Tilt or Angular Motion (Trigger 1)
  if (angularVelocity > gyroThreshold) {
    fallDetected = true;
    trigger1Activated = true;
    activateAlarm();
    Serial.println("Trigger 1 Activated: Sudden Angular Motion Detected!");
    Serial.print("Angular Velocity: "); Serial.println(angularVelocity);

    // Determine fall direction
    if (gyroX > gyroThreshold) {
      Serial.println("Fall Direction: Front Fall");
    } else if (gyroX < -gyroThreshold) {
      Serial.println("Fall Direction: Back Fall");
    } else if (gyroY > gyroThreshold) {
      Serial.println("Fall Direction: Right Fall");
    } else if (gyroY < -gyroThreshold) {
      Serial.println("Fall Direction: Left Fall");
    } else {
      Serial.println("Fall Direction: No Sharp Movement Detected");
    }
  }
}

void activateAlarm() {
  digitalWrite(BUZZER_PIN, HIGH); // Turn buzzer ON
  digitalWrite(LED_PIN, HIGH);    // Turn LED ON
  Serial.println("Alarm Activated!");
}

void resetSystem() {
  fallDetected = false;
  trigger1Activated = false;
  digitalWrite(BUZZER_PIN, LOW); // Turn buzzer OFF
  digitalWrite(LED_PIN, LOW);    // Turn LED OFF
  Serial.println("System Reset by Button Press.");
}

void calibrateSensor() {
  int numSamples = 100;
  int16_t gx, gy, gz;

  for (int i = 0; i < numSamples; i++) {
    mpu.getRotation(&gx, &gy, &gz);

    gyroOffsetX += gx / 131.0;
    gyroOffsetY += gy / 131.0;
    gyroOffsetZ += gz / 131.0;

    delay(50);
  }

  gyroOffsetX /= numSamples;
  gyroOffsetY /= numSamples;
  gyroOffsetZ /= numSamples;

  Serial.println("Calibration Complete!");
  Serial.print("Gyro Offsets -> X: "); Serial.print(gyroOffsetX);
  Serial.print(", Y: "); Serial.print(gyroOffsetY);
  Serial.print(", Z: "); Serial.println(gyroOffsetZ);
}
