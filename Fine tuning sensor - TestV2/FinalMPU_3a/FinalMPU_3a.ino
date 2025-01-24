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
int triggerCheck = 1;  
int alarmRaise = 1;       // Trigger stage (1: Trigger 1, 2: Trigger 2, 3: Trigger 3)
bool fallDetected = false;    // Track fall or critical events
unsigned long trigger1Time = 0; // Time when Trigger 1 was detected
unsigned long trigger2Time = 0; // Time when Trigger 2 was detected
const unsigned long trigger2Timeout = 2000; // Timeout for Trigger 2 detection (2 seconds)
const unsigned long trigger3Timeout = 2000; // Timeout for Trigger 3 detection (2 seconds)

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

  // Unified Trigger Check
  if (triggerCheck == 1) {
    // Trigger 1: Sudden Angular Motion
    float angularVelocity = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
    if (angularVelocity > gyroThreshold) {
      trigger1Time = millis();
      Serial.println("Trigger 1 Activated: Sudden Angular Motion Detected!");
      Serial.print("Angular Velocity: "); Serial.println(angularVelocity);
      triggerCheck++; // Proceed to Trigger 2
      alarmRaise++;
    }
  }

  if (triggerCheck == 2) {
    // Trigger 2: Free Fall Detection
    unsigned long currentTime = millis();
    float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    if (totalAccel < freeFallThreshold) {
      trigger2Time = millis();
      Serial.println("Trigger 2 Activated: Free Fall Detected!");
      triggerCheck++; // Proceed to Trigger 3
      alarmRaise++;

    } else if (currentTime - trigger1Time > trigger2Timeout) {
      Serial.println("Trigger 2 Not Detected: Timeout Reached, Moving to Trigger 3.");
      trigger2Time = millis(); // Ensure timing consistency
      triggerCheck++; // Proceed to Trigger 3 despite timeout
    }
  }

  if (triggerCheck == 3) {
    // Trigger 3: Sudden Impact Detection
    unsigned long currentTime = millis();
    float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    if (totalAccel > accelThreshold) {
      Serial.println("Trigger 3 Activated: Sudden Impact Detected!");
      Serial.print("Total Acceleration: "); Serial.println(totalAccel);
      triggerCheck++; // Proceed to Alarm stage
      alarmRaise++;
    } else if (currentTime - trigger2Time > trigger3Timeout) {
      Serial.println("Trigger 3 Not Detected: System Resetting.");
      resetSystem(); // Reset if Trigger 3 fails
    }
  }

  // Confirm Fall if all triggers are detected
  if (alarmRaise == 4) {
    fallDetected = true;
    activateAlarm();
    Serial.println("Fall Detected: Alarm Activated!");
  }

   if (triggerCheck == 4 && !alarmRaise == 4) {
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
  triggerCheck = 1; // Reset triggerCheck to 1
  alarmRaise = 1;
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