// Blynk Credentials
#define BLYNK_TEMPLATE_ID "TMPL6be0iNS5f"
#define BLYNK_TEMPLATE_NAME "Final Year Project"
#define BLYNK_AUTH_TOKEN "E097MoFQuAeCSBIEX3YW0TXMFSW9wudw"
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>

// WiFi Credentials
char ssid[] = "SR14";
char pass[] = "siusiusiu";

// GPS Configuration
static const int RXPin = D6; // NodeMCU RX connected to GPS TXD
static const int TXPin = D5; // NodeMCU TX connected to GPS RXD
static const uint32_t GPSBaud = 9600;

// TinyGPS++ and SoftwareSerial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// BlynkTimer object
BlynkTimer timer;

// MPU6050 object
MPU6050 mpu;

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);

  // Initialize GPS serial communication
  gpsSerial.begin(GPSBaud);

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Initialize I2C for MPU6050 on NodeMCU's SDA and SCL pins
  Wire.begin(D2, D1);
  mpu.initialize();  // Initialize the MPU6050 sensor
  if (mpu.testConnection()) {
    Serial.println("MPU6050 Ready!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  // Set up timers to send GPS and MPU6050 data every second
  timer.setInterval(1000L, sendGPSData);
  timer.setInterval(1000L, sendMPUData);  // Send MPU6050 data every second
}

void loop() {
  // Run Blynk and timer
  Blynk.run();
  timer.run();

  // Continuously read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

// Function to send GPS data to Blynk
void sendGPSData() {
  // Check if GPS location is valid
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Send GPS data to Blynk Virtual Pins
    Blynk.virtualWrite(V0, latitude);   // Send Latitude to Virtual Pin V0
    Blynk.virtualWrite(V1, longitude);  // Send Longitude to Virtual Pin V1

    // Print GPS data to Serial Monitor for debugging
    Serial.print("Location: ");
    Serial.print(latitude, 6);
    Serial.print(",");
    Serial.println(longitude, 6);
  } else {
    Serial.println("Waiting for valid GPS signal...");
  }
}

// Function to send MPU6050 data to Blynk
void sendMPUData() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Get acceleration values
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Get gyroscope values
  mpu.getRotation(&gx, &gy, &gz);

  // Convert raw acceleration data to 'g' units
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Convert raw gyroscope data to degrees/second
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Send accelerometer data to Blynk Virtual Pins
  Blynk.virtualWrite(V3, accelX);   // Accelerometer X-axis
  Blynk.virtualWrite(V4, accelY);   // Accelerometer Y-axis
  Blynk.virtualWrite(V5, accelZ);   // Accelerometer Z-axis

  // Send gyroscope data to Blynk Virtual Pins
  Blynk.virtualWrite(V6, gyroX);    // Gyroscope X-axis
  Blynk.virtualWrite(V7, gyroY);    // Gyroscope Y-axis
  Blynk.virtualWrite(V8, gyroZ);    // Gyroscope Z-axis

  // Print accelerometer data to Serial Monitor for debugging
  Serial.print("Acc X: "); Serial.println(accelX);
  Serial.print("Acc Y: "); Serial.println(accelY);
  Serial.print("Acc Z: "); Serial.println(accelZ);

  // Print gyroscope data to Serial Monitor for debugging
  Serial.print("Gyro X: "); Serial.println(gyroX);
  Serial.print("Gyro Y: "); Serial.println(gyroY);
  Serial.print("Gyro Z: "); Serial.println(gyroZ);
}
