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
#include "HX711.h"

// WiFi Credentials
char ssid[] = "SR14";
char pass[] = "siusiusiu";

// Pins for HX711 module
#define DT_PIN D4  // Data pin connected to NodeMCU pin D4
#define SCK_PIN D0 // Clock pin connected to NodeMCU pin D0

// GPS Configuration
const int RXPin = D6; // NodeMCU RX connected to GPS TXD
const int TXPin = D5; // NodeMCU TX connected to GPS RXD
const uint32_t GPSBaud = 9600;

// Button Configuration
#define BUTTON_PIN D3 // Physical button connected to D3
#define BUTTON_VIRTUAL_PIN V10 // Virtual pin for the button state in Blynk

// Hardware objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);
MPU6050 mpu;
HX711 scale;

// Variables
float calibration_factor = 100; // Initial calibration factor for HX711
float units;
const int buzzerPin = D7; // Buzzer connected to D7 on NodeMCU
bool buzzerOn = false;    // Track buzzer state
bool buttonPressed = false; // Track button press state

// Function to apply deadband to scale readings
float applyDeadband(float value, float threshold) {
  return (abs(value) < threshold) ? 0.00 : value;
}

// Blynk function to control the buzzer based on switch state
BLYNK_WRITE(V2) {
  buzzerOn = param.asInt(); // Get the state of the button (0 or 1)

  // Immediately update Blynk interface to reflect the state
  Blynk.virtualWrite(V2, buzzerOn);
}

BlynkTimer timer;

// Function to initialize all hardware components
void setup() {
  Serial.begin(115200);

  // Initialize GPS serial communication
  gpsSerial.begin(GPSBaud);

  // Connect to WiFi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Initialize I2C for MPU6050
  Wire.begin(D2, D1);
  mpu.initialize();  // Initialize MPU6050
  if (mpu.testConnection()) {
    Serial.println("MPU6050 Ready!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  // Initialize HX711
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale();
  scale.tare();  // Reset the scale to 0
  Serial.println("HX711 initialized.");

  // Set pin modes
  pinMode(buzzerPin, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Internal pull-up resistor for the button

  // Set up Blynk timer intervals
  timer.setInterval(1000L, sendGPSData);  // Send GPS data every second
  timer.setInterval(1000L, sendMPUData);   // Send MPU6050 data every second
  timer.setInterval(1000L, sendScaleData); // Send scale data every second
  timer.setInterval(100L, handleButtonPress); // Check button state every 100ms
}

void loop() {
  Blynk.run();
  timer.run();

  // Continuously read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Control the buzzer based on the switch state and button press
  if (buzzerOn) {
    playTone(1000, 500);  // Play a 1000 Hz tone for 500 milliseconds
    delay(500);           // Wait between beeps
  } else {
    digitalWrite(buzzerPin, LOW);  // Turn the buzzer OFF if switch is OFF
  }
}

// Function to send GPS data to Blynk
void sendGPSData() {
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    Blynk.virtualWrite(V0, latitude);   // Send Latitude to Virtual Pin V0
    Blynk.virtualWrite(V1, longitude);  // Send Longitude to Virtual Pin V1

    // Print GPS data to Serial Monitor
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

  // Get acceleration and gyroscope values
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Convert and send data to Blynk
  Blynk.virtualWrite(V3, ax / 16384.0);  // Accelerometer X-axis
  Blynk.virtualWrite(V4, ay / 16384.0);  // Accelerometer Y-axis
  Blynk.virtualWrite(V5, az / 16384.0);  // Accelerometer Z-axis
  Blynk.virtualWrite(V6, gx / 131.0);    // Gyroscope X-axis
  Blynk.virtualWrite(V7, gy / 131.0);    // Gyroscope Y-axis
  Blynk.virtualWrite(V8, gz / 131.0);    // Gyroscope Z-axis
}

// Function to send scale data to Blynk
void sendScaleData() {
  scale.set_scale(calibration_factor);

  // Take an average of 10 readings
  float total = 0;
  int readings = 10;
  for (int i = 0; i < readings; i++) {
    total += scale.get_units();
    delay(10);
  }
  units = total / readings;

  // Apply deadband to remove fluctuations
  units = applyDeadband(units, 2.5);

  // Send to Blynk Virtual Pin 9
  Blynk.virtualWrite(V9, units);

  // Display on Serial Monitor for debugging
  Serial.print("Reading: ");
  Serial.print(units);
  Serial.print(" grams | Calibration Factor: ");
  Serial.println(calibration_factor);
}

// Function to handle physical button presses
void handleButtonPress() {
  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW && buzzerOn) { // If button pressed and buzzer is ON
    buzzerOn = false; // Turn off the buzzer
    Blynk.virtualWrite(V2, 0); // Update Blynk button to OFF state
    Serial.println("Buzzer turned off by button press.");
    delay(300); // Debounce delay
  }

  // Send the button state to Blynk for Virtual Pin V10
  int virtualButtonState = (buttonState == LOW) ? 1 : 0; // Inverted logic
  Blynk.virtualWrite(BUTTON_VIRTUAL_PIN, virtualButtonState);
}

// Function to play a tone by simulating a frequency
void playTone(int frequency, int duration) {
  int period = 1000000L / frequency;   // Calculate period in microseconds
  int halfPeriod = period / 2;         // Half period for high and low

  unsigned long startTime = millis();  // Track the start time

  // Toggle the pin for the tone duration
  while (millis() - startTime < duration) {
    digitalWrite(buzzerPin, HIGH);  // Turn the buzzer on
    delayMicroseconds(halfPeriod);
    digitalWrite(buzzerPin, LOW);   // Turn the buzzer off
    delayMicroseconds(halfPeriod);
  }
}
