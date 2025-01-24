#include "HX711.h"

// Define pins for HX711 module
#define DT_PIN D4  // Data pin (DT) connected to NodeMCU pin D6
#define SCK_PIN D3 // Clock pin (SCK) connected to NodeMCU pin D5

HX711 scale;

float calibration_factor = 100; // Your calibrated factor
float zero_error = 0;           // Variable to store zero error
float units;

// Function to apply a deadband
float applyDeadband(float value, float threshold) {
  return (abs(value) < threshold) ? 0.00 : value;
}

void setup() {
  Serial.begin(115200);

  scale.begin(DT_PIN, SCK_PIN);  // Initialize HX711 with data and clock pins
  scale.set_scale(calibration_factor);
  scale.tare();  // Reset the scale to 0

  // Read the zero error
  zero_error = scale.get_units(10);  // Average of 10 readings with no load
  Serial.print("Zero error: ");
  Serial.println(zero_error);
}

void loop() {
  // Set calibration factor for reading
  scale.set_scale(calibration_factor);

  // Take an average of 10 readings
  float total = 0;
  int readings = 10;
  for (int i = 0; i < readings; i++) {
    total += scale.get_units();
    delay(10);  // Small delay between readings for stability
  }
  units = total / readings;  // Average of 10 readings

  // Remove zero error
  units -= zero_error;

  // Apply deadband
  units = applyDeadband(units, 3.0);  // Apply a deadband with a threshold of 3.0

  // Display readings
  Serial.print("Reading: ");
  Serial.print(units);
  Serial.println();

  delay(500);  // Delay for readability
}
