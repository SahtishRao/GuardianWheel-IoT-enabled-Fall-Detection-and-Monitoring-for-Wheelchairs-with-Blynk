#define BLYNK_TEMPLATE_ID "TMPL69u19B-e1"
#define BLYNK_TEMPLATE_NAME "Tester"
#define BLYNK_AUTH_TOKEN "O8XM-AsVftoOZnwDG49jKHHYNy4nMCOb"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// Define button pin
#define BUTTON_PIN D3 // Replace with your actual button pin

// WiFi Credentials
char ssid[] = "SR14";
char pass[] = "siusiusiu";

// Blynk Virtual Pin
#define VIRTUAL_PIN V0

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);

  // Initialize button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Internal pull-up resistor

  // Connect to WiFi and Blynk
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();

  // Read button state
  int buttonState = digitalRead(BUTTON_PIN);

  // Invert the logic
  int virtualButtonState = (buttonState == LOW) ? 1 : 0;

  // Print the state to Serial Monitor for debugging
  Serial.print("Button State: ");
  Serial.println(virtualButtonState);

  // Send the state to Blynk
  Blynk.virtualWrite(VIRTUAL_PIN, virtualButtonState);

  delay(100); // Small delay for debounce
}
