const int buzzerPin = D7;  // Connect the "S" pin of the buzzer to D7

void setup() {
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  playTone(1000, 500);  // Play a 1000 Hz tone for 500 milliseconds
  delay(500);           // Wait for 500 milliseconds between beeps
}

// Function to play a tone by simulating a frequency
void playTone(int frequency, int duration) {
  int period = 1000000L / frequency;   // Calculate period in microseconds
  int halfPeriod = period / 2;         // Half period for high and low

  unsigned long startTime = millis();  // Track the start time

  // Keep toggling the pin until the duration is met
  while (millis() - startTime < duration) {
    digitalWrite(buzzerPin, HIGH);  // Turn the buzzer on
    delayMicroseconds(halfPeriod);  // Wait for half of the period
    digitalWrite(buzzerPin, LOW);   // Turn the buzzer off
    delayMicroseconds(halfPeriod);  // Wait for the other half
  }
}