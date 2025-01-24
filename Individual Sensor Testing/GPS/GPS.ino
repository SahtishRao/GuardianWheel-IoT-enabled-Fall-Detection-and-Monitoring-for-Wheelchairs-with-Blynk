#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// GPS Configuration
static const int RXPin = D6; // NodeMCU RX connected to GPS TXD
static const int TXPin = D5; // NodeMCU TX connected to GPS RXD
static const uint32_t GPSBaud = 9600;

// TinyGPS++ and SoftwareSerial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// Timer variable
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second interval

void setup()
{
  // Start Serial Monitor
  Serial.begin(115200);

  // Initialize GPS serial communication
  gpsSerial.begin(GPSBaud);

  Serial.println("GPS Setup Complete. Waiting for data...");
}

void loop()
{
  // Continuously read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Get the current time
  unsigned long currentMillis = millis();

  // Check if the interval has elapsed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Check if GPS location is valid
    if (gps.location.isValid()) {
      float latitude = gps.location.lat();
      float longitude = gps.location.lng();

      // Print data to Serial Monitor
      Serial.print("Location: ");
      Serial.print(latitude, 6);
      Serial.print(",");
      Serial.println(longitude, 6);
    } else {
      Serial.println("Waiting for valid GPS signal...");
    }
  }
}
