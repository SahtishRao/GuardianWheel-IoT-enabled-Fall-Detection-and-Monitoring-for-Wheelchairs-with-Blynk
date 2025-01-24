// Blynk Credentials
#define BLYNK_TEMPLATE_ID "TMPL6be0iNS5f"
#define BLYNK_TEMPLATE_NAME "Final Year Project"
#define BLYNK_AUTH_TOKEN "E097MoFQuAeCSBIEX3YW0TXMFSW9wudw"
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

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

// Variables to track GPS data
float lastLatitude = 0.0;  // Last known latitude
float lastLongitude = 0.0; // Last known longitude
unsigned long lastGpsDataMillis = 0; // Time when last valid GPS data was received
const unsigned long gpsTimeout = 5000; // Timeout in milliseconds (5 seconds)
bool isGpsConnected = false; // Flag to track GPS connection status

void setup()
{
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

  // Set up a timer to send GPS data every second
  timer.setInterval(1000L, sendGPSData);
}

void loop()
{
  // Run Blynk and timer
  Blynk.run();
  timer.run();

  // Continuously read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    lastGpsDataMillis = millis(); // Update last valid data time
    isGpsConnected = true; // Mark GPS as connected
  }
}

void sendGPSData()
{
  unsigned long currentMillis = millis();

  // Step 1: Check for GPS Disconnection
  if ((currentMillis - lastGpsDataMillis > gpsTimeout)) {
    if (isGpsConnected) {
      Serial.println("GPS module disconnected or no real-time data.");
      isGpsConnected = false; // Mark GPS as disconnected

      // Turn off GPS Connected LED (V14) and turn on Disconnected LED (V15)
      Blynk.virtualWrite(V14, 0); // Turn OFF GPS connected LED
      Blynk.virtualWrite(V15, 255); // Turn ON GPS disconnected LED

      // Send the last known location to Blynk
      Blynk.virtualWrite(V0, lastLatitude);   // Send Last Latitude to Virtual Pin V0
      Blynk.virtualWrite(V1, lastLongitude); // Send Last Longitude to Virtual Pin V1

      // Print the last known location to Serial Monitor
      Serial.print("Last Known Location: ");
      Serial.print(lastLatitude, 6);
      Serial.print(",");
      Serial.println(lastLongitude, 6);
    }
    return; // Stop further execution until GPS is reconnected
  }

  // Step 2: GPS Location is Valid
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Update the last known location
    lastLatitude = latitude;
    lastLongitude = longitude;

    // Turn on GPS Connected LED (V14) and turn off Disconnected LED (V15)
    Blynk.virtualWrite(V14, 255); // Turn ON GPS connected LED
    Blynk.virtualWrite(V15, 0);   // Turn OFF GPS disconnected LED

    // Send data to Blynk Virtual Pins
    Blynk.virtualWrite(V0, latitude);   // Send Latitude to Virtual Pin V0
    Blynk.virtualWrite(V1, longitude);  // Send Longitude to Virtual Pin V1

    // Print data to Serial Monitor for debugging
    Serial.print("Location: ");
    Serial.print(latitude, 6);
    Serial.print(",");
    Serial.println(longitude, 6);
  } else {
    Serial.println("Waiting for valid GPS signal...");
  }
}
