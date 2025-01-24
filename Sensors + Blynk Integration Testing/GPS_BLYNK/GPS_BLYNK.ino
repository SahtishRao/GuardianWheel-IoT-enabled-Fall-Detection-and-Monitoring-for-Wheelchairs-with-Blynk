// Blynk Credentials
#define BLYNK_TEMPLATE_ID "TMPL6eeYKbXKQ"
#define BLYNK_TEMPLATE_NAME "TestV2"
#define BLYNK_AUTH_TOKEN "xFi6R1fhNRPPtX67AREzaHOzwnX51tUE"
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
  }
}

void sendGPSData()
{
  // Check if GPS location is valid
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

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
