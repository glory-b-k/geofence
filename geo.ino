#include <SoftwareSerial.h>

SoftwareSerial sim800l(14, 12); // RX, TX for SIM800L module
SoftwareSerial neo6m(4, 5);   // RX, TX for NEO-6M GPS module

const int buzzerPin = 16;      // Buzzer pin
const int buttonPin = 15;
const int buttonPin2 = 13;// Push button pin

float geofenceLat = 12.971598; // Latitude of geofence
float geofenceLon = 77.594562; // Longitude of geofence
float geofenceRadius = 0.01;    // Geofence radius in kilometers

bool geofenceAlertSent = false;

float latitude;
float longitude;

float lat;
float lon;

void setup() {

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  
  Serial.begin(9600);
  neo6m.begin(9600);
  sim800l.begin(9600);

  sim800l.println("AT");
  delay(100);
  sim800l.println("AT");
  delay(100);
  sim800l.println("AT");
  delay(5000);
  Serial.println("Ready");

  
}

void loop() {
  if (digitalRead(buttonPin) == HIGH) {
    Serial.println("Button Pressed");
    
    geofenceAlertSent = false; // Reset geofence alert flag when the button is pressed
    Serial.println("Geofense Reset");
  }

  if (digitalRead(buttonPin2) == HIGH)
  {
    geofenceLat = latitude;
    geofenceLon = longitude; 
  }

  if (neo6m.available() > 0) {
    if (neo6m.find("$GPGGA")) {
      // Parse GPS data
      latitude = getGPSData(2);
      longitude = getGPSData(4);
 
      Serial.print("Current Latitude  : ");
      Serial.print(latitude);
      Serial.print("   Current Longitude : ");
      Serial.println(longitude);

      // Check if the vehicle has crossed the geofence
      if (isInsideGeofence(latitude, longitude) && !geofenceAlertSent) {
        Serial.println("Geofence Crossed");
        Serial.println("Sending SMS");
        // Send SMS with location details
        sendSMSWithLocation(latitude, longitude);

        // Trigger the buzzer
        Serial.println("Triggered buzzer");
        triggerBuzzer();
        

        geofenceAlertSent = true; // Set the flag to avoid continuous alerts
        delay(60000);             // Delay for 1 minute to avoid spamming SMS
      }
    }
  }
}

float getGPSData(int index) {
  neo6m.find(",");
  for (int i = 0; i < index - 1; i++) {
    neo6m.find(",");
  }
  return neo6m.parseFloat();
}

bool isInsideGeofence(float lat, float lon) {
  // Simple distance-based geofence check
  float distance = calculateDistance(lat, lon, geofenceLat, geofenceLon);
  return distance <= geofenceRadius;
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Haversine formula to calculate distance between two points on the Earth
  float dlat = radians(lat2 - lat1);
  float dlon = radians(lon2 - lon1);
  float a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) * sin(dlon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = 6371.0 * c; // Earth radius in kilometers
  return distance;
}

void sendSMSWithLocation(float lat, float lon) {
  // Format the SMS with Google Maps link
  String message = "Vehicle has crossed the geofence! Location: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lon, 6);
  
  // Send SMS

  sim800l.println("ATD7411503108;");
  delay(10000);
  sim800l.println("ATH");
  delay(100);
  
  sim800l.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
  sim800l.print("AT+CMGS=\"7411503108\"\r"); // Replace with the recipient's phone number
  delay(1000);
  sim800l.print(message);
  delay(1000);
  sim800l.write(26); // End the SMS with Ctrl+Z
  delay(1000);
  Serial.println("SMS Sent");
}

void triggerBuzzer() {
  digitalWrite(buzzerPin, HIGH);
  delay(1000);
  digitalWrite(buzzerPin, LOW);
}