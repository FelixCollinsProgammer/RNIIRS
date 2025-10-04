#include <WiFi.h>

const char* ssid = "beacon_1";


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Creating Access Point: ");
  Serial.println(ssid);

  WiFi.softAP(ssid);

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Beacon is running.");
}

void loop() {
  delay(2000);
}