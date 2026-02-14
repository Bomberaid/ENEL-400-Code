#include <WiFi.h>
#include <WiFiUdp.h>

// const char* ssid = "airuc-guest";
// const char* password = "";

const char* ssid = "alvinkhuu";
const char* password = "AlvinKhuu";

const char* RECEIVER_IP = "10.13.33.169";
const uint16_t RECEIVER_PORT = 5000;

WiFiUDP udp;
uint32_t counter = 0;

// ⚠️ Change to a valid ADC pin if 16 doesn't work
const int joystickPin = 32;

void setup() {
  Serial.begin(115200);
  delay(500);

  // Setup joystick pin
  pinMode(joystickPin, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("Sender IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Read joystick value (0–4095 for ESP32)
  int joystickValue = analogRead(joystickPin);

  char msg[120];
  snprintf(msg, sizeof(msg), 
           "Packet #%lu | Joystick: %d",
           (unsigned long)counter++,
           joystickValue);

  udp.beginPacket(RECEIVER_IP, RECEIVER_PORT);
  udp.print(msg);
  udp.endPacket();

  Serial.println(msg);
  delay(100);  // Faster updates for joystick
}
