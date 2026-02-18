#include <WiFi.h>
#include <WiFiUdp.h>
#include <string>

const char* ssid = "Fi";
const char* password = "gN5ehgN%";

WiFiUDP udp;
const uint16_t LISTEN_PORT = 5000;

char buf[512];
const int motorPin = 32;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(motorPin, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // helps reduce lag/jitter
  WiFi.begin(ssid, password);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("Receiver IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(LISTEN_PORT);
  Serial.println("Listening on UDP port 5000");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  int len = udp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';

  // Serial.print("From ");
  // Serial.print(udp.remoteIP());
  // Serial.print(": ");
  // Serial.println(buf);

  int thumbstick_value = std::stoi(buf);
  int forward_value = map(thumbstick_value, 1900, 4095, 0, 255);
  int backward_value = map(thumbstick_value, 0, 2000, 255, 0);
  Serial.println(thumbstick_value);

  analogWrite(motorPin, forward_value);
  // if (thumbstick_value > 3000) {
  //   digitalWrite(motorPin, HIGH);
  //   Serial.println("running");
  // } 
  // else digitalWrite(motorPin, LOW);
}
