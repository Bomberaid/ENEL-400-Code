#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "alvinkhuu";
const char* password = "AlvinKhuu";

WiFiUDP udp;
const uint16_t LISTEN_PORT = 5000;

char buf[512];

void setup() {
  Serial.begin(115200);
  delay(500);

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

  Serial.print("From ");
  Serial.print(udp.remoteIP());
  Serial.print(": ");
  Serial.println(buf);
}
