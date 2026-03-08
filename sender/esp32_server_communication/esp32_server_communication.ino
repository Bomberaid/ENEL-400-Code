#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "Fi";
const char* password = "gN5ehgN%";

// 🔴 CHANGE to your computer's IP running Flask
const char* serverURL = "http://127.0.0.1:5000/movement";

const int joystickPin = 32;

void setup() {
  Serial.begin(115200);
  delay(500);

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
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {

  if (WiFi.status() == WL_CONNECTED) {

    int joystickValue = analogRead(joystickPin);

    // Example second axis (replace later)
    int leftRight = 10;

    HTTPClient http;
    http.begin(serverURL);

    // ✅ IMPORTANT: Tell Flask this is JSON
    http.addHeader("Content-Type", "application/json");

    // Build JSON body
    String json = String("{\"up/down\":") +
                  joystickValue +
                  ",\"left/right\":" +
                  leftRight +
                  "}";

    // Send POST request
    int responseCode = http.POST(json);

    Serial.print("Sent: ");
    Serial.println(json);

    Serial.print("Response code: ");
    Serial.println(responseCode);

    http.end();
  }
  else {
    Serial.println("WiFi disconnected");
  }

  delay(100);   // 10 updates/sec
}