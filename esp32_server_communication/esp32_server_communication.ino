#include <WiFi.h>
#include <HTTPClient.h>
//#include <ArduinoJson.h>

const char* ssid = "alvinkhuu";
const char* password = "AlvinKhuu";

// Your Flask server endpoint
const char* SERVER_URL = "http://209.38.135.76/movement";

//uint32_t counter = 0;
const int joystickPin = 32;
const int servoPin = 33;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(joystickPin, INPUT);
  pinMode(servoPin, INPUT);

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
    int servoValue = analogRead(servoPin);

    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");

    // Create JSON body
    String json = "{";
    json += "\"left/right\": " + String(servoValue) + ",";
    json += "\"up/down\": " + String(joystickValue);
    json += "}";

    int httpResponseCode = http.POST(json);

    Serial.println(json);
    Serial.print("Response code: ");
    Serial.println(httpResponseCode);

    http.end();
  }

  delay(20);
}