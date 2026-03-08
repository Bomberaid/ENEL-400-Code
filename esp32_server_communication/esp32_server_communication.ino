#include <WiFi.h>
#include <WebSocketsClient.h>

const char* ssid = "alvinkhuu";
const char* password = "AlvinKhuu";

const char* WS_HOST = "209.38.135.76";
const int WS_PORT = 80;
const char* WS_PATH = "/ws_input";

const int joystickPin = 32;
const int servoPin = 33;

WebSocketsClient webSocket;
bool wsConnected = false;

int lastJoystickValue = -1;
int lastServoValue = -1;
const int changeThreshold = 20;

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("Sender WebSocket connected");
      wsConnected = true;
      break;
    case WStype_DISCONNECTED:
      Serial.println("Sender WebSocket disconnected");
      wsConnected = false;
      break;
    case WStype_TEXT:
      Serial.println("Server: " + String((char*)payload));
      break;
  }
}

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

  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(2000);
  webSocket.enableHeartbeat(15000, 3000, 2);
}

void loop() {
  webSocket.loop();

  if (wsConnected) {
    int joystickValue = analogRead(joystickPin);
    int servoValue = analogRead(servoPin);

    if (abs(joystickValue - lastJoystickValue) >= changeThreshold ||
        abs(servoValue - lastServoValue) >= changeThreshold) {

      String json = "{";
      json += "\"left/right\": " + String(servoValue) + ",";
      json += "\"up/down\": " + String(joystickValue);
      json += "}";

      webSocket.sendTXT(json);

      lastJoystickValue = joystickValue;
      lastServoValue = servoValue;

      Serial.println("Sent: " + json);
    }
  }
}