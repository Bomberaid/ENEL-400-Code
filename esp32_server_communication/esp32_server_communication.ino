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

// New variables
int speed = 0;
String air = "Good";
int temperature = 25;
float kp = 2.0;
float ki = 0.5;
float kd = 0.1;

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
      json += "\"up/down\": " + String(joystickValue) + ",";
      json += "\"left/right\": " + String(servoValue) + ",";
      json += "\"speed\": " + String(speed) + ",";
      json += "\"air\": \"" + air + "\",";
      json += "\"temperature\": " + String(temperature) + ",";
      json += "\"kp\": " + String(kp, 2) + ",";
      json += "\"ki\": " + String(ki, 2) + ",";
      json += "\"kd\": " + String(kd, 2);
      json += "}";

      webSocket.sendTXT(json);

      lastJoystickValue = joystickValue;
      lastServoValue = servoValue;

      Serial.println("Sent: " + json);
    }
  }
}