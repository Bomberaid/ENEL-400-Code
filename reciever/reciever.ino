#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

const char* ssid = "alvinkhuu";
const char* password = "AlvinKhuu";

const char* WS_HOST = "209.38.135.76";
const int WS_PORT = 80;
const char* WS_PATH = "/ws";

const int forwardMotorPin = 25;
const int reverseMotorPin = 26;
const int forwardEnablePin = 12;
const int reverseEnablePin = 14;
const int servoPin = 18;
const int motorDutyCycleLimit = 100;

// Servo Variables
Servo steeringServo;
const int xMin = 0;
const int xCenter = 1940;
const int xMax = 4095;

const int deadZone = 120;
const int centerLockZone = 180;

const int leftAngle = 70;
const int centerAngle = 90;
const int rightAngle = 110;

float filteredX = xCenter;
int lastServoAngle = centerAngle;
unsigned long lastUpdate = 20;
const int updateInterval = 20;

// Latest received values
int latestServoValue = xCenter;
int latestMotorValue = 1950;

WebSocketsClient webSocket;
bool wsConnected = false;

void motorCode(int motor_value) {
  const int center = 1950;
  const int deadzone = 100;
  int duty = 0;

  if (motor_value > center + deadzone) {
    duty = map(motor_value, center, 4095, 0, motorDutyCycleLimit);
    analogWrite(forwardMotorPin, duty);
    digitalWrite(forwardEnablePin, HIGH);
    digitalWrite(reverseEnablePin, HIGH);
  }
  else if (motor_value < center - deadzone) {
    duty = map(motor_value, 0, center, motorDutyCycleLimit, 0);
    analogWrite(reverseMotorPin, duty);
    digitalWrite(forwardEnablePin, HIGH);
    digitalWrite(reverseEnablePin, HIGH);
  }
  else {
    duty = 0;
    analogWrite(forwardMotorPin, 0);
    analogWrite(reverseMotorPin, 0);
    digitalWrite(forwardEnablePin, LOW);
    digitalWrite(reverseEnablePin, LOW);
  }
}

void servoCode(int raw) {
  filteredX = 0.85 * filteredX + 0.15 * raw;
  int xSmooth = (int)filteredX;

  int error = xSmooth - xCenter;
  int servoAngle = centerAngle;

  if (abs(error) <= centerLockZone) {
    servoAngle = centerAngle;
  }
  else if (xSmooth < xCenter - deadZone) {
    servoAngle = map(xSmooth, xMin, xCenter - deadZone, rightAngle, centerAngle);
    servoAngle = constrain(servoAngle, centerAngle, rightAngle);
  }
  else if (xSmooth > xCenter + deadZone) {
    servoAngle = map(xSmooth, xCenter + deadZone, xMax, centerAngle, leftAngle);
    servoAngle = constrain(servoAngle, leftAngle, centerAngle);
  }

  if (millis() - lastUpdate >= updateInterval) {
    if (abs(servoAngle - lastServoAngle) >= 2) {
      steeringServo.write(servoAngle);
      lastServoAngle = servoAngle;
    }
    lastUpdate = millis();
  }
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      wsConnected = true;
      break;

    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      wsConnected = false;
      // Safe stop on disconnect
      analogWrite(forwardMotorPin, 0);
      analogWrite(reverseMotorPin, 0);
      digitalWrite(forwardEnablePin, LOW);
      digitalWrite(reverseEnablePin, LOW);
      steeringServo.write(centerAngle);
      break;

    case WStype_TEXT:
      StaticJsonDocument<200> doc;
      DeserializationError err = deserializeJson(doc, payload, length);
      if (!err) {
        latestMotorValue = doc["up/down"];
        latestServoValue = doc["left/right"];
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(forwardMotorPin, OUTPUT);
  pinMode(reverseMotorPin, OUTPUT);
  pinMode(forwardEnablePin, OUTPUT);
  pinMode(reverseEnablePin, OUTPUT);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(servoPin, 500, 2500);
  steeringServo.write(centerAngle);

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
  webSocket.loop();  // handles all WS communication

  // Always drive servo/motor from latest received values
  servoCode(latestServoValue);
  motorCode(latestMotorValue);
  Serial.print("Servo Value: ")
  Serial.println(latestServoValue);
  Serial.print("Motor Value: ")
  Serial.println(latestMotorValue);
}