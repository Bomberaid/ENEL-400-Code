#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

const char* ssid = "alvinkhuu";
const char* password = "AlvinKhuu";

const char* SERVER_URL = "http://209.38.135.76/movement";

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

unsigned long lastUpdate = 0;
const int updateInterval = 5;

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
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void servoCode(int raw) {
  filteredX = 0.70 * filteredX + 0.30 * raw;
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
    if (abs(servoAngle - lastServoAngle) >= 1) {
      steeringServo.write(servoAngle);
      lastServoAngle = servoAngle;
    }
    lastUpdate = millis();
  }

  Serial.print("Servo angle: ");
  Serial.println(servoAngle);
}

void loop() {

  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;
    http.begin(SERVER_URL);

    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {

      String payload = http.getString();
      Serial.println(payload);

      // Parse JSON
      StaticJsonDocument<200> doc;
      DeserializationError err = deserializeJson(doc, payload);

      if (!err) {
        int motor_value = doc["up/down"];
        int servo_value = doc["left/right"];
        int center = 1950;
        int deadzone = 100;

        int duty = 0;

        if (motor_value > center + deadzone) {
            // Set direction forward
            duty = map(motor_value, center, 4095, 0, motorDutyCycleLimit);
            analogWrite(forwardMotorPin, duty);

            // Both enable pins must be on to enable PWM mode on motor driver.
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
            duty = 0;  // stop
            analogWrite(forwardMotorPin, duty);
            analogWrite(reverseMotorPin, duty);

            digitalWrite(forwardEnablePin, LOW);
            digitalWrite(reverseEnablePin, LOW);
        }
        Serial.println(duty);

        servoCode(servo_value);
      }
      else {
        Serial.println("JSON parse failed");
      }
    }
    else {
      Serial.print("HTTP error: ");
      Serial.println(httpCode);
    }

    http.end();
  }

  delay(100);   // polling rate
}
