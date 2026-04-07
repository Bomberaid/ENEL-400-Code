// ============================================================
//  CAR MAIN  —  NRF24 Receiver + HC-SR04 proximity sensing
//                + GY-521 accelerometer → speed estimation
//  Receives ControlPacket, drives BTS7960 motor + servo.
//  Measures distance via HC-SR04; sends BuzzPacket to
//  controller when obstacle is within BUZZ_FAR_CM.
//  Reads MPU6050 accelerometer and integrates to estimate speed.
//
//  Pin assignments:
//    NRF24  CE   → GPIO 16
//    NRF24  CSN  → GPIO 4
//    NRF24  SCK  → GPIO 22
//    NRF24  MOSI → GPIO 23
//    NRF24  MISO → GPIO 19
//    BTS7960 RPWM → GPIO 25
//    BTS7960 LPWM → GPIO 26
//    BTS7960 R_EN → GPIO 27
//    BTS7960 L_EN → GPIO 14
//    Servo        → GPIO 18  (PCB trace)
//    HC-SR04 TRIG → GPIO 32
//    HC-SR04 ECHO → GPIO 33
//    UART TX      → GPIO 17  (sensor data → ESP32-CAM)
//    GY-521 SDA   → GPIO 21
//    GY-521 SCL   → GPIO 13
//    Status LED   → GPIO 2
//
//  Required libraries: RF24, ESP32Servo
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <Wire.h>

// ----- NRF24 -----
#define CE_PIN      16
#define CSN_PIN     4
#define SCK_PIN     22
#define MISO_PIN    19
#define MOSI_PIN    23

SPIClass nrfSPI(VSPI);

// ----- BTS7960 motor driver -----
#define RPWM        25
#define LPWM        26
#define R_EN        27
#define L_EN        14
#define PWM_FREQ    1000
#define PWM_RES     8
#define CH_R        2
#define CH_L        3

// ----- Servo -----
#define SERVO_PIN    18
#define SERVO_MIN    70
#define SERVO_MAX    110
#define SERVO_CENTER 90
#define SERVO_TRIM   6

// ----- HC-SR04 -----
#define TRIG_PIN           32
#define ECHO_PIN           33
#define BUZZ_FAR_CM        50.0f
#define SENSOR_INTERVAL_MS 60

// ----- UART to ESP32-CAM -----
#define UART_TX_PIN      17
#define UART_INTERVAL_MS 200
#define DIST_EMA_ALPHA   0.3f

// ----- GY-521 (MPU6050) -----
#define MPU_ADDR         0x68
#define ACCEL_SCALE      16384.0f
#define GRAVITY          9.81f
#define ACCEL_DEADZONE   0.005f

// ----- Misc -----
#define LED_PIN      2
#define TIMEOUT_MS   500

// ----- Voltage cap: 11.1V LiPo, 7.2V motor rated -----
const float VBAT = 11.1f;
const float VMAX = 7.2f;

RF24 radio(CE_PIN, CSN_PIN);
const byte CTRL_ADDRESS[6] = "RCAR1";
const byte BUZZ_ADDRESS[6] = "RCAR2";

struct ControlPacket {
  int16_t throttle;   // -100 to +100
  uint8_t steering;   //    0 to 100 (50 = centre)
};

struct BuzzPacket {
  float distanceCm;
};

ControlPacket pkt;
Servo steeringServo;

int   pwmCap        = 255;
float servoFiltered = SERVO_CENTER;
unsigned long lastRecvTime   = 0;
unsigned long lastMeasureMs  = 0;
unsigned long lastUartSendMs = 0;
float         lastDistCm     = 999.0f;
float         smoothDistCm   = 999.0f;
bool  linked        = false;

// ----- Accelerometer / speed -----
float accelX      = 0.0f;
float accelY      = 0.0f;
float velocityX   = 0.0f;
float velocityY   = 0.0f;
float accelOffsetX = 0.0f;
float accelOffsetY = 0.0f;
unsigned long lastAccelMs = 0;

// ---- Motor helpers ----
void motorStop() {
  ledcWrite(CH_R, 0);
  ledcWrite(CH_L, 0);
}

void motorDrive(int16_t throttle) {
  if (throttle > 0) {
    int pwm = constrain(map(throttle, 0, 100, 0, pwmCap), 0, pwmCap);
    ledcWrite(CH_R, pwm);
    ledcWrite(CH_L, 0);
  } else if (throttle < 0) {
    int pwm = constrain(map(-throttle, 0, 100, 0, pwmCap), 0, pwmCap);
    ledcWrite(CH_R, 0);
    ledcWrite(CH_L, pwm);
  } else {
    motorStop();
  }
}

// ---- HC-SR04 ----
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long us = pulseIn(ECHO_PIN, HIGH, 3500);
  if (us == 0) return 999.0f;
  return (us / 2.0f) * 0.0343f;
}

// ---- MPU6050 ----
void setupMPU() {
  Wire.begin(21, 13);  // SDA=21, SCL=13

  // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Accel range ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Serial.println("MPU6050 ready");
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050 — keep car still...");
  float sumX = 0, sumY = 0;
  int samples = 200;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // az

    sumX += ax / ACCEL_SCALE;
    sumY += ay / ACCEL_SCALE;
    delay(5);
  }

  accelOffsetX = sumX / samples;
  accelOffsetY = sumY / samples;
  Serial.printf("Calibration done — offsetX=%.3f offsetY=%.3f\n",
                accelOffsetX, accelOffsetY);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                  // az — not needed
  for (int i = 0; i < 8; i++) Wire.read();  // skip temp + gyro

  // Subtract calibration offsets
  accelX = (ax / ACCEL_SCALE) - accelOffsetX;
  accelY = (ay / ACCEL_SCALE) - accelOffsetY;

  // Dead zone
  if (fabs(accelX) < ACCEL_DEADZONE) accelX = 0.0f;
  if (fabs(accelY) < ACCEL_DEADZONE) accelY = 0.0f;

  // Integrate: v += a * dt
  unsigned long now = millis();
  float dt = (now - lastAccelMs) / 1000.0f;
  lastAccelMs = now;

  if (dt > 0.0f && dt < 0.5f) {
    velocityX += accelX * GRAVITY * dt;
    velocityY += accelY * GRAVITY * dt;
  }

  // Reset velocity only after throttle has been zero for 300ms
  static unsigned long throttleZeroMs = 0;
  if (pkt.throttle == 0) {
    if (throttleZeroMs == 0) throttleZeroMs = millis();
    if (millis() - throttleZeroMs > 300) {
      velocityX = 0.0f;
      velocityY = 0.0f;
    }
  } else {
    throttleZeroMs = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // BTS7960
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM, CH_R);
  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(LPWM, CH_L);
  motorStop();

  // Servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER + SERVO_TRIM);

  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // UART to ESP32-CAM
  Serial2.begin(115200, SERIAL_8N1, -1, UART_TX_PIN);

  // MPU6050
  setupMPU();
  calibrateMPU();
  lastAccelMs = millis();

  // Voltage cap
  float dMax = VMAX / VBAT;
  if (dMax > 1.0f) dMax = 1.0f;
  pwmCap = constrain((int)(dMax * 255.0f + 0.5f), 0, 255);

  // NRF24
  nrfSPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  radio.begin(&nrfSPI);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.setPayloadSize(sizeof(ControlPacket));
  radio.openWritingPipe(BUZZ_ADDRESS);
  radio.openReadingPipe(1, CTRL_ADDRESS);
  radio.startListening();

  Serial.println("=== NRF24 register dump ===");
  radio.printDetails();
  Serial.println("===========================");

  Serial.printf("pwmCap = %d\n", pwmCap);
  Serial.println("CAR MAIN ready — waiting for link...");
}

void loop() {
  // ---- Receive control packet ----
  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    lastRecvTime = millis();

    if (!linked) {
      linked = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("=== Link established ===");
    }

    motorDrive(pkt.throttle);

    float targetAngle = (float)map(pkt.steering, 0, 100, SERVO_MIN, SERVO_MAX) + SERVO_TRIM;
    servoFiltered = 0.75f * servoFiltered + 0.25f * targetAngle;
    steeringServo.write((int)servoFiltered);

    Serial.printf("throttle=%4d  steering=%4d  servo=%.1f\n",
                  pkt.throttle, (int)pkt.steering - 50, servoFiltered);
  }

  // ---- Safety timeout ----
  if (linked && (millis() - lastRecvTime > TIMEOUT_MS)) {
    linked = false;
    motorStop();
    steeringServo.write(SERVO_CENTER + SERVO_TRIM);
    servoFiltered = SERVO_CENTER + SERVO_TRIM;
    digitalWrite(LED_PIN, LOW);
    Serial.println("=== Link LOST — motors stopped ===");
  }

  // ---- Distance sensing + buzz ----
  if (millis() - lastMeasureMs >= SENSOR_INTERVAL_MS) {
    lastMeasureMs = millis();
    lastDistCm = measureDistance();
    if (lastDistCm < 999.0f) {
      smoothDistCm = DIST_EMA_ALPHA * lastDistCm + (1.0f - DIST_EMA_ALPHA) * smoothDistCm;
    }
    if (lastDistCm < BUZZ_FAR_CM) {
      BuzzPacket bpkt = { lastDistCm };
      radio.stopListening();
      radio.write(&bpkt, sizeof(bpkt));
      radio.startListening();
    }
  }

  // ---- Send sensor data to ESP32-CAM over UART ----
  if (millis() - lastUartSendMs >= UART_INTERVAL_MS) {
    lastUartSendMs = millis();
    readMPU();

    float speed = sqrt(velocityX * velocityX + velocityY * velocityY);
    Serial.printf("[ACCEL] ax=%.3f ay=%.3f vx=%.3f vy=%.3f speed=%.3f\n",
                  accelX, accelY, velocityX, velocityY, speed);

    char objStr[20];
    if (smoothDistCm < BUZZ_FAR_CM) {
      snprintf(objStr, sizeof(objStr), "%.0f cm", smoothDistCm);
    } else {
      strcpy(objStr, "None Detected");
    }

    char carBuf[160];
    snprintf(carBuf, sizeof(carBuf),
      "C{\"speed\":%.2f,\"air\":\"Good\",\"temperature\":0,\"closest_object\":\"%s\"}\n",
      speed, objStr);
    Serial2.print(carBuf);

    int udVal = (int)map((long)pkt.throttle, -100, 100, 0, 4095);
    int lrVal = (int)map((long)pkt.steering,    0, 100, 0, 4095);
    char moveBuf[128];
    snprintf(moveBuf, sizeof(moveBuf),
      "M{\"up/down\":%d,\"left/right\":%d,\"kp\":2.0,\"ki\":0.5,\"kd\":0.1,\"mode\":\"NRF Mode\"}\n",
      udVal, lrVal);
    Serial2.print(moveBuf);
  }
}