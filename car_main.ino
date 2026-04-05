// ============================================================
//  CAR MAIN  —  NRF24 Receiver
//  Receives ControlPacket, drives BTS7960 motor + servo
//
//  Pin assignments:
//    NRF24  CE   → GPIO 16
//    NRF24  CSN  → GPIO 4
//    NRF24  SCK  → GPIO 22  (moved off GPIO 18 — servo is on 18 per PCB)
//    NRF24  MOSI → GPIO 23
//    NRF24  MISO → GPIO 19
//    BTS7960 RPWM → GPIO 25
//    BTS7960 LPWM → GPIO 26
//    BTS7960 R_EN → GPIO 27
//    BTS7960 L_EN → GPIO 14
//    Servo        → GPIO 18  (PCB trace)
//    Status LED   → GPIO 2
//
//  Required libraries: RF24, ESP32Servo
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>

// ----- NRF24 -----
#define CE_PIN      16
#define CSN_PIN     4
#define SCK_PIN     22   // moved off GPIO 18 to avoid servo conflict
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
#define SERVO_PIN    18  // PCB trace
#define SERVO_MIN    70
#define SERVO_MAX    110
#define SERVO_CENTER 90
#define SERVO_TRIM   6   // degrees; positive = shift right, negative = shift left
                         // tune this if the car drives slanted at centre joystick

// ----- Misc -----
#define LED_PIN      2
#define TIMEOUT_MS   500   // stop motors if no packet for 500 ms (~25 missed packets at 50 Hz)

// ----- Voltage cap: 11.1V LiPo, 7.2V motor rated -----
const float VBAT = 11.1f;
const float VMAX = 7.2f;

RF24 radio(CE_PIN, CSN_PIN);
const byte ADDRESS[6] = "RCAR1";

struct ControlPacket {
  int16_t throttle;   // -100 to +100  (negative = reverse)
  uint8_t steering;   //    0 to 100   (50 = centre)
};

ControlPacket pkt;
Servo steeringServo;

int   pwmCap        = 255;
float servoFiltered = SERVO_CENTER;  // EMA state
unsigned long lastRecvTime = 0;
bool  linked        = false;

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

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // BTS7960 enable pins
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Motor PWM channels
  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM, CH_R);
  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(LPWM, CH_L);
  motorStop();

  // Servo — centre on boot
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER + SERVO_TRIM);

  // Compute PWM cap from battery / motor voltage rating
  float dMax = VMAX / VBAT;
  if (dMax > 1.0f) dMax = 1.0f;
  pwmCap = constrain((int)(dMax * 255.0f + 0.5f), 0, 255);

  // NRF24 — custom SPI so SCK stays off GPIO 18 (servo)
  nrfSPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  radio.begin(&nrfSPI);
  radio.setChannel(108);       // 2508 MHz — above WiFi bands
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.setPayloadSize(sizeof(ControlPacket));
  radio.openReadingPipe(1, ADDRESS);
  radio.startListening();

  Serial.println("=== NRF24 register dump ===");
  radio.printDetails();
  Serial.println("===========================");

  Serial.printf("pwmCap = %d\n", pwmCap);
  Serial.println("CAR MAIN ready — waiting for link...");
}

void loop() {
  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    lastRecvTime = millis();

    if (!linked) {
      linked = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("=== Link established ===");
    }

    // Drive motor
    motorDrive(pkt.throttle);

    // Map steering 0..100 → SERVO_MIN..SERVO_MAX, apply trim, then 75/25 EMA to smooth commands
    float targetAngle = (float)map(pkt.steering, 0, 100, SERVO_MIN, SERVO_MAX) + SERVO_TRIM;
    servoFiltered = 0.75f * servoFiltered + 0.25f * targetAngle;
    steeringServo.write((int)servoFiltered);

    Serial.printf("throttle=%4d  steering=%4d  servo=%.1f\n",
                  pkt.throttle, (int)pkt.steering - 50, servoFiltered);
  }

  // Safety timeout — stop everything if link is lost
  if (linked && (millis() - lastRecvTime > TIMEOUT_MS)) {
    linked = false;
    motorStop();
    steeringServo.write(SERVO_CENTER + SERVO_TRIM);
    servoFiltered = SERVO_CENTER + SERVO_TRIM;
    digitalWrite(LED_PIN, LOW);
    Serial.println("=== Link LOST — motors stopped ===");
  }
}
