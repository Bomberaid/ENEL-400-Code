// ============================================================
//  CONTROLLER MAIN  —  NRF24 Transmitter
//  Reads joystick (throttle + steering) and sends via NRF24
//
//  Pin assignments:
//    NRF24  CE   → GPIO 16
//    NRF24  CSN  → GPIO 4
//    NRF24  SCK  → GPIO 18
//    NRF24  MOSI → GPIO 23
//    NRF24  MISO → GPIO 19
//    Joystick VRy (throttle) → GPIO 35
//    Joystick VRx (steering) → GPIO 34
//    Status LED              → GPIO 2
//
//  Required libraries: RF24
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN           16
#define CSN_PIN          4
#define VRY_PIN          35   // throttle axis (forward / back)
#define VRX_PIN          34   // steering axis (left / right)
#define LED_PIN          2

#define DEADZONE         80
#define SEND_INTERVAL_MS 20   // 50 Hz

RF24 radio(CE_PIN, CSN_PIN);
const byte ADDRESS[6] = "RCAR1";

struct ControlPacket {
  int16_t throttle;   // -100 to +100  (negative = reverse)
  uint8_t steering;   //    0 to 100   (50 = centre)
};

ControlPacket pkt;

int throttleCenter = 0;
int steerCenter    = 0;

// Average N ADC samples to reduce noise
int readAvg(int pin, int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return (int)(sum / n);
}

// Map a raw ADC reading to -100..+100 with deadzone and slow centre-drift correction
int16_t mapAxis(int raw, int &center) {
  int centered = raw - center;
  int16_t out;

  if (abs(centered) <= DEADZONE) {
    out = 0;
    center = (center * 99 + raw) / 100;  // gentle drift correction
  } else if (centered > DEADZONE) {
    int inMax = 4095 - center;
    if (inMax < DEADZONE + 1) inMax = DEADZONE + 1;
    out = (int16_t)constrain(map(centered, DEADZONE, inMax, 0, 100), 0, 100);
  } else {
    int inMin = -center;
    if (inMin > -(DEADZONE + 1)) inMin = -(DEADZONE + 1);
    out = (int16_t)constrain(map(centered, inMin, -DEADZONE, -100, 0), -100, 0);
  }
  return out;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(3000);

  Serial.println("CONTROLLER MAIN — booting...");
  analogReadResolution(12);

  radio.begin();
  radio.setChannel(108);       // 2508 MHz — above WiFi bands, avoids ESP32-CAM interference
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);     // RX decoupling cap too small to power ACK TX; CRC still protects data
  radio.setPayloadSize(sizeof(ControlPacket));  // must match car_main exactly — CRC covers full payload
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();

  Serial.println("=== NRF24 register dump ===");
  radio.printDetails();
  Serial.println("===========================");

  Serial.println("Calibrating joystick — hands off...");
  throttleCenter = readAvg(VRY_PIN, 80);
  steerCenter    = readAvg(VRX_PIN, 80);
  Serial.printf("Throttle centre = %d  |  Steer centre = %d\n", throttleCenter, steerCenter);

  Serial.println("CONTROLLER ready.");
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  unsigned long start = millis();

  int rawThrottle = readAvg(VRY_PIN, 3);
  int rawSteer    = readAvg(VRX_PIN, 3);

  pkt.throttle = mapAxis(rawThrottle, throttleCenter);

  int16_t steerSigned = mapAxis(rawSteer, steerCenter);
  pkt.steering = (uint8_t)constrain(steerSigned + 50, 0, 100);  // shift -100..+100 → 0..100

  bool ok = radio.write(&pkt, sizeof(pkt));

  Serial.printf("throttle=%4d  steering=%4d  TX=%s\n",
                pkt.throttle, (int)pkt.steering - 50, ok ? "OK" : "FAIL");

  // Maintain 50 Hz send rate
  long elapsed = (long)(millis() - start);
  if (elapsed < SEND_INTERVAL_MS) delay(SEND_INTERVAL_MS - elapsed);
}
