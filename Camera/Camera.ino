#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char* ssid       = "alvinkhuu";
const char* password   = "AlvinKhuu";
const char* serverIP   = "164.90.158.104";
const int   serverPort = 3000;

#define CHUNK_SIZE 1400

// ----- Telemetry (Core 0 task) -----
#define TELEM_UART_RX   13          // wire from car ESP32 GPIO 13
#define TELEM_WS_HOST   "209.38.135.76"
#define TELEM_WS_PORT   80
#define TELEM_WS_PATH   "/ws_car_input"
#define MOVE_WS_PATH    "/ws_input"

WebSocketsClient telemWS;
bool telemWsConnected = false;

WebSocketsClient moveWS;
bool moveWsConnected = false;

WiFiUDP udp;
uint16_t frameID = 0;

void telemWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      telemWsConnected = true;
      Serial.println("[TELEM] Car data WS connected");
      break;
    case WStype_DISCONNECTED:
      telemWsConnected = false;
      Serial.println("[TELEM] Car data WS disconnected — will retry");
      break;
    default: break;
  }
}

void moveWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      moveWsConnected = true;
      Serial.println("[TELEM] Movement WS connected");
      break;
    case WStype_DISCONNECTED:
      moveWsConnected = false;
      Serial.println("[TELEM] Movement WS disconnected — will retry");
      break;
    default: break;
  }
}

// Runs on Core 0: reads UART from car ESP32, forwards JSON to Server 2 via WebSocket
void telemetryTask(void* param) {
  Serial2.begin(115200, SERIAL_8N1, TELEM_UART_RX, -1);  // RX=GPIO13, TX not used
  Serial.println("[TELEM] UART2 listening on GPIO " + String(TELEM_UART_RX));

  // Car sensor data → /ws_car_input
  telemWS.begin(TELEM_WS_HOST, TELEM_WS_PORT, TELEM_WS_PATH);
  telemWS.onEvent(telemWsEvent);
  telemWS.setReconnectInterval(3000);
  telemWS.enableHeartbeat(15000, 3000, 2);

  // Movement data → /ws_input
  moveWS.begin(TELEM_WS_HOST, TELEM_WS_PORT, MOVE_WS_PATH);
  moveWS.onEvent(moveWsEvent);
  moveWS.setReconnectInterval(3000);
  moveWS.enableHeartbeat(15000, 3000, 2);

  String lineBuffer = "";

  while (true) {
    telemWS.loop();
    moveWS.loop();

    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (c == '\n') {
        lineBuffer.trim();
        if (lineBuffer.length() > 1) {
          char prefix = lineBuffer[0];
          String json = lineBuffer.substring(1);
          Serial.println("[TELEM] UART rx: " + lineBuffer);

          if (prefix == 'C') {
            // Car sensor data
            if (telemWsConnected) {
              bool ok = telemWS.sendTXT(json);
              Serial.println(ok ? "[TELEM] Car WS send OK" : "[TELEM] Car WS send FAILED");
            } else {
              Serial.println("[TELEM] Car WS not connected — dropping");
            }
          } else if (prefix == 'M') {
            // Movement data
            if (moveWsConnected) {
              bool ok = moveWS.sendTXT(json);
              Serial.println(ok ? "[TELEM] Move WS send OK" : "[TELEM] Move WS send FAILED");
            } else {
              Serial.println("[TELEM] Move WS not connected — dropping");
            }
          }
        }
        lineBuffer = "";
      } else {
        lineBuffer += c;
      }
    }

    vTaskDelay(1);
  }
}

void sendFrameUDP(const uint8_t* data, size_t len) {
  int totalChunks = (len + CHUNK_SIZE - 1) / CHUNK_SIZE;

  for (int i = 0; i < totalChunks; i++) {
    size_t offset   = i * CHUNK_SIZE;
    size_t chunkLen = min((size_t)CHUNK_SIZE, len - offset);

    uint8_t packet[6 + CHUNK_SIZE];

    packet[0] = (frameID >> 8)     & 0xFF;
    packet[1] =  frameID           & 0xFF;
    packet[2] = (i >> 8)           & 0xFF;
    packet[3] =  i                 & 0xFF;
    packet[4] = (totalChunks >> 8) & 0xFF;
    packet[5] =  totalChunks       & 0xFF;

    memcpy(packet + 6, data + offset, chunkLen);

    udp.beginPacket(serverIP, serverPort);
    udp.write(packet, 6 + chunkLen);
    udp.endPacket();
  }

  frameID++;
}

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST; // Always grab newest frame
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  // Use PSRAM if available for better performance
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }

  // Sensor tweaks
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_quality(s, 20);
  s->set_gainceiling(s, GAINCEILING_4X);
  s->set_whitebal(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_gain_ctrl(s, 1);

  // Fix for OV3660 sensor
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  startCamera();

  WiFi.begin(ssid, password);
  WiFi.setSleep(false); // Disable WiFi sleep for lower latency
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());

  udp.begin(3001);
  Serial.printf("Streaming to %s:%d\n", serverIP, serverPort);

  // Launch telemetry task on Core 0 — keeps WebSocket + UART off the camera loop
  xTaskCreatePinnedToCore(
    telemetryTask,   // function
    "telemetry",     // name
    8192,            // stack size (bytes)
    NULL,            // parameter
    1,               // priority
    NULL,            // handle
    0                // Core 0
  );
  Serial.println("[TELEM] Telemetry task started on Core 0");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.reconnect();
    delay(5000);
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    sendFrameUDP(fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }
  delay(10);
}