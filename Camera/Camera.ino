#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char* ssid       = "alvinkhuu";
const char* password   = "AlvinKhuu";
const char* serverIP   = "164.90.158.104";
const int   serverPort = 3000;

#define CHUNK_SIZE 1400

WiFiUDP udp;
uint16_t frameID = 0;

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