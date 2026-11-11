#include "cam_hw.h"

#include <Arduino.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <img_converters.h>
#include <cstring>

namespace {

constexpr int FLASH_LEDC_CHANNEL = 2; // Avoid LEDC channel 0 used by camera XCLK
constexpr int MAX_STREAM_CLIENTS = 2;

SemaphoreHandle_t s_previewMutex = nullptr;
SemaphoreHandle_t s_newFrameSem = nullptr;
uint8_t *s_previewJpg = nullptr;
size_t s_previewJpgLen = 0;
size_t s_previewJpgCap = 0;
volatile uint32_t s_previewFrameSeq = 0;
volatile int s_streamClientCount = 0;
bool s_streamServerStarted = false;

void camStreamTask(void *pvParameters) {
  uint16_t port = static_cast<uint16_t>(reinterpret_cast<uintptr_t>(pvParameters));
  if (port == 0) port = 81;
  WiFiServer server(port);
  server.begin();
  server.setNoDelay(true);
  Serial.printf("[CAM_STREAM] MJPEG live stream server started on port %u\n", port);

  WiFiClient clients[MAX_STREAM_CLIENTS];
  uint32_t lastClientSeq[MAX_STREAM_CLIENTS] = {0};

  while (true) {
    // 1. Accept incoming clients
    if (server.hasClient()) {
      WiFiClient newClient = server.available();
      if (newClient) {
        newClient.setNoDelay(true);
        int slot = -1;
        for (int i = 0; i < MAX_STREAM_CLIENTS; ++i) {
          if (!clients[i] || !clients[i].connected()) {
            slot = i;
            break;
          }
        }
        if (slot >= 0) {
          if (clients[slot]) {
            clients[slot].stop();
          }
          // Read HTTP request lines (fast timeout avoids stalling existing streams)
          unsigned long t0 = millis();
          while (newClient.connected() && (millis() - t0 < 200)) {
            if (newClient.available()) {
              String l = newClient.readStringUntil('\n');
              if (l == "\r" || l.length() == 0) break;
            } else {
              vTaskDelay(pdMS_TO_TICKS(5));
            }
          }

          // Send multipart headers
          newClient.print(
              "HTTP/1.1 200 OK\r\n"
              "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n"
              "Access-Control-Allow-Origin: *\r\n"
              "Connection: close\r\n"
              "Cache-Control: no-cache, no-store, must-revalidate, max-age=0\r\n"
              "Pragma: no-cache\r\n\r\n");

          clients[slot] = newClient;
          lastClientSeq[slot] = 0;
          Serial.printf("[CAM_STREAM] Client slot #%d connected\n", slot);
        } else {
          newClient.print("HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\nBusy\r\n");
          newClient.stop();
        }
      }
    }

    // 2. Count active clients
    int activeCount = 0;
    for (int i = 0; i < MAX_STREAM_CLIENTS; ++i) {
      if (clients[i] && clients[i].connected()) {
        activeCount++;
      } else if (clients[i]) {
        clients[i].stop();
      }
    }
    s_streamClientCount = activeCount;

    // 3. Wait for new frame signal or timeout
    if (s_newFrameSem) {
      xSemaphoreTake(s_newFrameSem, pdMS_TO_TICKS(activeCount > 0 ? 50 : 100));
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    // 4. Push frame to connected clients
    if (activeCount > 0 && s_previewMutex && xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
      if (s_previewJpg && s_previewJpgLen > 0) {
        uint32_t currentSeq = s_previewFrameSeq;
        for (int i = 0; i < MAX_STREAM_CLIENTS; ++i) {
          if (!clients[i] || !clients[i].connected()) continue;
          if (lastClientSeq[i] == currentSeq) continue;

          char partHeader[96];
          int hlen = snprintf(partHeader, sizeof(partHeader),
                              "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                              static_cast<unsigned>(s_previewJpgLen));

          size_t w1 = clients[i].write(reinterpret_cast<const uint8_t *>(partHeader), hlen);
          size_t w2 = clients[i].write(s_previewJpg, s_previewJpgLen);
          size_t w3 = clients[i].write(reinterpret_cast<const uint8_t *>("\r\n"), 2);

          if (w1 != static_cast<size_t>(hlen) || w2 != s_previewJpgLen || w3 != 2) {
            clients[i].stop();
            Serial.printf("[CAM_STREAM] Client slot #%d disconnected on write error\n", i);
          } else {
            lastClientSeq[i] = currentSeq;
          }
        }
      }
      xSemaphoreGive(s_previewMutex);
    }
  }
}

} // namespace

void camHwInitLeds() {
  pinMode(CAM_HW_STATUS_LED_PIN, OUTPUT);
  camHwSetStatusLed(false);

  ledcSetup(FLASH_LEDC_CHANNEL, 5000, 8);
  ledcAttachPin(CAM_HW_FLASH_LED_PIN, FLASH_LEDC_CHANNEL);
  camHwSetFlashLed(0);
}

void camHwSetStatusLed(bool on) {
  // AI-Thinker red status LED on GPIO33 is active-LOW
  digitalWrite(CAM_HW_STATUS_LED_PIN, on ? LOW : HIGH);
}

void camHwSetFlashLed(uint8_t duty) {
  ledcWrite(FLASH_LEDC_CHANNEL, duty);
}

void camHwApplySensorSettings(bool hmirror,
                              bool vflip,
                              uint8_t flashDuty,
                              int8_t contrast) {
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, hmirror ? 1 : 0);
    s->set_vflip(s, vflip ? 1 : 0);
    s->set_contrast(s, contrast);
  }
  camHwSetFlashLed(flashDuty);
}

bool camHwInitCamera(pixformat_t pixelFormat,
                     framesize_t frameSize,
                     int jpegQuality,
                     bool hmirror,
                     bool vflip,
                     int8_t contrast) {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_HW_Y2_GPIO_NUM;
  config.pin_d1 = CAM_HW_Y3_GPIO_NUM;
  config.pin_d2 = CAM_HW_Y4_GPIO_NUM;
  config.pin_d3 = CAM_HW_Y5_GPIO_NUM;
  config.pin_d4 = CAM_HW_Y6_GPIO_NUM;
  config.pin_d5 = CAM_HW_Y7_GPIO_NUM;
  config.pin_d6 = CAM_HW_Y8_GPIO_NUM;
  config.pin_d7 = CAM_HW_Y9_GPIO_NUM;
  config.pin_xclk = CAM_HW_XCLK_GPIO_NUM;
  config.pin_pclk = CAM_HW_PCLK_GPIO_NUM;
  config.pin_vsync = CAM_HW_VSYNC_GPIO_NUM;
  config.pin_href = CAM_HW_HREF_GPIO_NUM;
  config.pin_sccb_sda = CAM_HW_SIOD_GPIO_NUM;
  config.pin_sccb_scl = CAM_HW_SIOC_GPIO_NUM;
  config.pin_pwdn = CAM_HW_PWDN_GPIO_NUM;
  config.pin_reset = CAM_HW_RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = pixelFormat;
  config.frame_size = frameSize;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.jpeg_quality = jpegQuality;
  config.fb_count = psramFound() ? 2 : 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: esp_camera_init failed (0x%x)\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, hmirror ? 1 : 0);
    s->set_vflip(s, vflip ? 1 : 0);
    s->set_contrast(s, contrast);
  }
  return true;
}

void camHwInitPreview() {
  if (!s_previewMutex) {
    s_previewMutex = xSemaphoreCreateMutex();
  }
  if (!s_newFrameSem) {
    s_newFrameSem = xSemaphoreCreateBinary();
  }
}

void camHwUpdatePreviewJpeg(const camera_fb_t *fb,
                            pixformat_t srcFormat,
                            uint8_t quality) {
  if (!fb || !fb->buf || fb->len == 0) return;

  uint8_t *jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height, srcFormat, quality, &jpgBuf, &jpgLen)) {
    return;
  }

  if (s_previewMutex && xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(40)) == pdTRUE) {
    if (jpgLen > s_previewJpgCap) {
      size_t newCap = (jpgLen + 8191) & ~4095;
      if (newCap < 32768) newCap = 32768;
      uint8_t *newBuf = static_cast<uint8_t *>(ps_realloc(s_previewJpg, newCap));
      if (!newBuf) {
        newBuf = static_cast<uint8_t *>(realloc(s_previewJpg, newCap));
      }
      if (newBuf) {
        s_previewJpg = newBuf;
        s_previewJpgCap = newCap;
      }
    }
    if (s_previewJpg && jpgLen <= s_previewJpgCap) {
      memcpy(s_previewJpg, jpgBuf, jpgLen);
      s_previewJpgLen = jpgLen;
      s_previewFrameSeq++;
    }
    xSemaphoreGive(s_previewMutex);

    if (s_newFrameSem) {
      xSemaphoreGive(s_newFrameSem);
    }
  }
  free(jpgBuf);
}

void camHwHandlePreviewRequest(WebServer &server) {
  if (!s_previewMutex || xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    server.send(503, "text/plain", "Preview busy");
    return;
  }
  if (!s_previewJpg || s_previewJpgLen == 0) {
    xSemaphoreGive(s_previewMutex);
    server.send(503, "text/plain", "Preview not ready");
    return;
  }
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.setContentLength(s_previewJpgLen);
  server.send(200, "image/jpeg", "");
  server.sendContent(reinterpret_cast<const char *>(s_previewJpg), s_previewJpgLen);
  xSemaphoreGive(s_previewMutex);
}

void camHwStartStreamServer(uint16_t port) {
  if (s_streamServerStarted) return;
  s_streamServerStarted = true;
  xTaskCreatePinnedToCore(camStreamTask,
                          "cam_stream",
                          4096,
                          reinterpret_cast<void *>(static_cast<uintptr_t>(port)),
                          1,
                          nullptr,
                          0); // Pin to Core 0 (along with lwIP / WiFi stack)
}

uint32_t camHwGetPreviewFrameSeq() {
  return s_previewFrameSeq;
}

bool camHwHasStreamClients() {
  return s_streamClientCount > 0;
}

// Estimate mean scene luminance [0,1] from a camera frame buffer.
// Reads every SAMPLE_STRIDE-th pixel to keep the cost low on the worker core.
float camHwEstimateMeanLuma(const camera_fb_t *fb) {
  if (!fb || !fb->buf || fb->len == 0 || fb->width == 0 || fb->height == 0) {
    return 0.0f;
  }

  constexpr int SAMPLE_STRIDE = 8;
  double acc = 0.0;
  size_t count = 0;

  if (fb->format == PIXFORMAT_GRAYSCALE) {
    const uint8_t *pix = static_cast<const uint8_t *>(fb->buf);
    for (int y = 0; y < fb->height; y += SAMPLE_STRIDE) {
      const uint8_t *row = pix + static_cast<size_t>(y) * fb->width;
      for (int x = 0; x < fb->width; x += SAMPLE_STRIDE) {
        acc += row[x];
        count++;
      }
    }
  } else {
    // RGB565: 16-bit little-endian pixel = (R<<11)|(G<<5)|B
    const uint16_t *pix = reinterpret_cast<const uint16_t *>(fb->buf);
    for (int y = 0; y < fb->height; y += SAMPLE_STRIDE) {
      const uint16_t *row = pix + static_cast<size_t>(y) * fb->width;
      for (int x = 0; x < fb->width; x += SAMPLE_STRIDE) {
        uint16_t px = row[x];
        uint8_t r = static_cast<uint8_t>((px >> 11) & 0x1F);
        uint8_t g = static_cast<uint8_t>((px >> 5) & 0x3F);
        uint8_t b = static_cast<uint8_t>(px & 0x1F);
        // Expand 5/6-bit channels to 8-bit for luminance.
        r = static_cast<uint8_t>((r * 255 + 15) / 31);
        g = static_cast<uint8_t>((g * 255 + 31) / 63);
        b = static_cast<uint8_t>((b * 255 + 15) / 31);
        // ITU-R BT.601 luma.
        acc += 0.299 * r + 0.587 * g + 0.114 * b;
        count++;
      }
    }
  }

  if (count == 0) return 0.0f;
  return static_cast<float>(acc / (255.0 * count));
}
