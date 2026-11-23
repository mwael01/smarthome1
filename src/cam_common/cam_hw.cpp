#include "cam_hw.h"

#include <Arduino.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <errno.h>
#include <esp_heap_caps.h>
#include <esp_jpg_decode.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <img_converters.h>
#include <lwip/sockets.h>
#include <cstring>

namespace {

constexpr int FLASH_LEDC_CHANNEL = 2; // Avoid LEDC channel 0 used by camera XCLK
constexpr int MAX_STREAM_CLIENTS = 4;
constexpr size_t PART_HEADER_RESERVE = 96;
constexpr size_t PART_TRAILER_RESERVE = 2;
constexpr size_t TCP_SEND_CHUNK_BYTES = 2872; // 2x TCP MSS (fits inside 5744B lwIP TCP_SND_BUF)

SemaphoreHandle_t s_previewMutex = nullptr;
SemaphoreHandle_t s_newFrameSem = nullptr;
uint8_t *s_previewAlloc = nullptr;
uint8_t *s_previewJpg = nullptr;
size_t s_previewJpgLen = 0;
size_t s_previewJpgCap = 0;
uint8_t *s_txPacketBuf = nullptr;
size_t s_txPacketCap = 0;
volatile uint32_t s_previewFrameSeq = 0;
volatile int s_streamClientCount = 0;
volatile unsigned long s_lastPreviewReqMs = 0;
volatile unsigned long s_lastPreviewEncodeMs = 0;
bool s_streamServerStarted = false;

struct CamInitCtx {
  const camera_config_t *cfg;
  esp_err_t err;
  SemaphoreHandle_t doneSem;
};

void camInitCore0Task(void *arg) {
  CamInitCtx *ctx = static_cast<CamInitCtx *>(arg);
  ctx->err = esp_camera_init(ctx->cfg);
  xSemaphoreGive(ctx->doneSem);
  vTaskDelete(nullptr);
}

// Native ESP-IDF lwIP socket transmitter (matches esp_http_server httpd_send_all).
// Sends in 2x-MSS chunks using blocking lwip_send with SO_SNDTIMEO and yields to
// the Core 0 Wi-Fi/lwIP tasks on EAGAIN so frames larger than TCP_SND_BUF (5744B)
// never fail on partial writes.
bool sendPacketAll(int fd, const uint8_t *buf, size_t len, bool &outPeerClosed) {
  outPeerClosed = false;
  if (fd < 0) return false;

  size_t sent = 0;
  int eagainRetries = 0;
  while (sent < len) {
    size_t chunk = len - sent;
    if (chunk > TCP_SEND_CHUNK_BYTES) {
      chunk = TCP_SEND_CHUNK_BYTES;
    }
    ssize_t n = lwip_send(fd, buf + sent, chunk, 0);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      eagainRetries = 0;
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (++eagainRetries > 30) {
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(8));
    } else {
      if (n == 0 || errno == ECONNRESET || errno == EPIPE || errno == ENOTCONN || errno == ECONNABORTED) {
        outPeerClosed = true;
      }
      return false;
    }
  }
  return true;
}

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
        int fd = newClient.fd();
        if (fd >= 0) {
          struct timeval tv = {};
          tv.tv_sec = 0;
          tv.tv_usec = 600000; // 600 ms blocking send timeout per chunk
          lwip_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        }
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
          static const char STREAM_HDR[] =
              "HTTP/1.1 200 OK\r\n"
              "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n"
              "Access-Control-Allow-Origin: *\r\n"
              "Connection: close\r\n"
              "Cache-Control: no-cache, no-store, must-revalidate, max-age=0\r\n"
              "Pragma: no-cache\r\n\r\n";
          bool closed = false;
          if (sendPacketAll(newClient.fd(),
                            reinterpret_cast<const uint8_t *>(STREAM_HDR),
                            sizeof(STREAM_HDR) - 1,
                            closed)) {
            clients[slot] = newClient;
            lastClientSeq[slot] = 0;
            Serial.printf("[CAM_STREAM] Client slot #%d connected\n", slot);
          } else {
            newClient.stop();
          }
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
      xSemaphoreTake(s_newFrameSem, pdMS_TO_TICKS(activeCount > 0 ? 30 : 100));
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    // 4. Copy latest pre-framed JPEG packet into s_txPacketBuf (<50 us lock hold)
    // and release s_previewMutex BEFORE performing network I/O.
    size_t txLen = 0;
    uint32_t currentSeq = 0;
    if (activeCount > 0 && s_previewMutex && xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      if (s_previewJpg && s_previewJpgLen > 0) {
        currentSeq = s_previewFrameSeq;
        bool anyClientNeedsFrame = false;
        for (int i = 0; i < MAX_STREAM_CLIENTS; ++i) {
          if (clients[i] && clients[i].connected() && lastClientSeq[i] != currentSeq) {
            anyClientNeedsFrame = true;
            break;
          }
        }
        if (anyClientNeedsFrame) {
          char partHeader[PART_HEADER_RESERVE];
          int hlen = snprintf(partHeader, sizeof(partHeader),
                              "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                              static_cast<unsigned>(s_previewJpgLen));
          if (hlen > 0 && static_cast<size_t>(hlen) <= PART_HEADER_RESERVE) {
            uint8_t *packetStart = s_previewJpg - hlen;
            memcpy(packetStart, partHeader, static_cast<size_t>(hlen));
            size_t totalLen = static_cast<size_t>(hlen) + s_previewJpgLen + PART_TRAILER_RESERVE;
            if (totalLen > s_txPacketCap) {
              size_t newCap = (totalLen + 4095) & ~4095;
              if (newCap < 32768) newCap = 32768;
              uint8_t *newTx = static_cast<uint8_t *>(ps_realloc(s_txPacketBuf, newCap));
              if (!newTx) {
                newTx = static_cast<uint8_t *>(realloc(s_txPacketBuf, newCap));
              }
              if (newTx) {
                s_txPacketBuf = newTx;
                s_txPacketCap = newCap;
              }
            }
            if (s_txPacketBuf && totalLen <= s_txPacketCap) {
              memcpy(s_txPacketBuf, packetStart, totalLen);
              txLen = totalLen;
            }
          }
        }
      }
      xSemaphoreGive(s_previewMutex);
    }

    // 5. Transmit packet to all active clients outside s_previewMutex
    if (txLen > 0 && s_txPacketBuf) {
      for (int i = 0; i < MAX_STREAM_CLIENTS; ++i) {
        if (!clients[i] || !clients[i].connected()) continue;
        if (lastClientSeq[i] == currentSeq) continue;

        bool peerClosed = false;
        if (sendPacketAll(clients[i].fd(), s_txPacketBuf, txLen, peerClosed)) {
          lastClientSeq[i] = currentSeq;
        } else {
          clients[i].stop();
          if (peerClosed) {
            Serial.printf("[CAM_STREAM] Client slot #%d disconnected\n", i);
          } else {
            Serial.printf("[CAM_STREAM] Client slot #%d disconnected on write timeout\n", i);
          }
        }
      }
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
  // Perform a clean hardware power-cycle on OV2640 PWDN (GPIO32) to reset
  // internal FIFO byte alignment and DSP state before SCCB initialization.
  if (CAM_HW_PWDN_GPIO_NUM >= 0) {
    pinMode(CAM_HW_PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(CAM_HW_PWDN_GPIO_NUM, HIGH);
    delay(20);
    digitalWrite(CAM_HW_PWDN_GPIO_NUM, LOW);
    delay(25);
  }

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
  // Use 20 MHz XCLK on original ESP32 (80 MHz APB / 4 = exact 50% duty cycle square wave).
  // Avoids the asymmetric LEDC odd-divider clock at 16 MHz that causes RGB565 pixel color corruption.
  config.xclk_freq_hz = 20000000;
  config.pixel_format = pixelFormat;
  config.frame_size = frameSize;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.jpeg_quality = jpegQuality;
  // Triple buffering keeps capture DMA fed while the stream task transmits the
  // previous frame. This is especially important on the S3 camera with PSRAM.
  config.fb_count = psramFound() ? 3 : 1;

  // Always execute esp_camera_init on Core 0 (PRO_CPU) so esp_intr_alloc binds
  // ll_cam_vsync_isr and ll_cam_dma_isr to Core 0 instead of Core 1 (AI core).
  esp_err_t err = ESP_FAIL;
  if (xPortGetCoreID() == 0) {
    err = esp_camera_init(&config);
  } else {
    SemaphoreHandle_t doneSem = xSemaphoreCreateBinary();
    if (doneSem) {
      CamInitCtx ctx = {&config, ESP_FAIL, doneSem};
      BaseType_t created = xTaskCreatePinnedToCore(
          camInitCore0Task, "cam_init_c0", 4096, &ctx, 5, nullptr, 0);
      if (created == pdPASS) {
        xSemaphoreTake(doneSem, portMAX_DELAY);
        err = ctx.err;
      } else {
        err = esp_camera_init(&config);
      }
      vSemaphoreDelete(doneSem);
    } else {
      err = esp_camera_init(&config);
    }
  }

  // Automatic fallback for sensors that do not support hardware JPEG compression (e.g. OV7670, GC0308)
  if (err == ESP_ERR_NOT_SUPPORTED && pixelFormat == PIXFORMAT_JPEG) {
    Serial.println("[CAM_HW] Sensor does not support hardware JPEG (e.g. OV7670 / GC0308).");
    Serial.println("[CAM_HW] Falling back to PIXFORMAT_RGB565 with software JPEG encoding...");
    config.pixel_format = PIXFORMAT_RGB565;
    if (config.frame_size > FRAMESIZE_VGA) {
      config.frame_size = FRAMESIZE_VGA;
    }
    if (!psramFound() && config.frame_size > FRAMESIZE_QVGA) {
      config.frame_size = FRAMESIZE_QVGA;
    }
    config.fb_count = psramFound() ? 3 : 1;

    if (xPortGetCoreID() == 0) {
      err = esp_camera_init(&config);
    } else {
      SemaphoreHandle_t doneSem = xSemaphoreCreateBinary();
      if (doneSem) {
        CamInitCtx ctx = {&config, ESP_FAIL, doneSem};
        BaseType_t created = xTaskCreatePinnedToCore(
            camInitCore0Task, "cam_init_c0", 4096, &ctx, 5, nullptr, 0);
        if (created == pdPASS) {
          xSemaphoreTake(doneSem, portMAX_DELAY);
          err = ctx.err;
        } else {
          err = esp_camera_init(&config);
        }
        vSemaphoreDelete(doneSem);
      } else {
        err = esp_camera_init(&config);
      }
    }
  }

  if (err != ESP_OK) {
    Serial.printf("ERROR: esp_camera_init failed (0x%x)\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("[CAM_HW] Sensor initialized: PID 0x%02X, pixel_format=%s\n",
                  s->id.PID,
                  config.pixel_format == PIXFORMAT_JPEG ? "Hardware JPEG" : "RGB565 (Software JPEG)");
    if (s->set_hmirror) s->set_hmirror(s, hmirror ? 1 : 0);
    if (s->set_vflip) s->set_vflip(s, vflip ? 1 : 0);
    if (s->set_contrast) s->set_contrast(s, contrast);
    if (s->set_brightness) s->set_brightness(s, 0);
    if (s->set_saturation) s->set_saturation(s, 0);
    if (s->set_whitebal) s->set_whitebal(s, 1);
    if (s->set_awb_gain) s->set_awb_gain(s, 1);
    if (s->set_wb_mode) s->set_wb_mode(s, 0);
    if (s->set_exposure_ctrl) s->set_exposure_ctrl(s, 1);
    if (s->set_aec2) s->set_aec2(s, 1);
    if (s->set_gain_ctrl) s->set_gain_ctrl(s, 1);
    if (s->set_gainceiling) s->set_gainceiling(s, GAINCEILING_4X);
    if (s->set_bpc) s->set_bpc(s, 1);     // Black pixel cancellation
    if (s->set_wpc) s->set_wpc(s, 1);     // White pixel cancellation
    if (s->set_raw_gma) s->set_raw_gma(s, 1); // Raw gamma correction
    if (s->set_lenc) s->set_lenc(s, 1);    // Lens shading correction
    if (s->set_dcw) s->set_dcw(s, 1);     // Clean downsampling
  }

  // Wait briefly for OV2640 PLL/AWB/AEC to settle after register programming, then
  // flush initial DMA frames so any mid-register-write byte-shifted RGB565 frames are discarded.
  delay(80);
  for (int i = 0; i < 10; ++i) {
    camera_fb_t *flushFb = esp_camera_fb_get();
    if (flushFb) {
      esp_camera_fb_return(flushFb);
    }
    delay(15);
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

void publishPreviewJpeg(const uint8_t *jpgBuf, size_t jpgLen) {
  if (!jpgBuf || jpgLen == 0) return;

  if (s_previewMutex && xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(40)) == pdTRUE) {
    if (jpgLen + PART_TRAILER_RESERVE > s_previewJpgCap) {
      size_t newCap = (jpgLen + 8191) & ~4095;
      if (newCap < 32768) newCap = 32768;
      size_t allocSize = PART_HEADER_RESERVE + newCap + PART_TRAILER_RESERVE;
      uint8_t *newBuf = static_cast<uint8_t *>(ps_realloc(s_previewAlloc, allocSize));
      if (!newBuf) {
        newBuf = static_cast<uint8_t *>(realloc(s_previewAlloc, allocSize));
      }
      if (newBuf) {
        s_previewAlloc = newBuf;
        s_previewJpg = s_previewAlloc + PART_HEADER_RESERVE;
        s_previewJpgCap = newCap;
      }
    }
    if (s_previewJpg && jpgLen <= s_previewJpgCap) {
      memcpy(s_previewJpg, jpgBuf, jpgLen);
      s_previewJpg[jpgLen] = '\r';
      s_previewJpg[jpgLen + 1] = '\n';
      s_previewJpgLen = jpgLen;
      s_previewFrameSeq++;
      s_lastPreviewEncodeMs = millis();
    }
    xSemaphoreGive(s_previewMutex);

    if (s_newFrameSem) {
      xSemaphoreGive(s_newFrameSem);
    }
  }
}

void camHwUpdatePreviewJpeg(const camera_fb_t *fb,
                            pixformat_t srcFormat,
                            uint8_t quality) {
  if (!fb || !fb->buf || fb->len == 0) return;

  if (srcFormat == PIXFORMAT_JPEG) {
    publishPreviewJpeg(fb->buf, fb->len);
    return;
  }

  // fmt2jpg uses 1-100 quality scale. If quality <= 30 (passed from OV2640 register scale),
  // boost it to crisp 75% quality so software conversion is never blocky/pixelated.
  uint8_t encQuality = (quality <= 30) ? 75 : quality;

  uint8_t *jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height, srcFormat, encQuality, &jpgBuf, &jpgLen)) {
    return;
  }
  publishPreviewJpeg(jpgBuf, jpgLen);
  free(jpgBuf);
}

void camHwUpdatePreviewJpegBytes(const uint8_t *jpgBuf, size_t jpgLen) {
  publishPreviewJpeg(jpgBuf, jpgLen);
}

void camHwHandlePreviewRequest(WebServer &server) {
  s_lastPreviewReqMs = millis();
  if (!s_previewMutex || xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    server.send(503, "text/plain", "Preview busy");
    return;
  }
  if (!s_previewJpg || s_previewJpgLen == 0) {
    xSemaphoreGive(s_previewMutex);
    server.send(503, "text/plain", "Preview not ready");
    return;
  }
  // Copy under the short mutex and release it before the socket write. A slow
  // browser must never stall the capture or MJPEG task.
  size_t jpgLen = s_previewJpgLen;
  uint8_t *jpgCopy = static_cast<uint8_t *>(ps_malloc(jpgLen));
  if (!jpgCopy) jpgCopy = static_cast<uint8_t *>(malloc(jpgLen));
  if (jpgCopy) memcpy(jpgCopy, s_previewJpg, jpgLen);
  xSemaphoreGive(s_previewMutex);
  if (!jpgCopy) {
    server.send(503, "text/plain", "Preview buffer unavailable");
    return;
  }
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.setContentLength(jpgLen);
  server.send(200, "image/jpeg", "");
  server.sendContent(reinterpret_cast<const char *>(jpgCopy), jpgLen);
  free(jpgCopy);
}

void camHwStartStreamServer(uint16_t port) {
  if (s_streamServerStarted) return;
  s_streamServerStarted = true;
  xTaskCreatePinnedToCore(camStreamTask,
                          "cam_stream",
                          6144,
                          reinterpret_cast<void *>(static_cast<uintptr_t>(port)),
                          2,
                          nullptr,
                          0); // Pin to Core 0 at prio 2 so ready frames transmit immediately
}

uint32_t camHwGetPreviewFrameSeq() {
  return s_previewFrameSeq;
}

bool camHwHasStreamClients() {
  return s_streamClientCount > 0;
}

bool camHwNeedsPreviewJpeg(unsigned long nowMs) {
  if (s_streamClientCount > 0) return true;
  if (s_previewJpgLen == 0) return true;
  unsigned long lastReq = s_lastPreviewReqMs;
  if (lastReq != 0 && (nowMs - lastReq < 3000)) return true;
  if (nowMs - s_lastPreviewEncodeMs >= 1000) return true;
  return false;
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
    // OV2640 PIXFORMAT_RGB565 is stored in big-endian byte order in fb->buf:
    // byte 0 = RRRRRGGG, byte 1 = GGGBBBBB -> 16-bit value = (R<<11)|(G<<5)|B
    const uint8_t *bytes = static_cast<const uint8_t *>(fb->buf);
    for (int y = 0; y < fb->height; y += SAMPLE_STRIDE) {
      const uint8_t *row = bytes + static_cast<size_t>(y) * fb->width * 2;
      for (int x = 0; x < fb->width; x += SAMPLE_STRIDE) {
        uint16_t px = (static_cast<uint16_t>(row[x * 2]) << 8) |
                      static_cast<uint16_t>(row[x * 2 + 1]);
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

namespace {

struct JpgGridCtx {
  const uint8_t *jpg;
  size_t len;
  int scaledW;
  int scaledH;
  float *cellSum;
  uint16_t *cellCount;
  int gridDim;
};

size_t jpgGridReader(void *arg, size_t index, uint8_t *buf, size_t len) {
  JpgGridCtx *ctx = static_cast<JpgGridCtx *>(arg);
  if (!ctx || index >= ctx->len) return 0;
  size_t avail = ctx->len - index;
  size_t toCopy = (len < avail) ? len : avail;
  if (buf && toCopy > 0) {
    memcpy(buf, ctx->jpg + index, toCopy);
  }
  return toCopy;
}

bool jpgGridWriter(void *arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
  JpgGridCtx *ctx = static_cast<JpgGridCtx *>(arg);
  if (!ctx) return false;
  if (!data) {
    ctx->scaledW = static_cast<int>(w);
    ctx->scaledH = static_cast<int>(h);
    return (ctx->scaledW >= 8 && ctx->scaledH >= 8);
  }
  if (ctx->scaledW <= 0 || ctx->scaledH <= 0) return false;

  const int gd = ctx->gridDim;
  for (uint16_t row = 0; row < h; ++row) {
    int py = static_cast<int>(y + row);
    int gy = (py * gd) / ctx->scaledH;
    if (gy >= gd) gy = gd - 1;
    const uint8_t *rgb = data + static_cast<size_t>(row) * w * 3;
    for (uint16_t col = 0; col < w; ++col) {
      int px = static_cast<int>(x + col);
      int gx = (px * gd) / ctx->scaledW;
      if (gx >= gd) gx = gd - 1;
      // TJpgDec outputs 3-byte RGB/BGR pixels; average luminance maps linearly to [0,1].
      uint8_t c0 = rgb[col * 3];
      uint8_t c1 = rgb[col * 3 + 1];
      uint8_t c2 = rgb[col * 3 + 2];
      float luma = (0.299f * c0 + 0.587f * c1 + 0.114f * c2) * (1.0f / 255.0f);
      int idx = gy * gd + gx;
      ctx->cellSum[idx] += luma;
      ctx->cellCount[idx]++;
    }
  }
  return true;
}

} // namespace

float camHwAnalyzeSceneLumaAndMotion(const camera_fb_t *fb,
                                     bool resetMotionBaseline,
                                     float *outMotionScore,
                                     bool *outMotionDetected) {
  if (outMotionScore) *outMotionScore = 0.0f;
  if (outMotionDetected) *outMotionDetected = false;

  if (!fb || !fb->buf || fb->len == 0 ||
      (fb->format != PIXFORMAT_JPEG && (fb->width < 16 || fb->height < 16))) {
    return 0.0f;
  }

  constexpr int GRID_DIM = 8;
  constexpr int NUM_CELLS = GRID_DIM * GRID_DIM;
  constexpr int SAMPLE_STRIDE = 6;

  static float s_prevCellLuma[NUM_CELLS] = {0.0f};
  static bool s_hasBaseline = false;

  float cellSum[NUM_CELLS] = {0.0f};
  uint16_t cellCount[NUM_CELLS] = {0};

  int w = static_cast<int>(fb->width);
  int h = static_cast<int>(fb->height);

  if (fb->format == PIXFORMAT_JPEG) {
    JpgGridCtx ctx = {fb->buf, fb->len, 0, 0, cellSum, cellCount, GRID_DIM};
    if (esp_jpg_decode(fb->len, JPG_SCALE_8X, jpgGridReader, jpgGridWriter, &ctx) != ESP_OK) {
      return 0.0f;
    }
  } else if (fb->format == PIXFORMAT_GRAYSCALE) {
    const uint8_t *pix = static_cast<const uint8_t *>(fb->buf);
    for (int y = 0; y < h; y += SAMPLE_STRIDE) {
      int gy = (y * GRID_DIM) / h;
      if (gy >= GRID_DIM) gy = GRID_DIM - 1;
      const uint8_t *row = pix + static_cast<size_t>(y) * w;
      for (int x = 0; x < w; x += SAMPLE_STRIDE) {
        int gx = (x * GRID_DIM) / w;
        if (gx >= GRID_DIM) gx = GRID_DIM - 1;
        int idx = gy * GRID_DIM + gx;
        cellSum[idx] += static_cast<float>(row[x]) * (1.0f / 255.0f);
        cellCount[idx]++;
      }
    }
  } else {
    const uint8_t *bytes = static_cast<const uint8_t *>(fb->buf);
    for (int y = 0; y < h; y += SAMPLE_STRIDE) {
      int gy = (y * GRID_DIM) / h;
      if (gy >= GRID_DIM) gy = GRID_DIM - 1;
      const uint8_t *row = bytes + static_cast<size_t>(y) * w * 2;
      for (int x = 0; x < w; x += SAMPLE_STRIDE) {
        int gx = (x * GRID_DIM) / w;
        if (gx >= GRID_DIM) gx = GRID_DIM - 1;
        uint16_t px = (static_cast<uint16_t>(row[x * 2]) << 8) |
                      static_cast<uint16_t>(row[x * 2 + 1]);
        uint8_t r = static_cast<uint8_t>((px >> 11) & 0x1F);
        uint8_t g = static_cast<uint8_t>((px >> 5) & 0x3F);
        uint8_t b = static_cast<uint8_t>(px & 0x1F);
        r = static_cast<uint8_t>((r * 255 + 15) / 31);
        g = static_cast<uint8_t>((g * 255 + 31) / 63);
        b = static_cast<uint8_t>((b * 255 + 15) / 31);
        float luma = (0.299f * r + 0.587f * g + 0.114f * b) * (1.0f / 255.0f);
        int idx = gy * GRID_DIM + gx;
        cellSum[idx] += luma;
        cellCount[idx]++;
      }
    }
  }

  float cellLuma[NUM_CELLS] = {0.0f};
  float totalLuma = 0.0f;
  int validCells = 0;
  for (int i = 0; i < NUM_CELLS; ++i) {
    if (cellCount[i] > 0) {
      cellLuma[i] = cellSum[i] / static_cast<float>(cellCount[i]);
      totalLuma += cellLuma[i];
      validCells++;
    }
  }
  float sceneLuma = (validCells > 0) ? (totalLuma / static_cast<float>(validCells)) : 0.0f;

  if (!s_hasBaseline || resetMotionBaseline) {
    for (int i = 0; i < NUM_CELLS; ++i) {
      s_prevCellLuma[i] = cellLuma[i];
    }
    s_hasBaseline = true;
    return sceneLuma;
  }

  // Compute global mean signed shift so camera AEC/AGC brightness drift is canceled out
  float meanSignedDiff = 0.0f;
  for (int i = 0; i < NUM_CELLS; ++i) {
    meanSignedDiff += (cellLuma[i] - s_prevCellLuma[i]);
  }
  meanSignedDiff /= static_cast<float>(NUM_CELLS);

  // In dark scenes (<0.22 luma), pixel amplitudes are small, so use a lower per-cell threshold
  float cellThresh = (sceneLuma < 0.22f) ? 0.013f : 0.026f;
  int activeCells = 0;
  float absDiffSum = 0.0f;

  for (int i = 0; i < NUM_CELLS; ++i) {
    float signedDelta = (cellLuma[i] - s_prevCellLuma[i]) - meanSignedDiff;
    float localDiff = (signedDelta < 0.0f) ? -signedDelta : signedDelta;
    absDiffSum += localDiff;
    if (localDiff >= cellThresh) {
      activeCells++;
    }
    // Exponential moving baseline update
    s_prevCellLuma[i] = 0.55f * s_prevCellLuma[i] + 0.45f * cellLuma[i];
  }

  float absGlobalShift = (meanSignedDiff < 0.0f) ? -meanSignedDiff : meanSignedDiff;
  // Normalize motion score [0..1] based on active spatial cells (3 cells = 0.25, 12 cells = 1.0)
  float score = static_cast<float>(activeCells) / 12.0f;
  if (score > 1.0f) score = 1.0f;
  // Also detect very close full-frame motion (e.g. hand/face right in front of lens)
  if (absGlobalShift > 0.055f && score < 0.5f) {
    score = 0.5f;
  }

  bool motion = (activeCells >= 3) || (absGlobalShift > 0.055f) ||
                ((absDiffSum / static_cast<float>(NUM_CELLS)) >= (cellThresh * 0.65f));

  if (outMotionScore) *outMotionScore = score;
  if (outMotionDetected) *outMotionDetected = motion;
  return sceneLuma;
}
