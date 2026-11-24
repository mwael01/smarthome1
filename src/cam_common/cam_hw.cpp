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
#include <atomic>
#include <cstring>

namespace {

constexpr int FLASH_LEDC_CHANNEL = 2; // Avoid LEDC channel 0 used by camera XCLK
constexpr int MAX_STREAM_CLIENTS = 4;
constexpr size_t PART_HEADER_RESERVE = 96;
constexpr size_t PART_TRAILER_RESERVE = 2;
constexpr size_t TCP_SEND_CHUNK_BYTES = 4096;
constexpr unsigned long FRAME_SEND_STALL_MS = 750;
constexpr unsigned long FRAME_SEND_LIMIT_MS = 1500;
constexpr unsigned long FRAME_IDLE_TIMEOUT_MS = 3000;

SemaphoreHandle_t s_previewMutex = nullptr;
uint8_t *s_previewAlloc = nullptr;
uint8_t *s_previewJpg = nullptr;
size_t s_previewJpgLen = 0;
size_t s_previewJpgCap = 0;
size_t s_previewHeaderLen = 0;
volatile uint32_t s_previewFrameSeq = 0;
std::atomic<int> s_streamClientCount{0};
volatile unsigned long s_lastPreviewReqMs = 0;
volatile unsigned long s_lastPreviewEncodeMs = 0;
bool s_streamServerStarted = false;
bool s_flashAvailable = false;

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

// Allow a frame to finish through brief Wi-Fi stalls while bounding the time
// spent on a blocked viewer. A partially sent frame must finish or close.
bool sendPacketAll(int fd, const uint8_t *buf, size_t len) {
  if (fd < 0) return false;

  size_t sent = 0;
  const unsigned long startMs = millis();
  unsigned long lastProgressMs = startMs;
  while (sent < len) {
    const unsigned long now = millis();
    if (now - lastProgressMs >= FRAME_SEND_STALL_MS || now - startMs >= FRAME_SEND_LIMIT_MS) {
      return false;
    }
    size_t chunk = min(len - sent, TCP_SEND_CHUNK_BYTES);
    ssize_t n = lwip_send(fd, buf + sent, chunk, 0);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      lastProgressMs = millis();
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      vTaskDelay(1);
    } else {
      return false;
    }
  }
  return true;
}

bool readStreamRequest(WiFiClient &client) {
  constexpr char headerEnd[] = "\r\n\r\n";
  const unsigned long startMs = millis();
  size_t matched = 0;
  while (client.connected() && millis() - startMs < 200) {
    int byte = client.read();
    if (byte < 0) {
      vTaskDelay(1);
      continue;
    }
    if (byte == headerEnd[matched]) {
      if (++matched == sizeof(headerEnd) - 1) return true;
    } else {
      matched = byte == '\r' ? 1 : 0;
    }
  }
  return false;
}

struct StreamClient {
  WiFiClient socket;
  SemaphoreHandle_t frameReady = nullptr;
  std::atomic<bool> occupied{false};
};

StreamClient s_streamClients[MAX_STREAM_CLIENTS];

void serveStreamClient(StreamClient &client) {
  static const char STREAM_HEADER[] =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Connection: close\r\n"
      "Cache-Control: no-cache, no-store, must-revalidate, max-age=0\r\n\r\n";

  int fd = client.socket.fd();
  struct timeval timeout = {0, 50000};
  if (fd < 0 || lwip_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) != 0 ||
      !readStreamRequest(client.socket) ||
      !sendPacketAll(fd, reinterpret_cast<const uint8_t *>(STREAM_HEADER),
                     sizeof(STREAM_HEADER) - 1)) {
    return;
  }

  uint8_t *packet = nullptr;
  size_t capacity = 0;
  uint32_t lastSeq = 0;
  unsigned long lastFrameMs = millis();
  while (client.socket.connected() && millis() - lastFrameMs < FRAME_IDLE_TIMEOUT_MS) {
    size_t packetLen = 0;
    if (xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
      if (s_previewJpgLen != 0 && s_previewFrameSeq != lastSeq) {
        size_t needed = s_previewHeaderLen + s_previewJpgLen + PART_TRAILER_RESERVE;
        if (needed > capacity) {
          size_t newCapacity = (needed + 4095) & ~size_t(4095);
          auto *buffer = static_cast<uint8_t *>(ps_realloc(packet, newCapacity));
          if (!buffer) buffer = static_cast<uint8_t *>(realloc(packet, newCapacity));
          if (buffer) {
            packet = buffer;
            capacity = newCapacity;
          }
        }
        if (needed <= capacity) {
          memcpy(packet, s_previewJpg - s_previewHeaderLen, needed);
          packetLen = needed;
          lastSeq = s_previewFrameSeq;
        }
      }
      xSemaphoreGive(s_previewMutex);
    }

    // Only one frame per viewer can be in flight. After sending it, pick up
    // the newest frame available and leave older frames behind.
    if (packetLen > 0) {
      if (!sendPacketAll(fd, packet, packetLen)) break;
      lastFrameMs = millis();
    } else {
      xSemaphoreTake(client.frameReady, pdMS_TO_TICKS(100));
    }
  }
  free(packet);
}

void streamClientTask(void *arg) {
  auto &client = *static_cast<StreamClient *>(arg);
  serveStreamClient(client);
  client.socket.stop();
  s_streamClientCount.fetch_sub(1);
  client.occupied.store(false);
  vTaskDelete(nullptr);
}

void camStreamTask(void *pvParameters) {
  uint16_t port = static_cast<uint16_t>(reinterpret_cast<uintptr_t>(pvParameters));
  WiFiServer server(port ? port : 81);
  server.begin();
  server.setNoDelay(true);
  Serial.printf("[CAM_STREAM] MJPEG live stream server started on port %u\n", port ? port : 81);

  while (true) {
    WiFiClient socket = server.available();
    if (!socket) {
      vTaskDelay(1);
      continue;
    }

    StreamClient *slot = nullptr;
    for (auto &client : s_streamClients) {
      if (client.frameReady && !client.occupied.load()) {
        slot = &client;
        break;
      }
    }

    if (slot) {
      slot->socket = socket;
      slot->occupied.store(true);
      s_streamClientCount.fetch_add(1);
      if (xTaskCreatePinnedToCore(streamClientTask, "cam_viewer", 4096, slot, 2, nullptr, 0) == pdPASS) {
        continue;
      }
      s_streamClientCount.fetch_sub(1);
      slot->socket = WiFiClient();
      slot->occupied.store(false);
    }
    static const char BUSY[] =
        "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
    lwip_send(socket.fd(), BUSY, sizeof(BUSY) - 1, MSG_DONTWAIT);
    socket.stop();
  }
}

} // namespace

void camHwInitLeds() {
#if CAM_HW_STATUS_LED_PIN >= 0
  pinMode(CAM_HW_STATUS_LED_PIN, OUTPUT);
  camHwSetStatusLed(false);
#endif

#if CAM_HW_FLASH_LED_PIN >= 0
  if (ledcSetup(FLASH_LEDC_CHANNEL, 5000, 8) > 0) {
    ledcWrite(FLASH_LEDC_CHANNEL, 0);
    ledcAttachPin(CAM_HW_FLASH_LED_PIN, FLASH_LEDC_CHANNEL);
    s_flashAvailable = true;
  }
#endif
}

bool camHwFlashAvailable() {
  return s_flashAvailable;
}

void camHwSetStatusLed(bool on) {
#if CAM_HW_STATUS_LED_PIN >= 0
  // AI-Thinker red status LED on GPIO33 is active-LOW
  digitalWrite(CAM_HW_STATUS_LED_PIN, on ? LOW : HIGH);
#endif
}

void camHwSetFlashLed(uint8_t duty) {
  if (s_flashAvailable) ledcWrite(FLASH_LEDC_CHANNEL, duty);
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
  const bool hasPsram = psramFound();
  // Reserve enough DMA buffer space for all advertised presets. The sensor
  // switches to the selected size before the application publishes frames.
  config.frame_size = hasPsram ? FRAMESIZE_VGA : frameSize;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.jpeg_quality = jpegQuality;
  // Triple buffering keeps capture DMA fed while the stream task transmits the
  // previous frame. This is especially important on the S3 camera with PSRAM.
  config.fb_count = hasPsram ? 3 : 1;

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
    if (!hasPsram && config.frame_size > FRAMESIZE_QVGA) {
      config.frame_size = FRAMESIZE_QVGA;
    }
    config.fb_count = hasPsram ? 3 : 1;

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
    if (frameSize != config.frame_size && (!s->set_framesize || s->set_framesize(s, frameSize) != 0)) {
      Serial.println("[CAM_HW] Failed to apply startup resolution");
      return false;
    }
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
  for (auto &client : s_streamClients) {
    if (!client.frameReady) client.frameReady = xSemaphoreCreateBinary();
  }
}

void publishPreviewJpeg(const uint8_t *jpgBuf, size_t jpgLen) {
  if (!jpgBuf || jpgLen == 0) return;

  if (s_previewMutex && xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(40)) == pdTRUE) {
    if (jpgLen > s_previewJpgCap) {
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
      char header[PART_HEADER_RESERVE];
      s_previewHeaderLen = snprintf(header, sizeof(header),
                                    "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                    static_cast<unsigned>(jpgLen));
      memcpy(s_previewJpg - s_previewHeaderLen, header, s_previewHeaderLen);
      memcpy(s_previewJpg, jpgBuf, jpgLen);
      s_previewJpg[jpgLen] = '\r';
      s_previewJpg[jpgLen + 1] = '\n';
      s_previewJpgLen = jpgLen;
      s_previewFrameSeq++;
      s_lastPreviewEncodeMs = millis();
    }
    xSemaphoreGive(s_previewMutex);

    for (auto &client : s_streamClients) {
      if (client.occupied.load()) xSemaphoreGive(client.frameReady);
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
  if (s_streamServerStarted || !s_previewMutex) return;
  s_streamServerStarted = xTaskCreatePinnedToCore(camStreamTask,
                          "cam_stream",
                          6144,
                          reinterpret_cast<void *>(static_cast<uintptr_t>(port)),
                          2,
                          nullptr,
                          0) == pdPASS;
  if (!s_streamServerStarted) Serial.println("[CAM_STREAM] Could not start stream task");
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
