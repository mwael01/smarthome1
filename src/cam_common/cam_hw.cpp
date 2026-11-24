#include "cam_hw.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace {

constexpr int CAMERA_XCLK_HZ = 20000000;

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

} // namespace

bool camHwInitCamera(framesize_t frameSize, int jpegQuality) {
  if (!psramFound()) {
    Serial.println("[SERIAL CAM] PSRAM is required");
    return false;
  }
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
  // Match the 20 MHz JPEG streaming clock in the Arduino CameraWebServer example.
  config.xclk_freq_hz = CAMERA_XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  // Reserve DMA buffer space for the largest preset, then select the startup size.
  config.frame_size = FRAMESIZE_VGA;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = jpegQuality;
  config.fb_count = 2;

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
    if (s->set_hmirror) s->set_hmirror(s, 0);
    if (s->set_vflip) s->set_vflip(s, 0);
    if (s->set_contrast) s->set_contrast(s, 0);
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

  // Drain frames captured while the sensor settings settle.
  delay(80);
  for (int i = 0; i < 10; ++i) {
    camera_fb_t *flushFb = esp_camera_fb_get();
    if (flushFb) {
      esp_camera_fb_return(flushFb);
    }
    delay(15);
  }
  Serial.printf("[SERIAL CAM] PID 0x%04X, sensor JPEG, %u frame buffers, XCLK %d MHz\n",
                s ? s->id.PID : 0, config.fb_count, CAMERA_XCLK_HZ / 1000000);
  return s != nullptr;
}
