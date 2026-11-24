#pragma once

#include <WebServer.h>
#include <esp_camera.h>
#include <stdint.h>

// Camera pin maps for the supported stream boards.
#if defined(CAMERA_MODEL_AI_THINKER)
#define CAM_HW_PWDN_GPIO_NUM  32
#define CAM_HW_RESET_GPIO_NUM -1
#define CAM_HW_XCLK_GPIO_NUM   0
#define CAM_HW_SIOD_GPIO_NUM  26
#define CAM_HW_SIOC_GPIO_NUM  27
#define CAM_HW_Y9_GPIO_NUM    35
#define CAM_HW_Y8_GPIO_NUM    34
#define CAM_HW_Y7_GPIO_NUM    39
#define CAM_HW_Y6_GPIO_NUM    36
#define CAM_HW_Y5_GPIO_NUM    21
#define CAM_HW_Y4_GPIO_NUM    19
#define CAM_HW_Y3_GPIO_NUM    18
#define CAM_HW_Y2_GPIO_NUM     5
#define CAM_HW_VSYNC_GPIO_NUM 25
#define CAM_HW_HREF_GPIO_NUM  23
#define CAM_HW_PCLK_GPIO_NUM  22
#define CAM_HW_FLASH_LED_PIN   4
#define CAM_HW_STATUS_LED_PIN 33
#elif defined(CAMERA_MODEL_ESP32S3_EYE)
#define CAM_HW_PWDN_GPIO_NUM  -1
#define CAM_HW_RESET_GPIO_NUM -1
#define CAM_HW_XCLK_GPIO_NUM  15
#define CAM_HW_SIOD_GPIO_NUM  4
#define CAM_HW_SIOC_GPIO_NUM  5
#define CAM_HW_Y9_GPIO_NUM    16
#define CAM_HW_Y8_GPIO_NUM    17
#define CAM_HW_Y7_GPIO_NUM    18
#define CAM_HW_Y6_GPIO_NUM    12
#define CAM_HW_Y5_GPIO_NUM    10
#define CAM_HW_Y4_GPIO_NUM     8
#define CAM_HW_Y3_GPIO_NUM     9
#define CAM_HW_Y2_GPIO_NUM    11
#define CAM_HW_VSYNC_GPIO_NUM  6
#define CAM_HW_HREF_GPIO_NUM   7
#define CAM_HW_PCLK_GPIO_NUM  13
// ESP32-S3-EYE has no dedicated flash. GPIO2 is mic data and GPIO48 is LCD backlight.
#ifndef CAM_HW_FLASH_LED_PIN
#define CAM_HW_FLASH_LED_PIN  -1
#endif
#define CAM_HW_STATUS_LED_PIN -1
#else
#error "Define a supported camera model"
#endif

void camHwInitLeds();
bool camHwFlashAvailable();
void camHwSetStatusLed(bool on);
void camHwSetFlashLed(uint8_t duty);

bool camHwInitCamera(pixformat_t pixelFormat,
                     framesize_t frameSize,
                     int jpegQuality,
                     bool hmirror,
                     bool vflip,
                     int8_t contrast = 0);

void camHwApplySensorSettings(bool hmirror,
                              bool vflip,
                              uint8_t flashDuty,
                              int8_t contrast = 0);

void camHwInitPreview();
void camHwUpdatePreviewJpeg(const camera_fb_t *fb,
                            pixformat_t srcFormat,
                            uint8_t quality = 58);
void camHwUpdatePreviewJpegBytes(const uint8_t *jpgBuf, size_t jpgLen);
void camHwHandlePreviewRequest(WebServer &server);
void camHwStartStreamServer(uint16_t port = 81);
uint32_t camHwGetPreviewFrameSeq();
bool camHwHasStreamClients();
bool camHwNeedsPreviewJpeg(unsigned long nowMs);

// Estimate mean scene luminance [0,1] from a camera frame buffer (RGB565 or grayscale).
float camHwEstimateMeanLuma(const camera_fb_t *fb);

// Estimate mean scene luminance [0,1] and localized motion using an 8x8 spatial block grid.
// Cancels global AEC/AGC brightness shifts so low-light movement is reliably detected.
float camHwAnalyzeSceneLumaAndMotion(const camera_fb_t *fb,
                                     bool resetMotionBaseline,
                                     float *outMotionScore,
                                     bool *outMotionDetected);
