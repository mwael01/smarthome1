#pragma once

#include <esp_camera.h>
#include <stdint.h>

// ESP32-S3-EYE-compatible camera connector pinout.
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

bool camHwInitCamera(framesize_t frameSize, int jpegQuality);
