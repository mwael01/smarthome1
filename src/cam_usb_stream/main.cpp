#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <stddef.h>
#include <esp_log.h>
#include <esp_rom_crc.h>

#include "cam_hw.h"

#if !defined(CAMERA_USB_UART)
#include <USB.h>
#include <USBCDC.h>
#endif

namespace {
constexpr size_t kMaxJpegSize = 128 * 1024;
constexpr int kDefaultQuality = 16;
constexpr uint32_t kLightTimeoutMs = 4000;
#if defined(CAMERA_UART_LIGHT_PIN)
constexpr int kProfileLightPin = CAMERA_UART_LIGHT_PIN;
constexpr int kProfileWhiteLevel = 255;
static_assert(kProfileLightPin == 38 || kProfileLightPin == 48,
              "The camera light profile must use GPIO38 or GPIO48");
#else
constexpr int kProfileLightPin = -1;
constexpr int kProfileWhiteLevel = 0;
#endif
constexpr uint8_t kFrameMagic[] = {0xff, 0xff, 0x00, 'U', 'S', 'B', 'C', '1'};

// All numeric fields are little-endian on the ESP32-S3.
struct FrameHeader {
  uint8_t magic[sizeof(kFrameMagic)];
  uint32_t jpegSize;
  uint32_t sequence;
  uint32_t captureUs;
  uint16_t width;
  uint16_t height;
  uint32_t crc;
};
static_assert(sizeof(FrameHeader) == 28, "Unexpected USB frame header layout");

struct Preset {
  const char *name;
  framesize_t size;
  uint16_t width;
  uint16_t height;
};
constexpr Preset kPresets[] = {
    {"qvga", FRAMESIZE_QVGA, 320, 240},
    {"hvga", FRAMESIZE_HVGA, 480, 320},
    {"vga", FRAMESIZE_VGA, 640, 480},
};

#if defined(CAMERA_USB_UART)
#ifndef CAMERA_USB_UART_BAUD
#define CAMERA_USB_UART_BAUD 2000000
#endif
HardwareSerial &cameraPort = Serial;
#else
USBCDC cameraPort;
#endif

uint8_t *packet = nullptr;
const Preset *activePreset = &kPresets[1];
int activeQuality = kDefaultQuality;
uint8_t framesToDiscard = 0;
uint32_t sequence = 0;
bool streaming = false;
Adafruit_NeoPixel light(1, -1, NEO_GRB + NEO_KHZ800);
int activeWhite = -1;
uint32_t lightCommandAtMs = 0;

void setWhite(int pin, uint8_t level) {
  if (light.getPin() != pin) {
    if (light.getPin() >= 0) {
      light.clear();
      light.show();
    }
    light.setPin(pin);
    activeWhite = -1;
    if (pin >= 0) light.begin();
  }
  if (pin >= 0 && activeWhite != level) {
    light.setPixelColor(0, level, level, level);
    light.show();
    activeWhite = level;
  }
}

void stopStreaming() {
  streaming = false;
  setWhite(-1, 0);
}

bool linkConnected() {
#if defined(CAMERA_USB_UART)
  return true;
#else
  return static_cast<bool>(cameraPort);
#endif
}

bool sendPacket(size_t size) {
#if defined(CAMERA_USB_UART)
  return cameraPort.write(packet, size) == size;
#else
  size_t offset = 0;
  uint32_t lastProgress = micros();
  while (offset < size && linkConnected()) {
    const int available = cameraPort.availableForWrite();
    if (available > 0) {
      // A larger write can spin inside Arduino's CDC driver when the host stalls.
      const size_t chunk = min(size - offset, static_cast<size_t>(available));
      const size_t sent = cameraPort.write(packet + offset, chunk);
      if (sent > 0) {
        offset += sent;
        lastProgress = micros();
        continue;
      }
    }
    const uint32_t stalledUs = micros() - lastProgress;
    if (stalledUs >= 150000) {
      return false;
    }
    // Short waits keep the 64-byte FIFO fed. Yield when backpressure persists.
    if (stalledUs < 500) {
      delayMicroseconds(10);
    } else {
      delay(1);
    }
  }
  return offset == size;
#endif
}

void applyCommand(const char *command) {
  if (strcmp(command, "STOP") == 0) {
    stopStreaming();
    return;
  }

  char name[8];
  char pinName[8];
  int quality;
  int white = 0;
  char extra;
  int fields = sscanf(command, "START %7s %2d %7s %3d %c",
                      name, &quality, pinName, &white, &extra);
  if ((fields != 2 && fields != 4) ||
      quality < 10 || quality > 63) {
    return;
  }
  int pin = kProfileLightPin;
  if (fields == 4) {
    if ((strcmp(pinName, "38") != 0 && strcmp(pinName, "48") != 0) ||
        white < 0 || white > 255) {
      return;
    }
    pin = atoi(pinName);
    // Keep the stream usable if a stale command names the other profile's pin.
    if (kProfileLightPin >= 0 && pin != kProfileLightPin) {
      pin = -1;
    }
  } else {
    white = kProfileWhiteLevel;
  }
  for (const Preset &preset : kPresets) {
    if (strcmp(name, preset.name) != 0) {
      continue;
    }
    sensor_t *sensor = esp_camera_sensor_get();
    if (activePreset != &preset) {
      if (!sensor->set_framesize || sensor->set_framesize(sensor, preset.size) != 0) {
        return;
      }
      activePreset = &preset;
      framesToDiscard = 2;
    }
    if (activeQuality != quality) {
      if (!sensor->set_quality || sensor->set_quality(sensor, quality) != 0) {
        return;
      }
      activeQuality = quality;
      framesToDiscard = 2;
    }
    setWhite(pin, white);
    lightCommandAtMs = millis();
    streaming = true;
    return;
  }
}

void readCommands() {
  static char command[32];
  static size_t length = 0;
  static bool overflow = false;
  for (int count = 0; count < 64 && cameraPort.available(); ++count) {
    const int byte = cameraPort.read();
    if (byte == '\n') {
      if (!overflow) {
        command[length] = '\0';
        applyCommand(command);
      }
      length = 0;
      overflow = false;
    } else if (byte >= 0 && byte != '\r') {
      if (length < sizeof(command) - 1) {
        command[length++] = static_cast<char>(byte);
      } else {
        overflow = true;
      }
    }
  }
}
}  // namespace

void setup() {
#if defined(CAMERA_USB_UART)
  cameraPort.setTxBufferSize(4096);
  cameraPort.begin(CAMERA_USB_UART_BAUD);
  cameraPort.setDebugOutput(false);
#else
  Serial.begin(115200);
  cameraPort.enableReboot(false);
  cameraPort.begin();
  USB.productName("SmartHome Camera USB");
  if (!USB.begin()) {
    Serial.println("[USB CAM] Native USB initialization failed");
    return;
  }
#endif

  if (!camHwInitCamera(activePreset->size, activeQuality)) {
    return;
  }
  packet = static_cast<uint8_t *>(ps_malloc(sizeof(FrameHeader) + kMaxJpegSize));
  if (!packet) {
    Serial.println("[SERIAL CAM] Could not allocate a frame buffer");
  }
#if defined(CAMERA_USB_UART)
  // UART0 also carries boot diagnostics. Keep logs out of the binary stream.
  esp_log_level_set("*", ESP_LOG_NONE);
#endif
}

void loop() {
  if (!packet || !linkConnected()) {
    stopStreaming();
    delay(5);
    return;
  }
  readCommands();
  if (activeWhite > 0 && millis() - lightCommandAtMs >= kLightTimeoutMs) {
    setWhite(-1, 0);
  }
  if (!streaming) {
    delay(5);
    return;
  }

  camera_fb_t *frame = esp_camera_fb_get();
  if (!frame) {
    delay(1);
    return;
  }
  const bool settled = framesToDiscard == 0;
  if (!settled) {
    --framesToDiscard;
  }
  const size_t size = frame->len;
  const bool valid = settled && frame->format == PIXFORMAT_JPEG &&
                     frame->width == activePreset->width && frame->height == activePreset->height &&
                     size >= 4 && size <= kMaxJpegSize &&
                     frame->buf[0] == 0xff && frame->buf[1] == 0xd8 &&
                     frame->buf[size - 2] == 0xff && frame->buf[size - 1] == 0xd9;
  if (!valid) {
    esp_camera_fb_return(frame);
    return;
  }

  FrameHeader header = {};
  memcpy(header.magic, kFrameMagic, sizeof(kFrameMagic));
  header.jpegSize = size;
  header.sequence = ++sequence;
  header.captureUs = static_cast<uint32_t>(frame->timestamp.tv_sec * 1000000ULL +
                                           frame->timestamp.tv_usec);
  header.width = frame->width;
  header.height = frame->height;
  uint8_t *jpeg = packet + sizeof(header);
  memcpy(jpeg, frame->buf, size);
  // Release the driver buffer before CRC and any waiting on the cable.
  esp_camera_fb_return(frame);
  const uint32_t headerCrc =
      esp_rom_crc32_le(0, reinterpret_cast<const uint8_t *>(&header), offsetof(FrameHeader, crc));
  header.crc = esp_rom_crc32_le(headerCrc, jpeg, size);
  memcpy(packet, &header, sizeof(header));
  if (!sendPacket(sizeof(header) + size)) {
    // The receiver retries START after a gap and resynchronizes on the next packet.
    stopStreaming();
  }
  delay(1);
}
