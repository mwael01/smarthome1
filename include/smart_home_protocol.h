#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Shared Wi-Fi SoftAP & ESP-NOW Protocol (Main Hub + face-stream camera)
// ---------------------------------------------------------------------------
namespace SmartHomeNet {

constexpr char AP_SSID[] = "SmartHome-Net";
constexpr char AP_PASS[] = "smarthome123";
constexpr uint8_t WIFI_CHANNEL = 1;

// Static IPs on the Main Hub's SoftAP subnet (192.168.4.0/24)
constexpr uint8_t HUB_IP[4] = {192, 168, 4, 1};
constexpr uint8_t FACE_CAM_IP[4] = {192, 168, 4, 101};
constexpr uint8_t SUBNET_MASK[4] = {255, 255, 255, 0};

constexpr uint32_t PACKET_MAGIC = 0x53484331; // "SHC1"

enum CamSource : uint8_t {
  CAM_SOURCE_FACE = 1,
};

enum CamEventType : uint8_t {
  CAM_EVENT_RECOGNIZED = 1,
  CAM_EVENT_UNKNOWN = 2,
  CAM_EVENT_HEARTBEAT = 3,
};

struct __attribute__((packed)) CamEventPacket {
  uint32_t magic;
  uint8_t source;     // CamSource
  uint8_t event;      // CamEventType
  int16_t matchId;    // Enrolled face ID, or -1 if unknown
  uint8_t confidence; // 0-100%
  uint32_t seq;
  char label[24];     // Null-terminated face name or status label
};

} // namespace SmartHomeNet
