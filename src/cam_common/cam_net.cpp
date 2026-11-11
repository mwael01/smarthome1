#include "cam_net.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstring>

namespace {

constexpr unsigned long HUB_CONNECT_GRACE_MS = 12000;
constexpr unsigned long STA_RETRY_INTERVAL_MS = 10000;
constexpr unsigned long STA_RETRY_FALLBACK_AP_MS = 30000;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 5000;

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

SmartHomeNet::CamSource s_source = SmartHomeNet::CAM_SOURCE_FACE;
SemaphoreHandle_t s_netMutex = nullptr;

bool s_espNowReady = false;
bool s_hubPeerAdded = false;
uint8_t s_hubBssid[6] = {0};
uint8_t s_hubChannel = 0;

volatile bool s_fallbackApActive = false;
unsigned long s_bootMs = 0;
unsigned long s_lastStaAttemptMs = 0;
unsigned long s_lastHeartbeatMs = 0;

volatile bool s_lastSendOk = false;
volatile bool s_lastEventSendOk = false;
volatile bool s_pendingTxIsHeartbeat = true;
uint32_t s_seq = 0;
volatile uint32_t s_sentCount = 0;

void onEspNowSent(const uint8_t * /*mac_addr*/, esp_now_send_status_t status) {
  bool ok = (status == ESP_NOW_SEND_SUCCESS);
  s_lastSendOk = ok;
  if (!s_pendingTxIsHeartbeat) {
    s_lastEventSendOk = ok;
  }
}

bool isNonZeroMac(const uint8_t *mac) {
  if (!mac) return false;
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != 0) return true;
  }
  return false;
}

bool ensurePeer(const uint8_t *mac, wifi_interface_t ifidx) {
  if (!s_espNowReady || !mac) return false;
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0; // Use current home channel
  peer.ifidx = ifidx;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void syncHubPeerLocked() {
  if (!s_espNowReady) return;
  if (WiFi.status() != WL_CONNECTED) return;

  const uint8_t *bssid = WiFi.BSSID();
  if (!isNonZeroMac(bssid)) return;

  uint8_t curChannel = WiFi.channel();

  if (!s_hubPeerAdded || memcmp(s_hubBssid, bssid, 6) != 0 || s_hubChannel != curChannel) {
    if (s_hubPeerAdded && esp_now_is_peer_exist(s_hubBssid)) {
      esp_now_del_peer(s_hubBssid);
    }
    memcpy(s_hubBssid, bssid, 6);
    s_hubChannel = curChannel;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, s_hubBssid, 6);
    peer.channel = 0; // 0 = automatically follow active STA channel
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    s_hubPeerAdded = (esp_now_add_peer(&peer) == ESP_OK);
    if (s_hubPeerAdded) {
      Serial.printf("ESP-NOW Hub peer registered: %02X:%02X:%02X:%02X:%02X:%02X (STA ch %d, auto-track)\n",
                    s_hubBssid[0], s_hubBssid[1], s_hubBssid[2],
                    s_hubBssid[3], s_hubBssid[4], s_hubBssid[5], curChannel);
    }
  }
}

void startFallbackApLocked() {
  if (s_fallbackApActive) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);

  IPAddress apIp = (s_source == SmartHomeNet::CAM_SOURCE_FACE)
                       ? IPAddress(192, 168, 11, 1)
                       : IPAddress(192, 168, 12, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, apIp, subnet);

  String apSsid = camNetFallbackApSsid();
  if (WiFi.softAP(apSsid.c_str(), SmartHomeNet::AP_PASS, SmartHomeNet::WIFI_CHANNEL, 0, 2)) {
    s_fallbackApActive = true;
    Serial.print("Fallback setup AP active: ");
    Serial.print(apSsid);
    Serial.print(" at http://");
    Serial.println(WiFi.softAPIP());
  }
}

void stopFallbackApLocked() {
  if (!s_fallbackApActive) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  s_fallbackApActive = false;
  Serial.println("Main Hub reachable; fallback setup AP disabled.");
}

} // namespace

void camNetInit(SmartHomeNet::CamSource source) {
  s_source = source;
  s_bootMs = millis();
  if (!s_netMutex) {
    s_netMutex = xSemaphoreCreateMutex();
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  const uint8_t *ipArr = (source == SmartHomeNet::CAM_SOURCE_FACE)
                             ? SmartHomeNet::FACE_CAM_IP
                             : SmartHomeNet::PLATE_CAM_IP;
  IPAddress localIp(ipArr[0], ipArr[1], ipArr[2], ipArr[3]);
  IPAddress hubIp(SmartHomeNet::HUB_IP[0], SmartHomeNet::HUB_IP[1],
                  SmartHomeNet::HUB_IP[2], SmartHomeNet::HUB_IP[3]);
  IPAddress subnet(SmartHomeNet::SUBNET_MASK[0], SmartHomeNet::SUBNET_MASK[1],
                   SmartHomeNet::SUBNET_MASK[2], SmartHomeNet::SUBNET_MASK[3]);
  WiFi.config(localIp, hubIp, subnet);

  // Dynamic channel scan: do not specify channel so STA scans all channels to find Hub
  WiFi.begin(SmartHomeNet::AP_SSID, SmartHomeNet::AP_PASS);
  s_lastStaAttemptMs = s_bootMs;

  if (esp_now_init() == ESP_OK) {
    s_espNowReady = true;
    esp_now_register_send_cb(onEspNowSent);
    ensurePeer(BROADCAST_MAC, WIFI_IF_STA);
    Serial.println("ESP-NOW initialized on STA interface.");
  } else {
    s_espNowReady = false;
    Serial.println("ERROR: esp_now_init failed!");
  }
}

void camNetLoop() {
  unsigned long now = millis();
  if (s_netMutex && xSemaphoreTake(s_netMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }

  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) {
    stopFallbackApLocked();
    syncHubPeerLocked();
  } else {
    if (!s_fallbackApActive && (now - s_bootMs >= HUB_CONNECT_GRACE_MS)) {
      startFallbackApLocked();
    }
    unsigned long retryMs = s_fallbackApActive ? STA_RETRY_FALLBACK_AP_MS : STA_RETRY_INTERVAL_MS;
    if (now - s_lastStaAttemptMs >= retryMs) {
      s_lastStaAttemptMs = now;
      // Avoid resetting SoftAP HTTP state with full WiFi.begin() while serving fallback clients
      if (!WiFi.reconnect()) {
        WiFi.begin(SmartHomeNet::AP_SSID, SmartHomeNet::AP_PASS);
      }
    }
  }

  if (s_netMutex) {
    xSemaphoreGive(s_netMutex);
  }

  if (now - s_lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    s_lastHeartbeatMs = now;
    camNetSendEvent(SmartHomeNet::CAM_EVENT_HEARTBEAT, 0, 100,
                    s_source == SmartHomeNet::CAM_SOURCE_FACE ? "FACE-CAM" : "PLATE-CAM");
  }
}

bool camNetPostHttpEvent(SmartHomeNet::CamEventType event,
                         int16_t matchId,
                         uint8_t confidence,
                         const char *label) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClient client;
  client.setTimeout(300);
  if (!client.connect("192.168.4.1", 80, 300)) {
    return false;
  }

  char jsonBody[128];
  snprintf(jsonBody, sizeof(jsonBody),
           "{\"source\":%d,\"event\":%d,\"matchId\":%d,\"confidence\":%u,\"label\":\"%s\"}",
           static_cast<int>(s_source),
           static_cast<int>(event),
           static_cast<int>(matchId),
           static_cast<unsigned>(confidence),
           label ? label : "none");

  char req[256];
  int reqLen = snprintf(req, sizeof(req),
                        "POST /api/cam/event HTTP/1.1\r\n"
                        "Host: 192.168.4.1\r\n"
                        "Content-Type: application/json\r\n"
                        "Content-Length: %d\r\n"
                        "Connection: close\r\n\r\n%s",
                        static_cast<int>(strlen(jsonBody)), jsonBody);

  client.write(reinterpret_cast<const uint8_t *>(req), reqLen);
  client.flush();
  client.stop();
  return true;
}

bool camNetSendEvent(SmartHomeNet::CamEventType event,
                     int16_t matchId,
                     uint8_t confidence,
                     const char *label) {
  if (!s_espNowReady) return false;

  if (s_netMutex && xSemaphoreTake(s_netMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }

  syncHubPeerLocked();

  SmartHomeNet::CamEventPacket pkt = {};
  pkt.magic = SmartHomeNet::PACKET_MAGIC;
  pkt.source = static_cast<uint8_t>(s_source);
  pkt.event = static_cast<uint8_t>(event);
  pkt.matchId = matchId;
  pkt.confidence = min<uint8_t>(confidence, 100);
  pkt.seq = ++s_seq;
  if (label) {
    strncpy(pkt.label, label, sizeof(pkt.label) - 1);
    pkt.label[sizeof(pkt.label) - 1] = '\0';
  }

  const uint8_t *targetMac = (WiFi.status() == WL_CONNECTED && s_hubPeerAdded)
                                 ? s_hubBssid
                                 : BROADCAST_MAC;
  if (targetMac == BROADCAST_MAC) {
    ensurePeer(BROADCAST_MAC, WIFI_IF_STA);
  }

  s_pendingTxIsHeartbeat = (event == SmartHomeNet::CAM_EVENT_HEARTBEAT);
  esp_err_t err = esp_now_send(targetMac,
                               reinterpret_cast<const uint8_t *>(&pkt),
                               sizeof(pkt));
  if (err != ESP_OK && targetMac != BROADCAST_MAC) {
    ensurePeer(BROADCAST_MAC, WIFI_IF_STA);
    err = esp_now_send(BROADCAST_MAC,
                       reinterpret_cast<const uint8_t *>(&pkt),
                       sizeof(pkt));
  }
  if (err == ESP_OK && event != SmartHomeNet::CAM_EVENT_HEARTBEAT) {
    s_sentCount = s_sentCount + 1;
  }

  if (s_netMutex) {
    xSemaphoreGive(s_netMutex);
  }

  // Dual-Path: for critical access events (RECOGNIZED or UNKNOWN),
  // send via HTTP POST to the Hub as a zero-loss fallback if connected to Wi-Fi.
  if (event != SmartHomeNet::CAM_EVENT_HEARTBEAT && WiFi.status() == WL_CONNECTED) {
    camNetPostHttpEvent(event, matchId, confidence, label);
  }

  return err == ESP_OK;
}

bool camNetHubConnected() {
  return WiFi.status() == WL_CONNECTED;
}

bool camNetFallbackApActive() {
  return s_fallbackApActive;
}

bool camNetLastSendOk() {
  return (s_sentCount > 0) ? s_lastEventSendOk : s_lastSendOk;
}

uint32_t camNetSentCount() {
  return s_sentCount;
}

String camNetIpString() {
  String out = "0.0.0.0";
  bool locked = s_netMutex && (xSemaphoreTake(s_netMutex, pdMS_TO_TICKS(10)) == pdTRUE);
  if (WiFi.status() == WL_CONNECTED) {
    out = WiFi.localIP().toString();
  } else if (s_fallbackApActive) {
    out = WiFi.softAPIP().toString();
  }
  if (locked) {
    xSemaphoreGive(s_netMutex);
  }
  return out;
}

String camNetFallbackApSsid() {
  return (s_source == SmartHomeNet::CAM_SOURCE_FACE)
             ? String("SmartHome-FaceCam")
             : String("SmartHome-PlateCam");
}

String camNetFallbackApIp() {
  return (s_source == SmartHomeNet::CAM_SOURCE_FACE)
             ? String("192.168.11.1")
             : String("192.168.12.1");
}

String camNetMacString() {
  return WiFi.macAddress();
}
