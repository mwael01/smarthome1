#include "cam_net.h"
#include "smart_home_json.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstring>

namespace {

constexpr unsigned long HUB_CONNECT_GRACE_MS = 12000;
constexpr unsigned long FALLBACK_RETRY_INTERVAL_MS = 30000;
constexpr unsigned long FALLBACK_PROBE_TIMEOUT_MS = 5000;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 5000;

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

SmartHomeNet::CamSource s_source = SmartHomeNet::CAM_SOURCE_FACE;
SemaphoreHandle_t s_netMutex = nullptr;

bool s_espNowReady = false;
bool s_hubPeerAdded = false;
uint8_t s_hubBssid[6] = {0};
uint8_t s_hubChannel = 0;

volatile bool s_fallbackApActive = false;
bool s_wasStaConnected = false;
bool s_fallbackProbeActive = false;
unsigned long s_lastStaAttemptMs = 0;
unsigned long s_lastApAttemptMs = 0;
unsigned long s_lastHeartbeatMs = 0;

volatile bool s_lastSendOk = true;
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

void startHubConnection() {
  const uint8_t *ip = SmartHomeNet::FACE_CAM_IP;
  const uint8_t *hub = SmartHomeNet::HUB_IP;
  const uint8_t *mask = SmartHomeNet::SUBNET_MASK;
  if (WiFi.config(IPAddress(ip[0], ip[1], ip[2], ip[3]),
                  IPAddress(hub[0], hub[1], hub[2], hub[3]),
                  IPAddress(mask[0], mask[1], mask[2], mask[3]))) {
    WiFi.begin(SmartHomeNet::AP_SSID, SmartHomeNet::AP_PASS);
  } else {
    Serial.println("[WIFI] Failed to configure camera station IP");
  }
}

bool startFallbackApLocked() {
  IPAddress apIp(192, 168, 11, 1);
  IPAddress subnet(255, 255, 255, 0);
  String apSsid = camNetFallbackApSsid();
  // Remove the STA interface so its scans cannot interrupt association or DHCP.
  if (!WiFi.mode(WIFI_AP) || !WiFi.softAPConfig(apIp, apIp, subnet) ||
      !WiFi.softAP(apSsid.c_str(), SmartHomeNet::AP_PASS, SmartHomeNet::WIFI_CHANNEL, 0, 2)) {
    Serial.println("[WIFI] Failed to start fallback AP or configure its IP/DHCP");
    return false;
  }
  WiFi.setSleep(false);
  s_fallbackApActive = true;
  Serial.printf("Fallback setup AP active: %s at http://%s\n",
                apSsid.c_str(), WiFi.softAPIP().toString().c_str());
  return true;
}

bool stopFallbackApLocked() {
  if (!s_fallbackApActive) return true;
  if (!WiFi.mode(WIFI_STA)) return false;
  WiFi.setSleep(false);
  s_fallbackApActive = false;
  Serial.println("Main Hub reachable; fallback setup AP disabled.");
  return true;
}

} // namespace

void camNetInit(SmartHomeNet::CamSource source) {
  s_source = source;
  if (!s_netMutex) {
    s_netMutex = xSemaphoreCreateMutex();
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setAutoReconnect(false);
  s_lastStaAttemptMs = millis();
  startHubConnection();

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
  bool triggerImmediateHeartbeat = false;
  if (st == WL_CONNECTED && (!s_fallbackApActive || WiFi.softAPgetStationNum() == 0) &&
      stopFallbackApLocked()) {
    s_fallbackProbeActive = false;
    syncHubPeerLocked();
    if (!s_wasStaConnected) {
      s_wasStaConnected = true;
      triggerImmediateHeartbeat = true;
    }
  } else {
    if (s_wasStaConnected) {
      s_wasStaConnected = false;
      s_lastStaAttemptMs = now;
      startHubConnection();
    }
    if (s_fallbackApActive) {
      const bool hasClients = WiFi.softAPgetStationNum() > 0;
      if (hasClients) {
        // Start a fresh idle interval once the last client leaves.
        s_lastStaAttemptMs = now;
      }
      if (s_fallbackProbeActive) {
        if (hasClients || now - s_lastStaAttemptMs >= FALLBACK_PROBE_TIMEOUT_MS) {
          s_fallbackProbeActive = !WiFi.mode(WIFI_AP);
        }
      } else if (!hasClients && now - s_lastStaAttemptMs >= FALLBACK_RETRY_INTERVAL_MS) {
        s_lastStaAttemptMs = now;
        s_fallbackProbeActive = WiFi.mode(WIFI_AP_STA);
        if (s_fallbackProbeActive) startHubConnection();
      }
    } else if (now - s_lastStaAttemptMs >= HUB_CONNECT_GRACE_MS &&
               now - s_lastApAttemptMs >= FALLBACK_PROBE_TIMEOUT_MS) {
      s_lastApAttemptMs = now;
      if (startFallbackApLocked()) s_lastStaAttemptMs = now;
    }
  }

  if (s_netMutex) {
    xSemaphoreGive(s_netMutex);
  }

  if (s_wasStaConnected &&
      (triggerImmediateHeartbeat || now - s_lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS)) {
    s_lastHeartbeatMs = now;
    camNetSendEvent(SmartHomeNet::CAM_EVENT_HEARTBEAT, 0, 100, "FACE-CAM");
  }
}

bool camNetPostHttpEvent(SmartHomeNet::CamEventType event,
                         int16_t matchId,
                         uint8_t confidence,
                         const char *label,
                         uint32_t seq) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClient client;
  client.setTimeout(300);
  if (!client.connect("192.168.4.1", 80, 250)) {
    return false;
  }

  String safeLabel = smartHomeJsonEscape(label ? label : "none");
  char jsonBody[160];
  snprintf(jsonBody, sizeof(jsonBody),
           "{\"source\":%d,\"event\":%d,\"matchId\":%d,\"confidence\":%u,\"seq\":%u,\"label\":\"%s\"}",
           static_cast<int>(s_source),
           static_cast<int>(event),
           static_cast<int>(matchId),
           static_cast<unsigned>(confidence),
           static_cast<unsigned>(seq),
           safeLabel.c_str());

  char req[320];
  int reqLen = snprintf(req, sizeof(req),
                        "POST /api/cam/event HTTP/1.1\r\n"
                        "Host: 192.168.4.1\r\n"
                        "Content-Type: application/json\r\n"
                        "Content-Length: %d\r\n"
                        "Connection: close\r\n\r\n%s",
                        static_cast<int>(strlen(jsonBody)), jsonBody);

  size_t written = client.write(reinterpret_cast<const uint8_t *>(req), reqLen);
  unsigned long waitStart = millis();
  while (client.connected() && !client.available() && (millis() - waitStart < 80)) {
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  while (client.available()) {
    client.read();
  }
  client.stop();
  return written == static_cast<size_t>(reqLen);
}

bool camNetSendEvent(SmartHomeNet::CamEventType event,
                     int16_t matchId,
                     uint8_t confidence,
                     const char *label) {
  if (!s_espNowReady || !s_wasStaConnected || WiFi.status() != WL_CONNECTED) return false;

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
  if (memcmp(targetMac, BROADCAST_MAC, sizeof(BROADCAST_MAC)) == 0) {
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

  uint32_t pktSeq = pkt.seq;

  if (s_netMutex) {
    xSemaphoreGive(s_netMutex);
  }

  // Dual-Path: for critical access events (RECOGNIZED or UNKNOWN), or if an ESP-NOW
  // heartbeat failed while connected to Wi-Fi, send via HTTP POST to the Hub.
  bool httpOk = false;
  if (WiFi.status() == WL_CONNECTED &&
      (event != SmartHomeNet::CAM_EVENT_HEARTBEAT || err != ESP_OK || !s_lastSendOk)) {
    httpOk = camNetPostHttpEvent(event, matchId, confidence, label, pktSeq);
  }

  return (err == ESP_OK) || httpOk;
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
  return String("SmartHome-FaceCam");
}

String camNetFallbackApIp() {
  return String("192.168.11.1");
}

String camNetMacString() {
  return WiFi.macAddress();
}
