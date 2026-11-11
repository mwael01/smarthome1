// Smart Home Camera 1 — On-Device Face Detection & Recognition (HW-297 / ESP32-CAM)
// Runs 2-stage ESP-DL face detection (MSR01 + MNP01) and MobileFaceNet S8 recognition
// locally on the ESP32-CAM, persists enrolled faces to the "fr" Flash partition, hosts a
// real-time MJPEG live streaming and management Web UI, and sends wireless ESP-NOW events
// to the Main ESP32 Hub.

#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <fb_gfx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <list>
#include <string>
#include <vector>

#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "face_recognition_tool.hpp"
#include "face_recognition_112_v1_s8.hpp"

#include "cam_hw.h"
#include "cam_net.h"
#include "smart_home_json.h"
#include "smart_home_protocol.h"

namespace {

constexpr unsigned long RECOGNIZED_COOLDOWN_MS = 2500;
constexpr unsigned long UNKNOWN_COOLDOWN_MS = 4000;
constexpr unsigned long ENROLL_TIMEOUT_MS = 12000;
constexpr size_t MAX_PENDING_DELETES = 16;
constexpr uint16_t STREAM_PORT = 81;

struct EnrolledEntry {
  int id;
  char name[24];
};

struct FaceConfig {
  float thresh = 0.55f;
  int unknownConfirm = 2;
  bool armed = true;
  uint8_t flashDuty = 0;
  bool flashAuto = true;
  bool hmirror = false;
  bool vflip = false;
};

struct FaceStatusSnapshot {
  bool cameraReady = false;
  bool modelReady = false;
  bool flashOk = false;
  bool faceDetected = false;
  char faceQuality[16] = "NO_FACE"; // "READY", "TOO_FAR", "TOO_CLOSE", "OFF_CENTER", "NO_FACE"
  int faceBox[4] = {0, 0, 0, 0};    // x1, y1, x2, y2
  bool enrollPending = false;
  uint32_t enrollRemainingMs = 0;
  char enrollTargetName[24] = "";
  char enrollMessage[64] = "Idle";
  char lastResult[24] = "NONE";
  char lastMatchName[24] = "---";
  int lastMatchId = -1;
  float lastSimilarity = 0.0f;
  uint32_t inferenceMs = 0;
  float fps = 0.0f;
  uint32_t frameCount = 0;
  int enrolledCount = 0;
  float sceneLuma = 0.0f;
  uint8_t flashDutyApplied = 0;
  std::vector<EnrolledEntry> enrolled;
  FaceConfig cfg;
};

Preferences prefs;
WebServer server(80);

SemaphoreHandle_t stateMutex = nullptr;

FaceStatusSnapshot g_status;
FaceConfig g_pendingCfg;
bool g_applyCfgPending = false;
bool g_enrollPending = false;
uint32_t g_enrollSeq = 0;
unsigned long g_enrollStartMs = 0;
char g_enrollName[24] = "User";
std::vector<int> g_pendingDeleteIds;
bool g_clearPending = false;

uint16_t packRgb565Color(uint32_t bgr888) {
  return static_cast<uint16_t>(((bgr888 >> 16) & 0x001F) |
                               ((bgr888 >> 3) & 0x07E0) |
                               ((bgr888 << 8) & 0xF800));
}

void drawFaceOverlay(camera_fb_t *fb,
                     const std::list<dl::detect::result_t> &results,
                     int matchId,
                     bool enrollActive) {
  fb_data_t gfx = {};
  gfx.width = fb->width;
  gfx.height = fb->height;
  gfx.bytes_per_pixel = 2;
  gfx.format = FB_RGB565;
  gfx.data = fb->buf;

  uint32_t bgr = 0x0000FFFF; // Yellow = detected / calibrating
  if (enrollActive) {
    bgr = 0x00FFFF00; // Cyan = enrolling
  } else if (matchId >= 0) {
    bgr = 0x0000FF00; // Green = recognized
  } else if (matchId == -2) {
    bgr = 0x000000FF; // Red = unknown
  }
  uint16_t color = packRgb565Color(bgr);

  for (const auto &res : results) {
    if (res.box.size() < 4) continue;
    int x = constrain(res.box[0], 0, fb->width - 1);
    int y = constrain(res.box[1], 0, fb->height - 1);
    int x2 = constrain(res.box[2], 0, fb->width - 1);
    int y2 = constrain(res.box[3], 0, fb->height - 1);
    int w = max(1, x2 - x + 1);
    int h = max(1, y2 - y + 1);
    if (x + w > fb->width) w = fb->width - x;
    if (y + h > fb->height) h = fb->height - y;

    // Outer 2px rectangle for clear visibility
    fb_gfx_drawFastHLine(&gfx, x, y, w, color);
    fb_gfx_drawFastHLine(&gfx, x, y + 1, w, color);
    fb_gfx_drawFastHLine(&gfx, x, y + h - 1, w, color);
    fb_gfx_drawFastHLine(&gfx, x, y + h - 2, w, color);
    fb_gfx_drawFastVLine(&gfx, x, y, h, color);
    fb_gfx_drawFastVLine(&gfx, x + 1, y, h, color);
    fb_gfx_drawFastVLine(&gfx, x + w - 1, y, h, color);
    fb_gfx_drawFastVLine(&gfx, x + w - 2, y, h, color);

    // 5-point facial landmarks (eyes, nose, mouth corners)
    for (size_t j = 0; j + 1 < res.keypoint.size(); j += 2) {
      int kx = constrain(res.keypoint[j], 0, fb->width - 3);
      int ky = constrain(res.keypoint[j + 1], 0, fb->height - 3);
      fb_gfx_fillRect(&gfx, kx, ky, 3, 3, color);
    }
  }
}

void loadConfigFromNvs(FaceConfig &cfg) {
  if (!prefs.begin("facecfg", true)) return;
  cfg.thresh = prefs.getFloat("thresh", 0.55f);
  cfg.unknownConfirm = prefs.getInt("uconf", 2);
  cfg.armed = prefs.getBool("armed", true);
  cfg.flashDuty = prefs.getUChar("flash", 0);
  cfg.flashAuto = prefs.getBool("flash_auto", true);
  cfg.hmirror = prefs.getBool("hmir", false);
  cfg.vflip = prefs.getBool("vflip", false);
  prefs.end();

  cfg.thresh = constrain(cfg.thresh, 0.30f, 0.85f);
  cfg.unknownConfirm = constrain(cfg.unknownConfirm, 1, 5);
}

void saveConfigToNvs(const FaceConfig &cfg) {
  if (!prefs.begin("facecfg", false)) return;
  prefs.putFloat("thresh", cfg.thresh);
  prefs.putInt("uconf", cfg.unknownConfirm);
  prefs.putBool("armed", cfg.armed);
  prefs.putUChar("flash", cfg.flashDuty);
  prefs.putBool("flash_auto", cfg.flashAuto);
  prefs.putBool("hmir", cfg.hmirror);
  prefs.putBool("vflip", cfg.vflip);
  prefs.end();
}

void syncEnrolledSnapshot(FaceRecognition112V1S8 &recognizer, FaceStatusSnapshot &snap) {
  std::vector<face_info_t> ids = recognizer.get_enrolled_ids();
  snap.enrolledCount = static_cast<int>(ids.size());
  snap.enrolled.clear();
  snap.enrolled.reserve(ids.size());
  for (const auto &item : ids) {
    EnrolledEntry e = {};
    e.id = item.id;
    if (item.name.empty()) {
      snprintf(e.name, sizeof(e.name), "Face-%d", item.id);
    } else {
      strncpy(e.name, item.name.c_str(), sizeof(e.name) - 1);
      e.name[sizeof(e.name) - 1] = '\0';
    }
    snap.enrolled.push_back(e);
  }
}

void sanitizeName(const String &raw, char *out, size_t outSize) {
  if (outSize == 0) return;
  size_t pos = 0;
  for (size_t i = 0; i < raw.length() && pos + 1 < outSize; ++i) {
    char c = raw[i];
    if (isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == ' ' || c == '(' || c == ')') {
      out[pos++] = c;
    }
  }
  while (pos > 0 && out[pos - 1] == ' ') pos--;
  out[pos] = '\0';
  if (pos == 0) {
    strncpy(out, "Person", outSize - 1);
    out[outSize - 1] = '\0';
  }
}

void faceWorkerTask(void * /*param*/) {
  FaceConfig activeCfg;
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    activeCfg = g_status.cfg;
    xSemaphoreGive(stateMutex);
  }
  uint8_t currentDuty = activeCfg.flashDuty;

  HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
  HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
  FaceRecognition112V1S8 recognizer;

  bool flashOk = false;
  if (recognizer.set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr") == 1) {
    int partStatus = recognizer.check_partition();
    if (partStatus >= 0) {
      recognizer.set_ids_from_flash();
    } else {
      Serial.printf("Initializing blank 'fr' flash partition (status=%d)\n", partStatus);
      recognizer.clear_id(true);
    }
    flashOk = (recognizer.check_partition() >= 0);
  } else {
    Serial.println("WARNING: 'fr' flash partition not found; faces will not persist.");
  }

  recognizer.set_thresh(activeCfg.thresh);

  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    g_status.modelReady = true;
    g_status.flashOk = flashOk;
    syncEnrolledSnapshot(recognizer, g_status);
    xSemaphoreGive(stateMutex);
  }

  Serial.printf("Face recognition ready: flashOk=%s enrolled=%d thresh=%.2f\n",
                flashOk ? "YES" : "NO",
                recognizer.get_enrolled_id_num(),
                recognizer.get_thresh());

  int unknownStreak = 0;
  unsigned long lastRecognizedSentMs = 0;
  unsigned long lastUnknownSentMs = 0;

  uint32_t fpsFrames = 0;
  unsigned long fpsLastMs = millis();
  float currentFps = 0.0f;

  // Auto-flash hysteresis state.
  bool flashOn = false;
  // Consecutive empty frame grabs; triggers camera re-init when high.
  int fbNullStreak = 0;

  while (true) {
    std::vector<int> deleteIds;
    bool doClear = false;
    bool doApplyCfg = false;
    bool enrollActive = false;
    uint32_t enrollSeq = 0;
    char enrollTarget[24] = "";
    uint32_t remainingMs = 0;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (g_clearPending) {
        doClear = true;
        g_clearPending = false;
        g_pendingDeleteIds.clear();
      }
      if (!g_pendingDeleteIds.empty()) {
        deleteIds.swap(g_pendingDeleteIds);
      }
      if (g_applyCfgPending) {
        activeCfg = g_pendingCfg;
        g_status.cfg = activeCfg;
        g_applyCfgPending = false;
        doApplyCfg = true;
      }
      if (g_enrollPending) {
        unsigned long elapsed = millis() - g_enrollStartMs;
        if (elapsed > ENROLL_TIMEOUT_MS) {
          g_enrollPending = false;
          g_status.enrollPending = false;
          g_status.enrollRemainingMs = 0;
          snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                   "Enroll timed out: no face detected");
        } else {
          enrollActive = true;
          enrollSeq = g_enrollSeq;
          remainingMs = ENROLL_TIMEOUT_MS - elapsed;
          g_status.enrollRemainingMs = remainingMs;
          snprintf(enrollTarget, sizeof(enrollTarget), "%s", g_enrollName);
          snprintf(g_status.enrollTargetName, sizeof(g_status.enrollTargetName), "%s", g_enrollName);
        }
      } else {
        g_status.enrollRemainingMs = 0;
      }
      xSemaphoreGive(stateMutex);
    }

    if (doClear) {
      recognizer.clear_id(true);
      unknownStreak = 0;
      Serial.println("Cleared all enrolled faces from Flash.");
      if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        syncEnrolledSnapshot(recognizer, g_status);
        snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage), "Cleared all enrolled faces");
        xSemaphoreGive(stateMutex);
      }
    }

    if (!deleteIds.empty()) {
      for (int id : deleteIds) {
        recognizer.delete_id(id, true);
        Serial.printf("Deleted face ID #%d from Flash.\n", id);
      }
      unknownStreak = 0;
      if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        syncEnrolledSnapshot(recognizer, g_status);
        if (deleteIds.size() == 1) {
          snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                   "Deleted face ID #%d", deleteIds[0]);
        } else {
          snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                   "Deleted %u faces", static_cast<unsigned>(deleteIds.size()));
        }
        xSemaphoreGive(stateMutex);
      }
    }

    if (doApplyCfg) {
      recognizer.set_thresh(activeCfg.thresh);
      camHwApplySensorSettings(activeCfg.hmirror, activeCfg.vflip, activeCfg.flashDuty);
      saveConfigToNvs(activeCfg);
      Serial.printf("Updated settings: thresh=%.2f armed=%d uconf=%d flash=%u\n",
                    activeCfg.thresh, activeCfg.armed, activeCfg.unknownConfirm, activeCfg.flashDuty);
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      fbNullStreak++;
      if (fbNullStreak % 50 == 0) {
        Serial.printf("[FACE] Warning: esp_camera_fb_get returned null (streak %d)\n", fbNullStreak);
      }
      vTaskDelay(pdMS_TO_TICKS(30));
      continue;
    }
    fbNullStreak = 0;
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      g_status.cameraReady = true;
      xSemaphoreGive(stateMutex);
    }

    unsigned long tStart = millis();
    float sceneLuma = camHwEstimateMeanLuma(fb);

    // Auto-flash with hysteresis: turn LED on in low light, off when bright.
    uint8_t duty = activeCfg.flashDuty;
    if (activeCfg.flashAuto) {
      if (!flashOn && sceneLuma < 0.18f) {
        flashOn = true;
        duty = activeCfg.flashDuty;
      } else if (flashOn && sceneLuma > 0.40f) {
        flashOn = false;
        duty = 0;
      } else {
        duty = flashOn ? activeCfg.flashDuty : 0;
      }
      if (duty != currentDuty) {
        camHwSetFlashLed(duty);
        currentDuty = duty;
      }
    } else {
      flashOn = (activeCfg.flashDuty > 0);
      if (activeCfg.flashDuty != currentDuty) {
        camHwSetFlashLed(activeCfg.flashDuty);
        currentDuty = activeCfg.flashDuty;
      }
    }

    std::vector<int> imgShape = {static_cast<int>(fb->height), static_cast<int>(fb->width), 3};
    std::list<dl::detect::result_t> &candidates =
        s1.infer(reinterpret_cast<uint16_t *>(fb->buf), imgShape);
    std::list<dl::detect::result_t> &results =
        s2.infer(reinterpret_cast<uint16_t *>(fb->buf), imgShape, candidates);

    // Pick largest face with 5-point facial landmarks
    const dl::detect::result_t *bestFace = nullptr;
    long bestArea = -1;
    for (const auto &res : results) {
      if (res.box.size() < 4 || res.keypoint.size() < 10) continue;
      long area = static_cast<long>(res.box[2] - res.box[0]) * static_cast<long>(res.box[3] - res.box[1]);
      if (area > bestArea) {
        bestArea = area;
        bestFace = &res;
      }
    }

    int overlayState = 0;
    bool detected = (bestFace != nullptr);
    int matchedId = -1;
    float similarity = 0.0f;
    char matchedName[24] = "---";
    char resultLabel[24] = "NO FACE";
    char qualityLabel[16] = "NO_FACE";
    int faceBox[4] = {0, 0, 0, 0};
    bool enrolledNow = false;
    int enrolledNewId = -1;
    bool enrollAttempted = false;

    if (bestFace) {
      faceBox[0] = constrain(bestFace->box[0], 0, fb->width - 1);
      faceBox[1] = constrain(bestFace->box[1], 0, fb->height - 1);
      faceBox[2] = constrain(bestFace->box[2], 0, fb->width - 1);
      faceBox[3] = constrain(bestFace->box[3], 0, fb->height - 1);
      int faceCenterX = (faceBox[0] + faceBox[2]) / 2;
      int faceCenterY = (faceBox[1] + faceBox[3]) / 2;

      int dx = faceCenterX - static_cast<int>(fb->width / 2);
      int dy = faceCenterY - static_cast<int>(fb->height / 2);
      if (dx < 0) dx = -dx;
      if (dy < 0) dy = -dy;

      if (bestArea < 2400) {
        strncpy(qualityLabel, "TOO_FAR", sizeof(qualityLabel) - 1);
      } else if (bestArea > 33000) {
        strncpy(qualityLabel, "TOO_CLOSE", sizeof(qualityLabel) - 1);
      } else if (dx > 48 || dy > 48) {
        strncpy(qualityLabel, "OFF_CENTER", sizeof(qualityLabel) - 1);
      } else {
        strncpy(qualityLabel, "READY", sizeof(qualityLabel) - 1);
      }

      std::vector<int> landmarks = bestFace->keypoint;
      if (enrollActive) {
        // Enroll face with valid landmarks
        enrollAttempted = true;
        enrolledNewId = recognizer.enroll_id(reinterpret_cast<uint16_t *>(fb->buf),
                                             imgShape,
                                             landmarks,
                                             std::string(enrollTarget),
                                             true);
        enrolledNow = (enrolledNewId >= 0);
        overlayState = enrolledNewId;
        snprintf(resultLabel, sizeof(resultLabel), enrolledNow ? "ENROLLED" : "ENROLL ERR");
        if (enrolledNow) {
          matchedId = enrolledNewId;
          strncpy(matchedName, enrollTarget, sizeof(matchedName) - 1);
          similarity = 1.0f;
        }
        unknownStreak = 0;
      } else {
        face_info_t info = recognizer.recognize(reinterpret_cast<uint16_t *>(fb->buf),
                                                imgShape,
                                                landmarks);
        matchedId = info.id;
        similarity = info.similarity;

        unsigned long now = millis();
        uint8_t confPct = static_cast<uint8_t>(constrain(lroundf(similarity * 100.0f), 0L, 100L));

        if (info.id >= 0) {
          unknownStreak = 0;
          overlayState = info.id;
          snprintf(resultLabel, sizeof(resultLabel), "RECOGNIZED");
          if (info.name.empty()) {
            snprintf(matchedName, sizeof(matchedName), "ID-%d", info.id);
          } else {
            strncpy(matchedName, info.name.c_str(), sizeof(matchedName) - 1);
            matchedName[sizeof(matchedName) - 1] = '\0';
          }
          camHwSetStatusLed(true);

          if (now - lastRecognizedSentMs >= RECOGNIZED_COOLDOWN_MS) {
            lastRecognizedSentMs = now;
            camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED,
                            static_cast<int16_t>(info.id),
                            confPct,
                            matchedName);
            Serial.printf("[FACE] RECOGNIZED id=%d name=\"%s\" sim=%.2f\n",
                          info.id, matchedName, similarity);
          }
        } else {
          overlayState = -2;
          snprintf(resultLabel, sizeof(resultLabel), "UNKNOWN");
          snprintf(matchedName, sizeof(matchedName), "Unknown");
          unknownStreak++;

          if (activeCfg.armed &&
              unknownStreak >= activeCfg.unknownConfirm &&
              (now - lastUnknownSentMs >= UNKNOWN_COOLDOWN_MS)) {
            lastUnknownSentMs = now;
            camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN, -1, confPct, "UNKNOWN");
            Serial.printf("[FACE] UNKNOWN INTRUDER ALERT (sim=%.2f streak=%d)\n",
                          similarity, unknownStreak);
          }
        }
      }
    } else {
      unknownStreak = 0;
      camHwSetStatusLed(false);
    }

    unsigned long elapsedMs = millis() - tStart;

    if (!results.empty()) {
      drawFaceOverlay(fb, results, overlayState, enrollActive);
    }
    camHwUpdatePreviewJpeg(fb, PIXFORMAT_RGB565, 68);
    esp_camera_fb_return(fb);

    // Real-time inference FPS tracking
    fpsFrames++;
    unsigned long nowMs = millis();
    if (nowMs - fpsLastMs >= 1000) {
      currentFps = (fpsFrames * 1000.0f) / (nowMs - fpsLastMs);
      fpsFrames = 0;
      fpsLastMs = nowMs;
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_status.faceDetected = detected;
      memcpy(g_status.faceBox, faceBox, sizeof(faceBox));
      strncpy(g_status.faceQuality, qualityLabel, sizeof(g_status.faceQuality) - 1);
      g_status.lastMatchId = matchedId;
      g_status.lastSimilarity = similarity;
      g_status.inferenceMs = static_cast<uint32_t>(elapsedMs);
      g_status.fps = currentFps;
      g_status.frameCount++;
      g_status.sceneLuma = sceneLuma;
      g_status.flashDutyApplied = currentDuty;
      snprintf(g_status.lastResult, sizeof(g_status.lastResult), "%s", resultLabel);
      snprintf(g_status.lastMatchName, sizeof(g_status.lastMatchName), "%s", matchedName);

      if (enrollActive && enrollAttempted && g_enrollSeq == enrollSeq) {
        g_enrollPending = false;
        g_status.enrollPending = false;
        g_status.enrollRemainingMs = 0;
        if (enrolledNow) {
          snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                   "Saved \"%s\" as ID #%d in Flash", enrollTarget, enrolledNewId);
          syncEnrolledSnapshot(recognizer, g_status);
          Serial.println(g_status.enrollMessage);
        } else {
          snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                   "Enroll failed (Flash full or bad alignment)");
        }
      } else if (enrollActive && enrolledNow) {
        syncEnrolledSnapshot(recognizer, g_status);
      }
      xSemaphoreGive(stateMutex);
    }

    // Adaptive cooperative yield: gives loopTask (HTTP server, preview snapshots,
    // and ESP-NOW/heartbeats) sufficient CPU slices on Core 1 without stalling inference.
    vTaskDelay(pdMS_TO_TICKS(detected || bestFace ? 10 : 30));
  }
}

void handleRoot() {
  static const char HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Smart Camera 1 — Face Recognition & Live Stream</title>
<style>
  :root {
    --bg: #090d16;
    --card: #131c2e;
    --card-border: #23324d;
    --card-hover: #19263e;
    --text: #f1f5f9;
    --text-muted: #94a3b8;
    --primary: #3b82f6;
    --primary-hover: #2563eb;
    --success: #10b981;
    --danger: #ef4444;
    --warning: #f59e0b;
    --cyan: #06b6d4;
    --radius: 12px;
  }
  * { box-sizing: border-box; }
  body {
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg);
    color: var(--text);
    margin: 0;
    padding: 16px;
    line-height: 1.45;
  }
  .wrap { max-width: 1040px; margin: 0 auto; }
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 10px;
    margin-bottom: 16px;
    padding-bottom: 12px;
    border-bottom: 1px solid var(--card-border);
  }
  h1 { font-size: 1.25rem; font-weight: 700; margin: 0; display: flex; align-items: center; gap: 8px; }
  .pulse-dot {
    width: 10px; height: 10px; border-radius: 50%; background: var(--success);
    box-shadow: 0 0 10px var(--success); animation: pulse 1.8s infinite;
  }
  @keyframes pulse { 0%, 100% { opacity: 1; transform: scale(1); } 50% { opacity: .4; transform: scale(.85); } }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 16px; }
  .card {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 16px;
    box-shadow: 0 4px 14px rgba(0,0,0,.35);
  }
  .card-title {
    font-size: .95rem; font-weight: 700; color: #cbd5e1;
    margin: 0 0 12px; display: flex; justify-content: space-between; align-items: center;
  }
  
  /* Video Player Container */
  .player-box {
    position: relative;
    width: 100%;
    max-width: 420px;
    margin: 0 auto 12px;
    border-radius: var(--radius);
    overflow: hidden;
    background: #020617;
    border: 1px solid var(--card-border);
    aspect-ratio: 1 / 1;
    box-shadow: 0 8px 24px rgba(0,0,0,.5);
  }
  .player-box img {
    width: 100%;
    height: 100%;
    object-fit: cover;
    display: block;
    background: #000;
  }
  
  /* Reticle / Face Guide HUD Overlay */
  .reticle {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    width: 58%;
    height: 72%;
    border-radius: 50% / 60%;
    border: 2px dashed rgba(148, 163, 184, 0.35);
    pointer-events: none;
    transition: all .25s ease;
  }
  .reticle.enrolling {
    border: 3px solid var(--cyan);
    box-shadow: 0 0 18px rgba(6, 182, 212, 0.45);
    animation: reticlePulse 1.2s infinite ease-in-out;
  }
  .reticle.recognized {
    border: 3px solid var(--success);
    box-shadow: 0 0 18px rgba(16, 185, 129, 0.45);
  }
  .reticle.unknown {
    border: 3px solid var(--danger);
    box-shadow: 0 0 18px rgba(239, 68, 68, 0.45);
  }
  @keyframes reticlePulse {
    0%, 100% { transform: translate(-50%, -50%) scale(1); opacity: 1; }
    50% { transform: translate(-50%, -50%) scale(1.04); opacity: .7; }
  }

  /* HUD Badges */
  .hud-top {
    position: absolute;
    top: 10px;
    left: 10px;
    right: 10px;
    display: flex;
    justify-content: space-between;
    pointer-events: none;
  }
  .hud-pill {
    background: rgba(15, 23, 42, 0.85);
    backdrop-filter: blur(6px);
    border: 1px solid rgba(255, 255, 255, 0.12);
    border-radius: 999px;
    padding: 3px 9px;
    font-size: .75rem;
    font-weight: 700;
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }
  .hud-bottom {
    position: absolute;
    bottom: 0;
    left: 0;
    right: 0;
    background: linear-gradient(to top, rgba(2, 6, 23, 0.95), rgba(2, 6, 23, 0));
    padding: 24px 12px 10px;
    display: flex;
    justify-content: space-between;
    align-items: flex-end;
    pointer-events: none;
  }
  .hud-banner-title { font-size: .92rem; font-weight: 700; }
  .hud-banner-sub { font-size: .75rem; color: var(--text-muted); }

  /* Buttons & Inputs */
  button {
    background: var(--primary);
    color: #fff;
    border: 0;
    border-radius: 8px;
    padding: 8px 14px;
    font-size: .88rem;
    font-weight: 600;
    cursor: pointer;
    transition: all .15s ease;
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }
  button:hover { background: var(--primary-hover); transform: translateY(-1px); }
  button:active { transform: translateY(0); }
  button.danger { background: var(--danger); }
  button.danger:hover { background: #dc2626; }
  button.success { background: var(--success); }
  button.success:hover { background: #059669; }
  button.outline {
    background: transparent;
    border: 1px solid var(--card-border);
    color: var(--text-muted);
  }
  button.outline:hover { background: var(--card-hover); color: var(--text); }
  button:disabled { opacity: .5; cursor: not-allowed; transform: none; }
  
  input[type="text"], input[type="number"], select {
    background: #0b1322;
    color: var(--text);
    border: 1px solid var(--card-border);
    border-radius: 8px;
    padding: 8px 12px;
    font-size: .9rem;
    width: 100%;
    outline: none;
  }
  input[type="text"]:focus, input[type="number"]:focus {
    border-color: var(--primary);
    box-shadow: 0 0 0 2px rgba(59, 130, 246, 0.25);
  }
  .row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 7px 0;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
    font-size: .88rem;
  }
  .row:last-child { border-bottom: 0; }
  
  /* Status Badges */
  .badge {
    padding: 2px 8px;
    border-radius: 999px;
    font-size: .75rem;
    font-weight: 700;
    letter-spacing: .3px;
  }
  .badge.ok { background: rgba(16, 185, 129, 0.2); color: #6ee7b7; border: 1px solid rgba(16, 185, 129, 0.35); }
  .badge.bad { background: rgba(239, 68, 68, 0.2); color: #fca5a5; border: 1px solid rgba(239, 68, 68, 0.35); }
  .badge.warn { background: rgba(245, 158, 11, 0.2); color: #fde68a; border: 1px solid rgba(245, 158, 11, 0.35); }
  .badge.info { background: rgba(6, 182, 212, 0.2); color: #a5f3fc; border: 1px solid rgba(6, 182, 212, 0.35); }
  .badge.off { background: rgba(148, 163, 184, 0.15); color: #94a3b8; }

  /* Mode selector tabs */
  .tabs {
    display: flex;
    background: #0b1322;
    border-radius: 8px;
    padding: 3px;
    gap: 4px;
    margin-bottom: 12px;
    border: 1px solid var(--card-border);
  }
  .tab-btn {
    flex: 1;
    background: transparent;
    color: var(--text-muted);
    border-radius: 6px;
    padding: 6px 10px;
    font-size: .8rem;
    text-align: center;
    cursor: pointer;
    border: 0;
  }
  .tab-btn.active { background: var(--card); color: #fff; font-weight: 700; box-shadow: 0 2px 6px rgba(0,0,0,.3); }

  /* Progress bar for enrollment */
  .progress-wrap {
    height: 8px;
    background: #0b1322;
    border-radius: 999px;
    overflow: hidden;
    margin: 8px 0;
    display: none;
    border: 1px solid var(--card-border);
  }
  .progress-bar {
    height: 100%;
    width: 0%;
    background: linear-gradient(90deg, var(--cyan), var(--primary));
    transition: width .2s linear;
  }

  /* Table */
  table { width: 100%; border-collapse: collapse; margin-top: 8px; font-size: .88rem; }
  th, td { padding: 8px; text-align: left; border-bottom: 1px solid var(--card-border); }
  th { color: var(--text-muted); font-weight: 600; font-size: .75rem; text-transform: uppercase; }
  
  /* Toggle Switch */
  .switch { position: relative; display: inline-block; width: 42px; height: 22px; }
  .switch input { opacity: 0; width: 0; height: 0; }
  .slider {
    position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0;
    background-color: #334155; transition: .2s; border-radius: 22px;
  }
  .slider:before {
    position: absolute; content: ""; height: 16px; width: 16px; left: 3px; bottom: 3px;
    background-color: white; transition: .2s; border-radius: 50%;
  }
  input:checked + .slider { background-color: var(--primary); }
  input:checked + .slider:before { transform: translateX(20px); }

  /* Toast Notification */
  #toast {
    position: fixed;
    bottom: 20px;
    right: 20px;
    background: #1e293b;
    border: 1px solid var(--card-border);
    color: #fff;
    padding: 10px 18px;
    border-radius: 8px;
    box-shadow: 0 6px 18px rgba(0,0,0,.5);
    transform: translateY(100px);
    opacity: 0;
    transition: all .25s ease;
    z-index: 999;
    font-size: .88rem;
  }
  #toast.show { transform: translateY(0); opacity: 1; }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1><span class="pulse-dot"></span> Camera 1 — Face Recognition Live</h1>
    <div style="display:flex;gap:8px;align-items:center">
      <span id="hub-badge" class="badge off">CHECKING LINK</span>
      <span id="fps-badge" class="badge info">0.0 FPS</span>
    </div>
  </header>

  <div class="grid">
    <!-- LEFT CARD: LIVE CAMERA & AI OVERLAY -->
    <div class="card">
      <div class="card-title">
        <span>Live Camera View</span>
        <div style="display:flex;gap:6px">
          <button class="outline" style="padding:4px 8px;font-size:.78rem" onclick="toggleStreamMode()" id="stream-mode-btn">MJPEG Live</button>
        </div>
      </div>

      <div class="player-box" id="player-container">
        <img id="cam" alt="Camera Live Stream" src="/preview.jpg">
        <div id="reticle" class="reticle"></div>

        <div class="hud-top">
          <div class="hud-pill" id="hud-stream-pill">
            <span style="display:inline-block;width:7px;height:7px;border-radius:50%;background:var(--success)"></span>
            <span id="hud-stream-text">STREAMING :81</span>
          </div>
          <div class="hud-pill" id="hud-quality-pill">NO FACE</div>
        </div>

        <div class="hud-bottom">
          <div>
            <div class="hud-banner-title" id="hud-name">---</div>
            <div class="hud-banner-sub" id="hud-guide">Look towards camera</div>
          </div>
          <div style="text-align:right">
            <div id="hud-match-score" style="font-weight:700;font-size:1.1rem;color:#fff">0%</div>
            <div style="font-size:.72rem;color:var(--text-muted)" id="hud-ms">0 ms</div>
          </div>
        </div>
      </div>

      <!-- Quick LED & Test Controls -->
      <div class="row">
        <span>Flash LED Brightness</span>
        <div style="display:flex;align-items:center;gap:8px;width:55%">
          <input id="flash-slider" type="range" min="0" max="180" value="0" style="flex:1" oninput="updateFlashPreview(this.value)">
          <span id="flash-val" style="font-size:.8rem;width:28px;text-align:right">0</span>
        </div>
      </div>

      <div class="row">
        <span>Auto Flash in Low Light</span>
        <label class="switch"><input id="cfg-flash-auto" type="checkbox"><span class="slider"></span></label>
      </div>

      <div class="row">
        <span>Wireless Link / Storage</span>
        <span id="status-storage-info" style="font-size:.82rem;color:var(--text-muted)">Flash: OK | ESP-NOW Active</span>
      </div>

      <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px">
        <button class="success" onclick="testDoor(1)" style="justify-content:center">Test Door Open</button>
        <button class="danger" onclick="testDoor(0)" style="justify-content:center">Test Intruder Siren</button>
      </div>
    </div>

    <!-- RIGHT CARD: USER ADDITION & SAVED FACES -->
    <div class="card">
      <div class="card-title">
        <span>Authorized Face Enrollment</span>
        <span id="face-count-badge" class="badge off">0 / 16</span>
      </div>

      <!-- Enrollment Flow Tabs -->
      <div class="tabs">
        <button class="tab-btn active" id="tab-single" onclick="setEnrollMode('single')">Single Capture</button>
        <button class="tab-btn" id="tab-multi" onclick="setEnrollMode('multi')">Multi-Angle Guided (3x)</button>
      </div>

      <div style="margin-bottom:12px">
        <label style="font-size:.82rem;color:var(--text-muted);display:block;margin-bottom:4px">Person Name</label>
        <div style="display:flex;gap:8px">
          <input id="ename" type="text" maxlength="20" placeholder="e.g. Alice, David" value="User">
          <button id="enroll-btn" class="primary" onclick="startEnrollment()" style="white-space:nowrap">Enroll Face</button>
          <button id="cancel-btn" class="danger" onclick="cancelEnrollment()" style="display:none;white-space:nowrap">Cancel</button>
        </div>
      </div>

      <!-- Enrollment Progress Bar -->
      <div class="progress-wrap" id="prog-wrap">
        <div class="progress-bar" id="prog-bar"></div>
      </div>

      <div id="enroll-status-banner" style="background:#0b1322;border:1px solid var(--card-border);border-radius:8px;padding:9px 12px;font-size:.84rem;color:#93c5fd;margin-bottom:12px;display:flex;justify-content:space-between;align-items:center">
        <span id="enroll-msg">Idle: Enter name and click Enroll</span>
        <span id="enroll-timer" style="font-weight:700"></span>
      </div>

      <!-- Saved Faces Section -->
      <div style="display:flex;justify-content:space-between;align-items:center;margin-top:14px">
        <span style="font-weight:700;font-size:.9rem">Enrolled Faces in Flash</span>
        <button class="outline danger" style="padding:4px 8px;font-size:.78rem" onclick="clearAllFaces()">Clear All</button>
      </div>

      <div style="max-height:220px;overflow-y:auto;margin-top:6px">
        <table>
          <thead><tr><th>ID</th><th>Name</th><th style="text-align:right">Action</th></tr></thead>
          <tbody id="face-table-body">
            <tr><td colspan="3" style="text-align:center;color:var(--text-muted)">Loading enrolled faces...</td></tr>
          </tbody>
        </table>
      </div>

      <hr style="border:0;border-top:1px solid var(--card-border);margin:16px 0 12px">

      <!-- Settings Collapsible -->
      <div class="card-title" style="margin-bottom:8px">
        <span>Security & Camera Settings</span>
      </div>

      <div class="row">
        <span>Intruder Siren Armed</span>
        <label class="switch"><input id="cfg-armed" type="checkbox"><span class="slider"></span></label>
      </div>

      <div class="row">
        <span>Similarity Threshold (<span id="thresh-val">0.55</span>)</span>
        <div style="display:flex;align-items:center;gap:6px;width:55%">
          <input id="cfg-thresh" type="range" min="0.30" max="0.80" step="0.05" value="0.55" style="flex:1" oninput="document.getElementById('thresh-val').textContent=parseFloat(this.value).toFixed(2)">
        </div>
      </div>

      <div class="row">
        <span>Preset Profiles</span>
        <div style="display:flex;gap:4px">
          <button class="outline" style="padding:3px 7px;font-size:.75rem" onclick="setPresetThresh(0.65)">Strict</button>
          <button class="outline" style="padding:3px 7px;font-size:.75rem" onclick="setPresetThresh(0.55)">Balanced</button>
          <button class="outline" style="padding:3px 7px;font-size:.75rem" onclick="setPresetThresh(0.45)">Relaxed</button>
        </div>
      </div>

      <div class="row">
        <span>Confirm Frames (Unknown)</span>
        <input id="cfg-uconf" type="number" min="1" max="5" value="2" style="width:65px">
      </div>

      <div class="row">
        <span>Orientation</span>
        <span>
          <label style="margin-right:12px"><input id="cfg-hmir" type="checkbox"> Mirror H</label>
          <label><input id="cfg-vflip" type="checkbox"> Flip V</label>
        </span>
      </div>

      <div style="margin-top:10px">
        <button onclick="saveSettings()" style="width:100%;justify-content:center">Save Settings to NVS</button>
      </div>
    </div>
  </div>
</div>

<div id="toast">Settings Saved</div>

<script>
let currentMode = 'mjpeg'; // 'mjpeg' or 'snapshot'
let mjpegFails = 0;
let snapshotActive = false;
let enrollWizardMode = 'single'; // 'single' or 'multi'
let multiStep = 0; // 0=front, 1=left, 2=right
let multiBaseName = '';
let cfgLoaded = false;
let clientFrames = 0;
let lastFpsTs = performance.now();
let lastListSig = '';

function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2600);
}

// Live Stream Handling
function initCamera() {
  const img = document.getElementById('cam');
  const streamUrl = window.location.protocol + '//' + window.location.hostname + ':81/stream';

  img.onload = () => {
    clientFrames++;
    const now = performance.now();
    if (now - lastFpsTs >= 1000) {
      const fps = Math.round((clientFrames * 1000) / (now - lastFpsTs) * 10) / 10;
      document.getElementById('fps-badge').textContent = fps + ' FPS';
      clientFrames = 0;
      lastFpsTs = now;
    }
  };

  img.onerror = () => {
    if (currentMode === 'mjpeg') {
      mjpegFails++;
      if (mjpegFails >= 2) {
        console.warn('Switching to fast snapshot polling fallback');
        setStreamMode('snapshot');
      }
    }
  };

  setStreamMode('mjpeg');
}

function setStreamMode(mode) {
  currentMode = mode;
  const img = document.getElementById('cam');
  const btn = document.getElementById('stream-mode-btn');
  const pill = document.getElementById('hud-stream-text');

  if (mode === 'mjpeg') {
    snapshotActive = false;
    btn.textContent = 'Mode: MJPEG';
    pill.textContent = 'STREAMING :81';
    img.src = window.location.protocol + '//' + window.location.hostname + ':81/stream?t=' + Date.now();
  } else {
    btn.textContent = 'Mode: Snapshots';
    pill.textContent = 'FAST SNAPSHOT';
    snapshotActive = true;
    pollNextSnapshot();
  }
}

function toggleStreamMode() {
  setStreamMode(currentMode === 'mjpeg' ? 'snapshot' : 'mjpeg');
}

function pollNextSnapshot() {
  if (!snapshotActive || currentMode !== 'snapshot') return;
  const img = document.getElementById('cam');
  const next = new Image();
  next.onload = () => {
    if (snapshotActive) {
      img.src = next.src;
      requestAnimationFrame(pollNextSnapshot);
    }
  };
  next.onerror = () => {
    if (snapshotActive) setTimeout(pollNextSnapshot, 400);
  };
  next.src = '/preview.jpg?t=' + Date.now();
}

// Flash slider real-time debounce
let flashTimer = null;
function updateFlashPreview(val) {
  document.getElementById('flash-val').textContent = val;
  clearTimeout(flashTimer);
  flashTimer = setTimeout(async () => {
    const q = new URLSearchParams({ flash: val });
    await fetch('/api/config?' + q.toString(), { method: 'POST' });
  }, 120);
}

function setPresetThresh(val) {
  document.getElementById('cfg-thresh').value = val;
  document.getElementById('thresh-val').textContent = val.toFixed(2);
}

// Enrollment Mode Switching
function setEnrollMode(mode) {
  enrollWizardMode = mode;
  document.getElementById('tab-single').className = 'tab-btn' + (mode === 'single' ? ' active' : '');
  document.getElementById('tab-multi').className = 'tab-btn' + (mode === 'multi' ? ' active' : '');
  multiStep = 0;
}

async function startEnrollment() {
  const base = document.getElementById('ename').value.trim() || 'User';
  multiBaseName = base;

  let targetName = base;
  if (enrollWizardMode === 'multi') {
    const labels = ['Front', 'Left 15deg', 'Right 15deg'];
    targetName = base + ' (' + labels[multiStep] + ')';
  }

  document.getElementById('enroll-btn').disabled = true;
  document.getElementById('cancel-btn').style.display = 'inline-flex';
  document.getElementById('prog-wrap').style.display = 'block';

  await fetch('/api/enroll?name=' + encodeURIComponent(targetName), { method: 'POST' });
  refreshStatus();
}

async function cancelEnrollment() {
  await fetch('/api/enroll/cancel', { method: 'POST' });
  multiStep = 0;
  document.getElementById('enroll-btn').disabled = false;
  document.getElementById('cancel-btn').style.display = 'none';
  document.getElementById('prog-wrap').style.display = 'none';
  refreshStatus();
}

// Polling status & updating UI
async function refreshStatus() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();

    // Hub link & storage badge
    const hub = document.getElementById('hub-badge');
    hub.textContent = d.hubConnected ? ('HUB: ' + d.ip) : (d.fallbackAp ? 'STANDALONE AP' : 'CONNECTING...');
    hub.className = 'badge ' + (d.hubConnected ? 'ok' : 'warn');

    document.getElementById('status-storage-info').textContent =
      'Flash DB: ' + (d.flashOk ? 'OK' : 'ERR') + ' | ' + d.enrolledCount + ' Faces Enrolled | Inference: ' + d.inferenceMs + ' ms';
    document.getElementById('face-count-badge').textContent = d.enrolledCount + ' / 16';

    // Video HUD elements
    const reticle = document.getElementById('reticle');
    const qPill = document.getElementById('hud-quality-pill');
    const nameBanner = document.getElementById('hud-name');
    const guideBanner = document.getElementById('hud-guide');
    const scoreBanner = document.getElementById('hud-match-score');
    document.getElementById('hud-ms').textContent = d.inferenceMs + ' ms';

    // Face Quality guidance
    if (!d.faceDetected) {
      qPill.textContent = 'NO FACE';
      qPill.className = 'hud-pill badge off';
      reticle.className = 'reticle';
      guideBanner.textContent = 'Stand 40–70cm directly in front';
    } else {
      if (d.faceQuality === 'TOO_FAR') {
        qPill.textContent = 'MOVE CLOSER';
        qPill.className = 'hud-pill badge warn';
        guideBanner.textContent = 'Move closer to the camera';
      } else if (d.faceQuality === 'TOO_CLOSE') {
        qPill.textContent = 'STEP BACK';
        qPill.className = 'hud-pill badge warn';
        guideBanner.textContent = 'Step back slightly';
      } else if (d.faceQuality === 'OFF_CENTER') {
        qPill.textContent = 'CENTER FACE';
        qPill.className = 'hud-pill badge warn';
        guideBanner.textContent = 'Position face inside the guide';
      } else {
        qPill.textContent = 'PERFECT ALIGNMENT';
        qPill.className = 'hud-pill badge ok';
        guideBanner.textContent = 'Hold steady...';
      }
    }

    // Recognition result
    if (d.enrollPending) {
      reticle.className = 'reticle enrolling';
      nameBanner.textContent = 'ENROLLING: ' + (d.enrollTargetName || 'User');
      scoreBanner.textContent = 'CAPTURE';
      scoreBanner.style.color = 'var(--cyan)';
    } else if (d.lastResult === 'RECOGNIZED') {
      reticle.className = 'reticle recognized';
      nameBanner.textContent = d.lastMatchName;
      scoreBanner.textContent = Math.round(d.lastSimilarity * 100) + '%';
      scoreBanner.style.color = 'var(--success)';
      guideBanner.textContent = 'Identity Verified';
    } else if (d.lastResult === 'UNKNOWN') {
      reticle.className = 'reticle unknown';
      nameBanner.textContent = 'UNKNOWN VISITOR';
      scoreBanner.textContent = Math.round(d.lastSimilarity * 100) + '%';
      scoreBanner.style.color = 'var(--danger)';
      guideBanner.textContent = d.cfg.armed ? 'Intruder Alert Triggered' : 'Unregistered Face';
    } else {
      nameBanner.textContent = '---';
      scoreBanner.textContent = '0%';
      scoreBanner.style.color = '#fff';
    }

    // Enrollment card status & countdown
    const progWrap = document.getElementById('prog-wrap');
    const progBar = document.getElementById('prog-bar');
    const enrollBtn = document.getElementById('enroll-btn');
    const cancelBtn = document.getElementById('cancel-btn');
    const timerText = document.getElementById('enroll-timer');
    const msgText = document.getElementById('enroll-msg');

    if (d.enrollPending) {
      progWrap.style.display = 'block';
      enrollBtn.disabled = true;
      cancelBtn.style.display = 'inline-flex';
      const pct = Math.max(0, Math.min(100, Math.round((d.enrollRemainingMs / 12000) * 100)));
      progBar.style.width = pct + '%';
      timerText.textContent = Math.ceil(d.enrollRemainingMs / 1000) + 's';
      msgText.textContent = d.enrollMessage;
    } else {
      progWrap.style.display = 'none';
      enrollBtn.disabled = false;
      cancelBtn.style.display = 'none';
      timerText.textContent = '';
      msgText.textContent = d.enrollMessage;

      // Check if multi-angle wizard was active and just finished a step
      if (enrollWizardMode === 'multi' && d.enrollMessage.indexOf('Saved') !== -1 && multiStep < 2) {
        multiStep++;
        showToast('Step ' + multiStep + '/3 complete! Preparing next angle...');
        setTimeout(() => startEnrollment(), 1400);
      }
    }

    // Render Enrolled Table
    renderFaceTable(d.enrolled, d.lastMatchId);

    // Initial config load
    if (!cfgLoaded && d.cfg) {
      cfgLoaded = true;
      document.getElementById('cfg-armed').checked = d.cfg.armed;
      document.getElementById('cfg-thresh').value = d.cfg.thresh.toFixed(2);
      document.getElementById('thresh-val').textContent = d.cfg.thresh.toFixed(2);
      document.getElementById('cfg-uconf').value = d.cfg.unknownConfirm;
      document.getElementById('flash-slider').value = d.cfg.flashDuty;
      document.getElementById('flash-val').textContent = d.cfg.flashDuty;
      document.getElementById('cfg-flash-auto').checked = d.cfg.flashAuto;
      document.getElementById('cfg-hmir').checked = d.cfg.hmirror;
      document.getElementById('cfg-vflip').checked = d.cfg.vflip;
    }
  } catch (err) {}
}

function renderFaceTable(list, activeMatchId) {
  const sig = list.map(i => i.id + ':' + i.name).join('|') + '@' + activeMatchId;
  if (sig === lastListSig) return;
  lastListSig = sig;

  const tbody = document.getElementById('face-table-body');
  tbody.innerHTML = '';

  if (list.length === 0) {
    tbody.innerHTML = '<tr><td colspan="3" style="text-align:center;color:var(--text-muted);padding:14px">No enrolled faces yet. Add one above!</td></tr>';
    return;
  }

  list.forEach(item => {
    const tr = document.createElement('tr');
    const isCurrent = (item.id === activeMatchId);

    const tdId = document.createElement('td');
    tdId.innerHTML = '<span class="badge ' + (isCurrent ? 'ok' : 'off') + '">#' + item.id + '</span>';

    const tdName = document.createElement('td');
    tdName.innerHTML = '<strong>' + escapeHtml(item.name) + '</strong>' + (isCurrent ? ' <span style="font-size:.72rem;color:var(--success)">[IN VIEW]</span>' : '');

    const tdAct = document.createElement('td');
    tdAct.style.textAlign = 'right';
    const btn = document.createElement('button');
    btn.className = 'danger';
    btn.style.padding = '3px 8px';
    btn.style.fontSize = '.75rem';
    btn.textContent = 'Delete';
    btn.onclick = () => deleteFace(item.id, item.name);
    tdAct.appendChild(btn);

    tr.appendChild(tdId);
    tr.appendChild(tdName);
    tr.appendChild(tdAct);
    tbody.appendChild(tr);
  });
}

function escapeHtml(str) {
  return str.replace(/[&<>'"]/g, tag => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' }[tag] || tag));
}

async function deleteFace(id, name) {
  if (confirm('Delete authorized face #' + id + ' ("' + name + '")?')) {
    await fetch('/api/delete?id=' + id, { method: 'POST' });
    showToast('Deleted face #' + id);
    refreshStatus();
  }
}

async function clearAllFaces() {
  if (confirm('Are you sure you want to delete ALL enrolled faces from Flash storage?')) {
    await fetch('/api/clear', { method: 'POST' });
    showToast('All faces cleared');
    refreshStatus();
  }
}

async function saveSettings() {
  const q = new URLSearchParams({
    armed: document.getElementById('cfg-armed').checked ? 1 : 0,
    thresh: document.getElementById('cfg-thresh').value,
    uconf: document.getElementById('cfg-uconf').value,
    flash: document.getElementById('flash-slider').value,
    flash_auto: document.getElementById('cfg-flash-auto').checked ? 1 : 0,
    hmir: document.getElementById('cfg-hmir').checked ? 1 : 0,
    vflip: document.getElementById('cfg-vflip').checked ? 1 : 0
  });
  await fetch('/api/config?' + q.toString(), { method: 'POST' });
  showToast('Settings saved to NVS');
  cfgLoaded = false;
  refreshStatus();
}

async function testDoor(ok) {
  await fetch('/api/test?ok=' + ok, { method: 'POST' });
  showToast(ok ? 'Triggered Valid Door Opening' : 'Triggered Intruder Siren Alert');
}

// Start live systems
initCamera();
refreshStatus();
setInterval(refreshStatus, 800);
</script>
</body>
</html>)HTML";
  server.send_P(200, "text/html", HTML);
}

void handlePreview() {
  camHwHandlePreviewRequest(server);
}

void handleStatus() {
  FaceStatusSnapshot snap;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snap = g_status;
    xSemaphoreGive(stateMutex);
  }

  String json = "{";
  json += "\"cameraReady\":" + String(snap.cameraReady ? "true" : "false") + ",";
  json += "\"modelReady\":" + String(snap.modelReady ? "true" : "false") + ",";
  json += "\"flashOk\":" + String(snap.flashOk ? "true" : "false") + ",";
  json += "\"hubConnected\":" + String(camNetHubConnected() ? "true" : "false") + ",";
  json += "\"fallbackAp\":" + String(camNetFallbackApActive() ? "true" : "false") + ",";
  json += "\"ip\":\"" + camNetIpString() + "\",";
  json += "\"streamPort\":" + String(STREAM_PORT) + ",";
  json += "\"faceDetected\":" + String(snap.faceDetected ? "true" : "false") + ",";
  json += "\"faceQuality\":\"" + String(snap.faceQuality) + "\",";
  json += "\"faceBox\":[" + String(snap.faceBox[0]) + "," + String(snap.faceBox[1]) + "," +
          String(snap.faceBox[2]) + "," + String(snap.faceBox[3]) + "],";
  json += "\"enrollPending\":" + String(snap.enrollPending ? "true" : "false") + ",";
  json += "\"enrollRemainingMs\":" + String(snap.enrollRemainingMs) + ",";
  json += "\"enrollTargetName\":\"" + smartHomeJsonEscape(snap.enrollTargetName) + "\",";
  json += "\"enrollMessage\":\"" + smartHomeJsonEscape(snap.enrollMessage) + "\",";
  json += "\"lastResult\":\"" + smartHomeJsonEscape(snap.lastResult) + "\",";
  json += "\"lastMatchName\":\"" + smartHomeJsonEscape(snap.lastMatchName) + "\",";
  json += "\"lastMatchId\":" + String(snap.lastMatchId) + ",";
  json += "\"lastSimilarity\":" + String(snap.lastSimilarity, 3) + ",";
  json += "\"inferenceMs\":" + String(snap.inferenceMs) + ",";
  json += "\"fps\":" + String(snap.fps, 1) + ",";
  json += "\"frameCount\":" + String(snap.frameCount) + ",";
  json += "\"sceneLuma\":" + String(snap.sceneLuma, 3) + ",";
  json += "\"flashDutyApplied\":" + String(snap.flashDutyApplied) + ",";
  json += "\"enrolledCount\":" + String(snap.enrolledCount) + ",";
  json += "\"enrolled\":[";
  for (size_t i = 0; i < snap.enrolled.size(); ++i) {
    if (i > 0) json += ",";
    json += "{\"id\":" + String(snap.enrolled[i].id) +
            ",\"name\":\"" + smartHomeJsonEscape(snap.enrolled[i].name) + "\"}";
  }
  json += "],\"cfg\":{";
  json += "\"armed\":" + String(snap.cfg.armed ? "true" : "false") + ",";
  json += "\"thresh\":" + String(snap.cfg.thresh, 2) + ",";
  json += "\"unknownConfirm\":" + String(snap.cfg.unknownConfirm) + ",";
  json += "\"flashDuty\":" + String(snap.cfg.flashDuty) + ",";
  json += "\"flashAuto\":" + String(snap.cfg.flashAuto ? "true" : "false") + ",";
  json += "\"hmirror\":" + String(snap.cfg.hmirror ? "true" : "false") + ",";
  json += "\"vflip\":" + String(snap.cfg.vflip ? "true" : "false");
  json += "}}";

  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(200, "application/json", json);
}

void handleEnroll() {
  if (server.hasArg("cancel") && server.arg("cancel") == "1") {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_enrollPending = false;
      g_status.enrollPending = false;
      g_status.enrollRemainingMs = 0;
      snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage), "Enrollment cancelled");
      xSemaphoreGive(stateMutex);
      server.send(200, "application/json", "{\"cancelled\":true}");
    } else {
      server.send(503, "application/json", "{\"cancelled\":false}");
    }
    return;
  }

  String rawName = server.hasArg("name") ? server.arg("name") : String("User");
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    sanitizeName(rawName, g_enrollName, sizeof(g_enrollName));
    g_enrollSeq++;
    g_enrollPending = true;
    g_enrollStartMs = millis();
    g_status.enrollPending = true;
    g_status.enrollRemainingMs = ENROLL_TIMEOUT_MS;
    snprintf(g_status.enrollTargetName, sizeof(g_status.enrollTargetName), "%s", g_enrollName);
    snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
             "Looking for face to enroll \"%s\"...", g_enrollName);
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"queued\":true}");
  } else {
    server.send(503, "application/json", "{\"queued\":false}");
  }
}

void handleEnrollCancel() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_enrollPending = false;
    g_status.enrollPending = false;
    g_status.enrollRemainingMs = 0;
    snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage), "Enrollment cancelled");
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"cancelled\":true}");
  } else {
    server.send(503, "application/json", "{\"cancelled\":false}");
  }
}

void handleDelete() {
  String rawId = server.hasArg("id") ? server.arg("id") : String("");
  bool isDigits = (rawId.length() > 0);
  for (size_t i = 0; i < rawId.length(); ++i) {
    if (!isdigit(static_cast<unsigned char>(rawId[i]))) {
      isDigits = false;
      break;
    }
  }
  int id = isDigits ? rawId.toInt() : -1;
  if (id >= 0 && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (std::find(g_pendingDeleteIds.begin(), g_pendingDeleteIds.end(), id) == g_pendingDeleteIds.end() &&
        g_pendingDeleteIds.size() < MAX_PENDING_DELETES) {
      g_pendingDeleteIds.push_back(id);
    }
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"queued\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"invalid id\"}");
  }
}

void handleClear() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_clearPending = true;
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"queued\":true}");
  } else {
    server.send(503, "application/json", "{\"queued\":false}");
  }
}

void handleConfig() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    FaceConfig next = g_applyCfgPending ? g_pendingCfg : g_status.cfg;
    if (server.hasArg("armed")) next.armed = (server.arg("armed").toInt() != 0);
    if (server.hasArg("thresh")) next.thresh = constrain(server.arg("thresh").toFloat(), 0.30f, 0.85f);
    if (server.hasArg("uconf")) next.unknownConfirm = constrain(server.arg("uconf").toInt(), 1, 5);
    if (server.hasArg("flash")) next.flashDuty = static_cast<uint8_t>(constrain(server.arg("flash").toInt(), 0, 180));
    if (server.hasArg("flash_auto")) next.flashAuto = (server.arg("flash_auto").toInt() != 0);
    if (server.hasArg("hmir")) next.hmirror = (server.arg("hmir").toInt() != 0);
    if (server.hasArg("vflip")) next.vflip = (server.arg("vflip").toInt() != 0);
    g_pendingCfg = next;
    g_applyCfgPending = true;
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(503, "application/json", "{\"ok\":false}");
  }
}

void handleTest() {
  bool ok = server.hasArg("ok") && (server.arg("ok").toInt() != 0);
  bool sent = ok ? camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED, 0, 100, "TEST-USER")
                 : camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN, -1, 0, "TEST-UNKNOWN");
  server.send(200, "application/json", sent ? "{\"sent\":true}" : "{\"sent\":false}");
}

} // namespace

void setup() {
  camHwInitLeds();

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Smart Home Camera 1: Face Recognition & Live Stream (HW-297) ===");

  stateMutex = xSemaphoreCreateMutex();
  camHwInitPreview();

  FaceConfig cfg;
  loadConfigFromNvs(cfg);
  bool camOk = camHwInitCamera(PIXFORMAT_RGB565,
                               FRAMESIZE_240X240,
                               12,
                               cfg.hmirror,
                               cfg.vflip);
  if (camOk) {
    camHwSetFlashLed(cfg.flashAuto ? 0 : cfg.flashDuty);
    Serial.println("[FACE] OV2640 camera initialized successfully (240x240 RGB565).");
  } else {
    Serial.println("[FACE] ERROR: OV2640 camera initialization failed!");
  }
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    g_status.cfg = cfg;
    g_status.cameraReady = camOk;
    xSemaphoreGive(stateMutex);
  }

  camNetInit(SmartHomeNet::CAM_SOURCE_FACE);

  // Dedicated real-time MJPEG streaming task on port 81
  camHwStartStreamServer(STREAM_PORT);

  // HTTP Web Server routes on port 80
  server.on("/", HTTP_GET, handleRoot);
  server.on("/preview.jpg", HTTP_GET, handlePreview);
  server.on("/stream", HTTP_GET, []() {
    String host = server.hostHeader();
    int colon = host.indexOf(':');
    if (colon > 0) host = host.substring(0, colon);
    server.sendHeader("Location", "http://" + host + ":81/stream");
    server.send(302, "text/plain", "Redirecting to :81/stream");
  });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/enroll", HTTP_POST, handleEnroll);
  server.on("/api/enroll/cancel", HTTP_POST, handleEnrollCancel);
  server.on("/api/enroll/cancel", HTTP_GET, handleEnrollCancel);
  server.on("/api/delete", HTTP_POST, handleDelete);
  server.on("/api/clear", HTTP_POST, handleClear);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/test", HTTP_POST, handleTest);
  server.begin();

  // Dual-core split: Core 0 = Wi-Fi/lwIP + MJPEG stream (prio 1, set in cam_hw).
  // Core 1 = face recognition worker (prio 1) + Arduino loop / HTTP + ESP-NOW (prio 1).
  // Both run cooperatively at priority 1 to ensure networking and WebServer are never starved.
  vTaskPrioritySet(nullptr, 1);

  if (camOk) {
    xTaskCreatePinnedToCore(faceWorkerTask,
                            "face_worker",
                            24576, // 24 KB stack ensures generous headroom for MobileFaceNet S8
                            nullptr,
                            1, // Equal priority to loopTask for cooperative execution
                            nullptr,
                            1); // Pin to Core 1
    Serial.println("[FACE] Face recognition worker started on Core 1 (prio 1, 24KB stack).");
  }
}

void loop() {
  camNetLoop();
  server.handleClient();
  delay(10);
}
