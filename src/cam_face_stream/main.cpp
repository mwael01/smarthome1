// ESP32-S3 face camera — high-FPS stream + remote laptop AI feedback
// Focuses exclusively on:
//   1. Real-time hardware MJPEG live video streaming (:81/stream & /preview.jpg)
//   2. Darkness + Motion Auto-Flash (5-second auto-off)
//   3. Hosting a live management Web Dashboard + REST Feedback API (/api/recognition/result)
//   4. Forwarding Laptop recognition/intruder verdicts to the Main Hub via ESP-NOW + HTTP
//
// Face detection & MobileFaceNet recognition run on the companion Laptop Analyzer instead
// of on the ESP32-CAM, maximizing streaming FPS and freeing PSRAM/CPU headroom.

#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "cam_hw.h"
#include "cam_net.h"
#include "smart_home_json.h"
#include "smart_home_protocol.h"

namespace {

constexpr unsigned long RECOGNIZED_COOLDOWN_MS = 2500;
constexpr unsigned long UNKNOWN_COOLDOWN_MS = 4000;
constexpr unsigned long ENROLL_TIMEOUT_MS = 12000;
constexpr unsigned long LAPTOP_LINK_TIMEOUT_MS = 5000;
constexpr unsigned long REMOTE_FACE_HOLD_MS = 2200;
constexpr unsigned long AUTO_FLASH_HOLD_MS = 5000;
constexpr unsigned long FLASH_SETTLE_BLANK_MS = 800;
constexpr float DARKNESS_LUMA_THRESH = 0.22f;
constexpr uint16_t STREAM_PORT = 81;
constexpr uint8_t DEFAULT_AUTO_FLASH_DUTY = 64;
constexpr size_t MAX_MOTION_JPG_BUF = 96 * 1024;
constexpr size_t MAX_ENROLLED_MIRROR = 24;

struct EnrolledEntry {
  int id;
  char name[24];
};

// frameRes: 0 = QVGA (320x240), 1 = HVGA (480x320), 2 = VGA (640x480)
struct StreamConfig {
  int unknownConfirm = 2;
  bool armed = true;
  uint8_t flashDuty = 0;
  bool flashAuto = true;
  bool hmirror = false;
  bool vflip = false;
  uint8_t frameRes = 2;
  uint8_t jpegQuality = 12;
};

struct StreamStatusSnapshot {
  bool cameraReady = false;
  bool laptopConnected = false;
  uint32_t laptopAgeMs = 0;
  char laptopModel[24] = "Laptop AI";
  bool faceDetected = false;
  bool motionDetected = false;
  char faceQuality[16] = "NO_FACE";
  int faceBox[4] = {0, 0, 0, 0}; // x1, y1, x2, y2 in current frame coordinates
  int frameWidth = 640;
  int frameHeight = 480;
  bool enrollPending = false;
  uint32_t enrollRemainingMs = 0;
  char enrollTargetName[24] = "";
  char enrollMessage[72] = "Idle: Ready for Laptop enrollment";
  char pendingAction[12] = "none"; // "none", "delete", "clear"
  uint32_t pendingActionSeq = 0;
  int pendingDeleteId = -1;
  char lastResult[24] = "NONE";
  char lastMatchName[24] = "---";
  int lastMatchId = -1;
  float lastSimilarity = 0.0f;
  uint32_t inferenceMs = 0;
  float fps = 0.0f;
  uint32_t frameCount = 0;
  float sceneLuma = 0.0f;
  uint8_t flashDutyApplied = 0;
  uint32_t feedbackCount = 0;
  int enrolledCount = 0;
  std::vector<EnrolledEntry> enrolled;
  StreamConfig cfg;
};

Preferences prefs;
WebServer server(80);
SemaphoreHandle_t stateMutex = nullptr;

StreamStatusSnapshot g_status;
StreamConfig g_pendingCfg;
bool g_applyCfgPending = false;
volatile bool g_sensorCfgPending = false;

volatile bool g_enrollPending = false;
unsigned long g_enrollStartMs = 0;
char g_enrollName[24] = "User";

volatile bool g_faceDetectedLive = false;
volatile unsigned long g_lastFaceSeenMs = 0;
volatile unsigned long g_lastLaptopFeedbackMs = 0;
volatile float g_streamFps = 0.0f;
volatile uint32_t g_capturedFrames = 0;

int g_unknownStreak = 0;
unsigned long g_lastRecognizedSentMs = 0;
unsigned long g_lastUnknownSentMs = 0;

uint8_t *g_motionJpgBuf = nullptr;

framesize_t toFrameSize(uint8_t resCode) {
  switch (resCode) {
    case 0: return FRAMESIZE_QVGA;
    case 1: return FRAMESIZE_HVGA;
    default: return FRAMESIZE_VGA;
  }
}

void resolutionDims(uint8_t resCode, int &outW, int &outH) {
  switch (resCode) {
    case 0:
      outW = 320;
      outH = 240;
      return;
    case 1:
      outW = 480;
      outH = 320;
      return;
    default:
      outW = 640;
      outH = 480;
      return;
  }
}

void loadConfigFromNvs(StreamConfig &cfg) {
  if (!prefs.begin("streamcfg", false)) return;
  cfg.unknownConfirm = constrain(prefs.getInt("uconf", 2), 1, 5);
  cfg.armed = prefs.getBool("armed", true);
  cfg.flashDuty = static_cast<uint8_t>(constrain(prefs.getUChar("flash", 0), 0, 180));
  cfg.flashAuto = prefs.getBool("flash_auto", true);
  cfg.hmirror = prefs.getBool("hmir", false);
  cfg.vflip = prefs.getBool("vflip", false);
  cfg.frameRes = static_cast<uint8_t>(constrain(prefs.getUChar("res", 2), 0, 2));
  cfg.jpegQuality = static_cast<uint8_t>(constrain(prefs.getUChar("qual", 12), 8, 30));
  prefs.end();
}

void saveConfigToNvs(const StreamConfig &cfg) {
  if (!prefs.begin("streamcfg", false)) return;
  prefs.putInt("uconf", cfg.unknownConfirm);
  prefs.putBool("armed", cfg.armed);
  prefs.putUChar("flash", cfg.flashDuty);
  prefs.putBool("flash_auto", cfg.flashAuto);
  prefs.putBool("hmir", cfg.hmirror);
  prefs.putBool("vflip", cfg.vflip);
  prefs.putUChar("res", cfg.frameRes);
  prefs.putUChar("qual", cfg.jpegQuality);
  prefs.end();
}

void sanitizeText(const String &raw, char *out, size_t outSize, const char *fallback) {
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
  if (pos == 0 && fallback) {
    strncpy(out, fallback, outSize - 1);
    out[outSize - 1] = '\0';
  }
}

void toUpperCopy(const char *src, char *dst, size_t dstSize) {
  if (!dst || dstSize == 0) return;
  size_t i = 0;
  if (src) {
    for (; src[i] != '\0' && i + 1 < dstSize; ++i) {
      dst[i] = static_cast<char>(toupper(static_cast<unsigned char>(src[i])));
    }
  }
  dst[i] = '\0';
}

// Parse a float value from JSON by key (returns true if key found).
bool jsonGetFloat(const String &json, const char *key, float &outVal) {
  if (!key) return false;
  int keyLen = strlen(key);
  int pos = 0;
  while (pos < static_cast<int>(json.length())) {
    int idx = json.indexOf(key, pos);
    if (idx < 0) return false;
    if (idx > 0 && json[idx - 1] == '"' && json[idx + keyLen] == '"') {
      int p = idx + keyLen + 1;
      while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
      if (p < static_cast<int>(json.length()) && json[p] == ':') {
        p++;
        while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
        outVal = json.substring(p).toFloat();
        return true;
      }
    }
    pos = idx + keyLen;
  }
  return false;
}

// Parse a bool (or 0/1 int) from JSON by key.
bool jsonGetBool(const String &json, const char *key, bool &outVal) {
  if (!key) return false;
  int keyLen = strlen(key);
  int pos = 0;
  while (pos < static_cast<int>(json.length())) {
    int idx = json.indexOf(key, pos);
    if (idx < 0) return false;
    if (idx > 0 && json[idx - 1] == '"' && json[idx + keyLen] == '"') {
      int p = idx + keyLen + 1;
      while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
      if (p < static_cast<int>(json.length()) && json[p] == ':') {
        p++;
        while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
        if (json.startsWith("true", p)) {
          outVal = true;
          return true;
        }
        if (json.startsWith("false", p)) {
          outVal = false;
          return true;
        }
        outVal = (json.substring(p).toInt() != 0);
        return true;
      }
    }
    pos = idx + keyLen;
  }
  return false;
}

bool parseFourIntsFromString(const String &raw, int outBox[4]) {
  int vals[4] = {0, 0, 0, 0};
  int count = 0;
  const char *p = raw.c_str();
  while (*p && count < 4) {
    while (*p && (*p == ' ' || *p == '[' || *p == ',' || *p == '\t')) p++;
    if (!*p || *p == ']') break;
    char *endPtr = nullptr;
    long v = strtol(p, &endPtr, 10);
    if (endPtr == p) break;
    vals[count++] = static_cast<int>(v);
    p = endPtr;
  }
  if (count == 4) {
    memcpy(outBox, vals, sizeof(vals));
    return true;
  }
  return false;
}

bool jsonGetBox(const String &json, const char *key, int outBox[4]) {
  if (!key) return false;
  int keyLen = strlen(key);
  int pos = 0;
  while (pos < static_cast<int>(json.length())) {
    int idx = json.indexOf(key, pos);
    if (idx < 0) return false;
    if (idx > 0 && json[idx - 1] == '"' && json[idx + keyLen] == '"') {
      int p = idx + keyLen + 1;
      while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
      if (p < static_cast<int>(json.length()) && json[p] == ':') {
        p++;
        while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
        if (p < static_cast<int>(json.length()) && json[p] == '[') {
          int closeIdx = json.indexOf(']', p);
          if (closeIdx > p) {
            return parseFourIntsFromString(json.substring(p, closeIdx + 1), outBox);
          }
        }
      }
    }
    pos = idx + keyLen;
  }
  return false;
}

// Parse {"enrolled":[{"id":0,"name":"Alice"},...]} or raw array [{"id":0,"name":"Alice"}].
bool parseEnrolledArray(const String &body, std::vector<EnrolledEntry> &outList) {
  int arrStart = body.indexOf('[');
  int arrEnd = body.lastIndexOf(']');
  if (arrStart < 0 || arrEnd < arrStart) return false;

  outList.clear();
  int cursor = arrStart;
  while (cursor < arrEnd && outList.size() < MAX_ENROLLED_MIRROR) {
    int objStart = body.indexOf('{', cursor);
    if (objStart < 0 || objStart > arrEnd) break;
    int objEnd = body.indexOf('}', objStart);
    if (objEnd < 0 || objEnd > arrEnd) break;

    String itemJson = body.substring(objStart, objEnd + 1);
    int id = -1;
    char nameBuf[24] = "";
    if (smartHomeJsonGetInt(itemJson, "id", id) && id >= 0) {
      if (!smartHomeJsonGetString(itemJson, "name", nameBuf, sizeof(nameBuf))) {
        snprintf(nameBuf, sizeof(nameBuf), "Face-%d", id);
      }
      EnrolledEntry entry = {};
      entry.id = id;
      sanitizeText(String(nameBuf), entry.name, sizeof(entry.name), "Person");
      outList.push_back(entry);
    }
    cursor = objEnd + 1;
  }
  return true;
}

// Lookup helper so unified param extraction works cleanly for query args AND JSON body.
bool readStrParam(const String &body, const char *argName, char *out, size_t outSize) {
  if (server.hasArg(argName)) {
    String v = server.arg(argName);
    strncpy(out, v.c_str(), outSize - 1);
    out[outSize - 1] = '\0';
    return true;
  }
  return !body.isEmpty() && smartHomeJsonGetString(body, argName, out, outSize);
}

bool readIntParam(const String &body, const char *argName, int &outVal) {
  if (server.hasArg(argName)) {
    outVal = server.arg(argName).toInt();
    return true;
  }
  return !body.isEmpty() && smartHomeJsonGetInt(body, argName, outVal);
}

bool readFloatParam(const String &body, const char *argName, float &outVal) {
  if (server.hasArg(argName)) {
    outVal = server.arg(argName).toFloat();
    return true;
  }
  return !body.isEmpty() && jsonGetFloat(body, argName, outVal);
}

bool readBoolParam(const String &body, const char *argName, bool &outVal) {
  if (server.hasArg(argName)) {
    String v = server.arg(argName);
    v.toLowerCase();
    outVal = (v == "1" || v == "true" || v == "yes");
    return true;
  }
  return !body.isEmpty() && jsonGetBool(body, argName, outVal);
}

// Capture on core 1 while core 0 sends the latest JPEG over Wi-Fi.
void camCaptureTask(void * /*param*/) {
  StreamConfig capCfg;
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    capCfg = g_status.cfg;
    xSemaphoreGive(stateMutex);
  }
  const bool flashAvailable = camHwFlashAvailable();
  if (!flashAvailable) {
    capCfg.flashAuto = false;
    capCfg.flashDuty = 0;
  }

  unsigned long streamFpsLastMs = millis();
  unsigned long lastMotionSampleMs = 0;
  unsigned long lastMotionMs = 0;
  unsigned long lastActivityMs = 0;
  unsigned long flashSettleUntilMs = 0;
  uint32_t streamFrames = 0;
  uint32_t frameLoopCounter = 0;
  float sceneLuma = 0.35f;
  bool motionRecent = false;
  bool flashOn = false;
  uint8_t currentDuty = capCfg.flashAuto ? 0 : capCfg.flashDuty;

  while (true) {
    if (g_sensorCfgPending) {
      StreamConfig nextCfg;
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        nextCfg = g_status.cfg;
        g_sensorCfgPending = false;
        xSemaphoreGive(stateMutex);
      } else {
        nextCfg = capCfg;
      }

      sensor_t *s = esp_camera_sensor_get();
      if (s) {
        if (nextCfg.hmirror != capCfg.hmirror && s->set_hmirror) {
          s->set_hmirror(s, nextCfg.hmirror ? 1 : 0);
        }
        if (nextCfg.vflip != capCfg.vflip && s->set_vflip) {
          s->set_vflip(s, nextCfg.vflip ? 1 : 0);
        }
        if (nextCfg.frameRes != capCfg.frameRes && s->set_framesize) {
          s->set_framesize(s, toFrameSize(nextCfg.frameRes));
          int rw = 640, rh = 480;
          resolutionDims(nextCfg.frameRes, rw, rh);
          if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_status.frameWidth = rw;
            g_status.frameHeight = rh;
            xSemaphoreGive(stateMutex);
          }
        }
        if (nextCfg.jpegQuality != capCfg.jpegQuality && s->set_quality) {
          s->set_quality(s, nextCfg.jpegQuality);
        }
      }
      if (!flashAvailable) {
        nextCfg.flashAuto = false;
        nextCfg.flashDuty = 0;
      }
      capCfg = nextCfg;
      if (!capCfg.flashAuto) {
        camHwSetFlashLed(capCfg.flashDuty);
        currentDuty = capCfg.flashDuty;
        flashOn = (capCfg.flashDuty > 0);
        flashSettleUntilMs = millis() + FLASH_SETTLE_BLANK_MS;
      }
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(8));
      continue;
    }

    unsigned long now = millis();
    bool validJpg = (fb->format == PIXFORMAT_JPEG &&
                     fb->buf != nullptr &&
                     fb->len >= 128 &&
                     fb->buf[0] == 0xFF &&
                     fb->buf[1] == 0xD8 &&
                     fb->buf[fb->len - 2] == 0xFF &&
                     fb->buf[fb->len - 1] == 0xD9);
    bool validRgb = (fb->format == PIXFORMAT_RGB565 &&
                     fb->buf != nullptr &&
                     fb->len >= 256);

    if (!validJpg && !validRgb) {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(4));
      continue;
    }

    uint16_t fbW = fb->width;
    uint16_t fbH = fb->height;
    bool inSettleWindow = (now < flashSettleUntilMs);
    bool motionNow = false;
    float motionScore = 0.0f;

    if (validJpg) {
      // 1. Immediately push raw hardware JPEG into preview/MJPEG buffer (<1ms)
      camHwUpdatePreviewJpegBytes(fb->buf, fb->len);

      // 2. Periodically snapshot the compressed JPEG (~18KB) to analyze luma/motion AFTER releasing DMA
      size_t motionCopyLen = 0;
      bool shouldSampleMotion = (g_motionJpgBuf != nullptr) &&
                                ((frameLoopCounter++ % 3U) == 0U) &&
                                (now - lastMotionSampleMs >= 110UL) &&
                                (fb->len <= MAX_MOTION_JPG_BUF);
      if (shouldSampleMotion) {
        memcpy(g_motionJpgBuf, fb->buf, fb->len);
        motionCopyLen = fb->len;
        lastMotionSampleMs = now;
      }

      // Return camera DMA descriptor immediately (<2 ms hold prevents EV-VSYNC-OVF)
      esp_camera_fb_return(fb);
      fb = nullptr;

      // 3. Analyze darkness + motion using 1/8 JPEG decode
      if (motionCopyLen > 0) {
        camera_fb_t motionFb = {};
        motionFb.buf = g_motionJpgBuf;
        motionFb.len = motionCopyLen;
        motionFb.width = fbW;
        motionFb.height = fbH;
        motionFb.format = PIXFORMAT_JPEG;

        sceneLuma = camHwAnalyzeSceneLumaAndMotion(&motionFb,
                                                   inSettleWindow,
                                                   &motionScore,
                                                   &motionNow);
      }
    } else {
      // For sensors without hardware JPEG (e.g. OV7670 / GC0308):
      // Analyze luma/motion directly on raw RGB565 pixels (<0.2 ms)
      bool shouldSampleMotion = ((frameLoopCounter++ % 3U) == 0U) &&
                                (now - lastMotionSampleMs >= 110UL);
      if (shouldSampleMotion) {
        sceneLuma = camHwAnalyzeSceneLumaAndMotion(fb,
                                                   inSettleWindow,
                                                   &motionScore,
                                                   &motionNow);
        lastMotionSampleMs = now;
      }

      // Encode RGB565 to JPEG and push to preview & MJPEG stream
      camHwUpdatePreviewJpeg(fb, PIXFORMAT_RGB565, capCfg.jpegQuality);

      esp_camera_fb_return(fb);
      fb = nullptr;
    }

    if (motionNow) {
      lastMotionMs = now;
    }
    motionRecent = (lastMotionMs != 0) && ((now - lastMotionMs) < 1200UL);
    bool remoteFacePresent = g_faceDetectedLive && ((now - g_lastFaceSeenMs) < REMOTE_FACE_HOLD_MS);
    bool enrollActive = g_enrollPending;

    if (motionNow || remoteFacePresent || enrollActive) {
      lastActivityMs = now;
    }

    uint8_t autoDuty = (capCfg.flashDuty > 0) ? capCfg.flashDuty : DEFAULT_AUTO_FLASH_DUTY;
    if (capCfg.flashAuto) {
      if (!flashOn) {
        if (sceneLuma < DARKNESS_LUMA_THRESH && (motionNow || remoteFacePresent || enrollActive)) {
          flashOn = true;
          lastActivityMs = now;
          flashSettleUntilMs = now + FLASH_SETTLE_BLANK_MS;
          camHwSetFlashLed(autoDuty);
          currentDuty = autoDuty;
          Serial.printf("[STREAM] Auto-flash ON (luma=%.2f motion=%d remoteFace=%d)\n",
                        sceneLuma, motionNow ? 1 : 0, remoteFacePresent ? 1 : 0);
        }
      } else {
        // Luma measured with the flash on includes its own light.
        if (now - lastActivityMs >= AUTO_FLASH_HOLD_MS) {
          flashOn = false;
          flashSettleUntilMs = now + FLASH_SETTLE_BLANK_MS;
          camHwSetFlashLed(0);
          currentDuty = 0;
          Serial.println("[STREAM] Auto-flash OFF (no motion or face for 5s)");
        } else if (autoDuty != currentDuty) {
          camHwSetFlashLed(autoDuty);
          currentDuty = autoDuty;
          flashSettleUntilMs = now + FLASH_SETTLE_BLANK_MS;
        }
      }
    } else {
      flashOn = (capCfg.flashDuty > 0);
      if (capCfg.flashDuty != currentDuty) {
        camHwSetFlashLed(capCfg.flashDuty);
        currentDuty = capCfg.flashDuty;
        flashSettleUntilMs = now + FLASH_SETTLE_BLANK_MS;
      }
    }

    g_capturedFrames++;
    streamFrames++;
    if (now - streamFpsLastMs >= 1000) {
      g_streamFps = (streamFrames * 1000.0f) / (now - streamFpsLastMs);
      streamFrames = 0;
      streamFpsLastMs = now;

      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_status.cameraReady = true;
        g_status.fps = g_streamFps;
        g_status.frameCount = g_capturedFrames;
        g_status.sceneLuma = sceneLuma;
        g_status.motionDetected = motionRecent;
        g_status.flashDutyApplied = currentDuty;
        g_status.frameWidth = fbW;
        g_status.frameHeight = fbH;
        xSemaphoreGive(stateMutex);
      }
    }

    vTaskDelay(camHwHasStreamClients() ? 1 : pdMS_TO_TICKS(16));
  }
}

void addOrUpdateEnrolledMirrorLocked(int id, const char *name) {
  if (id < 0 || !name) return;
  for (auto &entry : g_status.enrolled) {
    if (entry.id == id) {
      sanitizeText(String(name), entry.name, sizeof(entry.name), "Person");
      return;
    }
  }
  if (g_status.enrolled.size() < MAX_ENROLLED_MIRROR) {
    EnrolledEntry e = {};
    e.id = id;
    sanitizeText(String(name), e.name, sizeof(e.name), "Person");
    g_status.enrolled.push_back(e);
    g_status.enrolledCount = static_cast<int>(g_status.enrolled.size());
  }
}

// POST /api/recognition/result (alias: POST /api/feedback)
// Called by the Laptop Face Recognition Engine after analyzing :81/stream frames.
// Updates the ESP dashboard state and invokes Hub ESP-NOW + HTTP API for door open or alarm.
void handleRecognitionResult() {
  String body = server.hasArg("plain") ? server.arg("plain") : String("");

  char rawResult[24] = "";
  int eventCode = -1;
  bool hasResultStr = readStrParam(body, "result", rawResult, sizeof(rawResult));
  readIntParam(body, "event", eventCode);

  char normResult[24] = "NO_FACE";
  if (hasResultStr) {
    toUpperCopy(rawResult, normResult, sizeof(normResult));
  } else if (eventCode == SmartHomeNet::CAM_EVENT_RECOGNIZED) {
    strcpy(normResult, "RECOGNIZED");
  } else if (eventCode == SmartHomeNet::CAM_EVENT_UNKNOWN) {
    strcpy(normResult, "UNKNOWN");
  }

  int matchId = -1;
  if (!readIntParam(body, "matchId", matchId)) {
    readIntParam(body, "id", matchId);
  }

  char nameBuf[24] = "---";
  if (!readStrParam(body, "name", nameBuf, sizeof(nameBuf))) {
    readStrParam(body, "label", nameBuf, sizeof(nameBuf));
  }
  char cleanName[24] = "---";
  sanitizeText(String(nameBuf), cleanName, sizeof(cleanName), "---");

  float rawConf = 0.0f;
  if (!readFloatParam(body, "confidence", rawConf)) {
    if (!readFloatParam(body, "sim", rawConf)) {
      readFloatParam(body, "similarity", rawConf);
    }
  }
  float sim = (rawConf > 1.0f) ? constrain(rawConf / 100.0f, 0.0f, 1.0f)
                               : constrain(rawConf, 0.0f, 1.0f);
  uint8_t confPct = static_cast<uint8_t>(constrain(lroundf(sim * 100.0f), 0L, 100L));

  bool isRecognized = (strcmp(normResult, "RECOGNIZED") == 0 ||
                       strcmp(normResult, "MATCH") == 0 ||
                       strcmp(normResult, "ENROLLED") == 0);
  bool isUnknown = (strcmp(normResult, "UNKNOWN") == 0 || strcmp(normResult, "INTRUDER") == 0);
  bool isAligning = (strcmp(normResult, "ALIGNING") == 0 || strcmp(normResult, "VERIFYING") == 0 ||
                     strcmp(normResult, "DETECTED") == 0);

  bool faceDetected = (isRecognized || isUnknown || isAligning);
  readBoolParam(body, "faceDetected", faceDetected);

  char rawQual[16] = "";
  char qualUpper[16] = "NO_FACE";
  if (readStrParam(body, "faceQuality", rawQual, sizeof(rawQual))) {
    toUpperCopy(rawQual, qualUpper, sizeof(qualUpper));
  } else {
    strcpy(qualUpper, faceDetected ? "READY" : "NO_FACE");
  }

  int box[4] = {0, 0, 0, 0};
  bool hasBox = false;
  if (server.hasArg("box")) {
    hasBox = parseFourIntsFromString(server.arg("box"), box);
  } else if (!body.isEmpty()) {
    char boxStr[40] = "";
    if (jsonGetBox(body, "box", box)) {
      hasBox = true;
    } else if (smartHomeJsonGetString(body, "box", boxStr, sizeof(boxStr))) {
      hasBox = parseFourIntsFromString(String(boxStr), box);
    }
  }

  int inferMs = 0;
  readIntParam(body, "inferenceMs", inferMs);
  if (inferMs < 0) inferMs = 0;

  char modelBuf[24] = "";
  bool hasModel = readStrParam(body, "laptopModel", modelBuf, sizeof(modelBuf));

  bool triggerHub = true;
  readBoolParam(body, "triggerHub", triggerHub);

  bool forceDispatch = false;
  readBoolParam(body, "force", forceDispatch);

  unsigned long now = millis();
  g_lastLaptopFeedbackMs = now;
  g_faceDetectedLive = faceDetected;
  if (faceDetected) {
    g_lastFaceSeenMs = now;
  }

  StreamConfig activeCfg;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    activeCfg = g_status.cfg;
    xSemaphoreGive(stateMutex);
  }

  bool hubDispatched = false;
  const char *hubAction = "none";
  char displayResult[24] = "NO FACE";

  if (isRecognized) {
    g_unknownStreak = 0;
    if (matchId < 0) matchId = 0;
    if (strcmp(cleanName, "---") == 0) {
      snprintf(cleanName, sizeof(cleanName), "ID-%d", matchId);
    }
    strcpy(displayResult, "RECOGNIZED");
    camHwSetStatusLed(true);

    if (triggerHub && (forceDispatch || (now - g_lastRecognizedSentMs >= RECOGNIZED_COOLDOWN_MS))) {
      g_lastRecognizedSentMs = now;
      hubDispatched = camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED,
                                      static_cast<int16_t>(matchId),
                                      confPct,
                                      cleanName);
      hubAction = "DOOR_OPEN";
      Serial.printf("[REMOTE] RECOGNIZED id=%d name=\"%s\" sim=%.2f -> Hub Door Open (%s)\n",
                    matchId, cleanName, sim, hubDispatched ? "OK" : "ERR");
    } else {
      hubAction = "COOLDOWN";
    }
  } else if (isUnknown) {
    camHwSetStatusLed(false);
    matchId = -1;
    if (strcmp(cleanName, "---") == 0) {
      strcpy(cleanName, "Unknown");
    }
    g_unknownStreak++;

    if (g_unknownStreak >= activeCfg.unknownConfirm || forceDispatch) {
      strcpy(displayResult, "UNKNOWN");
      if (triggerHub && activeCfg.armed &&
          (forceDispatch || (now - g_lastUnknownSentMs >= UNKNOWN_COOLDOWN_MS))) {
        g_lastUnknownSentMs = now;
        hubDispatched = camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN,
                                        -1,
                                        confPct,
                                        "UNKNOWN");
        hubAction = "INTRUDER_ALARM";
        Serial.printf("[REMOTE] UNKNOWN INTRUDER (streak=%d sim=%.2f) -> Hub Alarm (%s)\n",
                      g_unknownStreak, sim, hubDispatched ? "OK" : "ERR");
      } else {
        hubAction = activeCfg.armed ? "COOLDOWN" : "DISARMED";
      }
    } else {
      strcpy(displayResult, "VERIFYING");
      hubAction = "CONFIRM_STREAK";
    }
  } else if (isAligning || faceDetected) {
    g_unknownStreak = 0;
    camHwSetStatusLed(false);
    strcpy(displayResult, "ALIGNING");
  } else {
    g_unknownStreak = 0;
    camHwSetStatusLed(false);
    strcpy(displayResult, "NO FACE");
  }

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_status.laptopConnected = true;
    g_status.laptopAgeMs = 0;
    if (hasModel) {
      sanitizeText(String(modelBuf), g_status.laptopModel, sizeof(g_status.laptopModel), "Laptop AI");
    }
    g_status.faceDetected = faceDetected;
    strncpy(g_status.faceQuality, qualUpper, sizeof(g_status.faceQuality) - 1);
    g_status.faceQuality[sizeof(g_status.faceQuality) - 1] = '\0';
    if (hasBox) {
      memcpy(g_status.faceBox, box, sizeof(box));
    } else if (!faceDetected) {
      memset(g_status.faceBox, 0, sizeof(g_status.faceBox));
    }
    g_status.lastMatchId = matchId;
    g_status.lastSimilarity = sim;
    g_status.inferenceMs = static_cast<uint32_t>(inferMs);
    g_status.feedbackCount++;
    snprintf(g_status.lastResult, sizeof(g_status.lastResult), "%s", displayResult);
    snprintf(g_status.lastMatchName, sizeof(g_status.lastMatchName), "%s", cleanName);
    if (strcmp(normResult, "ENROLLED") == 0 && matchId >= 0) {
      addOrUpdateEnrolledMirrorLocked(matchId, cleanName);
    }
    xSemaphoreGive(stateMutex);
  }

  String resp = "{";
  resp += "\"ok\":true,";
  resp += "\"result\":\"" + smartHomeJsonEscape(displayResult) + "\",";
  resp += "\"hubDispatched\":" + String(hubDispatched ? "true" : "false") + ",";
  resp += "\"hubAction\":\"" + String(hubAction) + "\",";
  resp += "\"armed\":" + String(activeCfg.armed ? "true" : "false") + ",";
  resp += "\"unknownStreak\":" + String(g_unknownStreak) + ",";
  resp += "\"unknownConfirm\":" + String(activeCfg.unknownConfirm);
  resp += "}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(200, "application/json", resp);
}

void handleLaptopHeartbeat() {
  String body = server.hasArg("plain") ? server.arg("plain") : String("");
  char modelBuf[24] = "";
  bool hasModel = readStrParam(body, "laptopModel", modelBuf, sizeof(modelBuf));

  g_lastLaptopFeedbackMs = millis();
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_status.laptopConnected = true;
    g_status.laptopAgeMs = 0;
    if (hasModel) {
      sanitizeText(String(modelBuf), g_status.laptopModel, sizeof(g_status.laptopModel), "Laptop AI");
    }
    xSemaphoreGive(stateMutex);
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleLaptopEnrolled() {
  String body = server.hasArg("plain") ? server.arg("plain") : String("");
  std::vector<EnrolledEntry> parsed;
  if (!parseEnrolledArray(body, parsed)) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid enrolled json array\"}");
    return;
  }

  g_lastLaptopFeedbackMs = millis();
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_status.laptopConnected = true;
    g_status.laptopAgeMs = 0;
    g_status.enrolled = parsed;
    g_status.enrolledCount = static_cast<int>(parsed.size());
    xSemaphoreGive(stateMutex);
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json",
              "{\"ok\":true,\"count\":" + String(parsed.size()) + "}");
}

void handleEnrollAck() {
  String body = server.hasArg("plain") ? server.arg("plain") : String("");
  bool ok = false;
  if (!readBoolParam(body, "ok", ok)) {
    int idCheck = -1;
    ok = readIntParam(body, "id", idCheck) && idCheck >= 0;
  }
  int newId = 0;
  if (!readIntParam(body, "id", newId)) {
    readIntParam(body, "matchId", newId);
  }
  char nameBuf[24] = "";
  if (!readStrParam(body, "name", nameBuf, sizeof(nameBuf))) {
    snprintf(nameBuf, sizeof(nameBuf), "%s", g_enrollName);
  }
  char cleanName[24] = "User";
  sanitizeText(String(nameBuf), cleanName, sizeof(cleanName), "User");

  char errMsg[64] = "Enroll failed on Laptop";
  readStrParam(body, "msg", errMsg, sizeof(errMsg));

  g_lastLaptopFeedbackMs = millis();
  bool hubSent = false;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_status.laptopConnected = true;
    g_status.laptopAgeMs = 0;
    g_enrollPending = false;
    g_status.enrollPending = false;
    g_status.enrollRemainingMs = 0;

    if (ok && newId >= 0) {
      addOrUpdateEnrolledMirrorLocked(newId, cleanName);
      snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
               "Saved \"%s\" on Laptop as ID #%d", cleanName, newId);
      g_status.lastMatchId = newId;
      g_status.lastSimilarity = 1.0f;
      snprintf(g_status.lastMatchName, sizeof(g_status.lastMatchName), "%s", cleanName);
      snprintf(g_status.lastResult, sizeof(g_status.lastResult), "RECOGNIZED");
    } else {
      snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage), "%s", errMsg);
    }
    xSemaphoreGive(stateMutex);
  }

  if (ok && newId >= 0) {
    g_lastRecognizedSentMs = millis();
    hubSent = camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED,
                              static_cast<int16_t>(newId),
                              100,
                              cleanName);
  }

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json",
              String("{\"ok\":") + (ok ? "true" : "false") +
                  ",\"hubDispatched\":" + (hubSent ? "true" : "false") + "}");
}

void handleCommandAck() {
  String body = server.hasArg("plain") ? server.arg("plain") : String("");
  int seq = -1;
  readIntParam(body, "seq", seq);

  g_lastLaptopFeedbackMs = millis();
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_status.laptopConnected = true;
    g_status.laptopAgeMs = 0;
    if (seq < 0 || static_cast<uint32_t>(seq) == g_status.pendingActionSeq) {
      if (strcmp(g_status.pendingAction, "delete") == 0) {
        snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                 "Laptop removed face #%d", g_status.pendingDeleteId);
      } else if (strcmp(g_status.pendingAction, "clear") == 0) {
        snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                 "Laptop cleared all faces");
      }
      strcpy(g_status.pendingAction, "none");
      g_status.pendingDeleteId = -1;
    }
    xSemaphoreGive(stateMutex);
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStatus() {
  unsigned long now = millis();
  // Return a safe health snapshot even if the capture task briefly owns the
  // mutex. Missing laptop feedback must never make this endpoint fail.
  StreamStatusSnapshot snap = {};
  snap.frameWidth = 640;
  snap.frameHeight = 480;
  strcpy(snap.laptopModel, "Laptop AI");
  strcpy(snap.faceQuality, "NO_FACE");
  strcpy(snap.lastResult, "NONE");
  strcpy(snap.lastMatchName, "---");
  strcpy(snap.pendingAction, "none");
  snap.pendingDeleteId = -1;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    if (g_lastLaptopFeedbackMs != 0 && (now - g_lastLaptopFeedbackMs <= LAPTOP_LINK_TIMEOUT_MS)) {
      g_status.laptopConnected = true;
      g_status.laptopAgeMs = static_cast<uint32_t>(now - g_lastLaptopFeedbackMs);
    } else {
      if (g_status.laptopConnected) {
        g_status.laptopConnected = false;
        g_status.faceDetected = false;
        g_faceDetectedLive = false;
        memset(g_status.faceBox, 0, sizeof(g_status.faceBox));
        strcpy(g_status.faceQuality, "NO_FACE");
        strcpy(g_status.lastResult, "NONE");
        strcpy(g_status.lastMatchName, "---");
        g_status.lastMatchId = -1;
        g_status.lastSimilarity = 0.0f;
        camHwSetStatusLed(false);
      }
      g_status.laptopAgeMs = (g_lastLaptopFeedbackMs == 0)
                                 ? 0
                                 : static_cast<uint32_t>(now - g_lastLaptopFeedbackMs);
    }

    if (g_enrollPending) {
      unsigned long elapsed = now - g_enrollStartMs;
      if (elapsed >= ENROLL_TIMEOUT_MS) {
        g_enrollPending = false;
        g_status.enrollPending = false;
        g_status.enrollRemainingMs = 0;
        snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                 "Enroll timed out: wait for Laptop confirmation");
      } else {
        g_status.enrollRemainingMs = ENROLL_TIMEOUT_MS - elapsed;
      }
    } else {
      g_status.enrollRemainingMs = 0;
    }

    snap = g_status;
    xSemaphoreGive(stateMutex);
  }

  String json = "{";
  json += "\"mode\":\"REMOTE_LAPTOP_AI\",";
  json += "\"cameraReady\":" + String(snap.cameraReady ? "true" : "false") + ",";
  json += "\"modelReady\":" + String(snap.laptopConnected ? "true" : "false") + ",";
  json += "\"flashOk\":" + String(camHwFlashAvailable() ? "true" : "false") + ",";
  json += "\"laptopConnected\":" + String(snap.laptopConnected ? "true" : "false") + ",";
  json += "\"laptopAgeMs\":" + String(snap.laptopAgeMs) + ",";
  json += "\"laptopModel\":\"" + smartHomeJsonEscape(snap.laptopModel) + "\",";
  json += "\"feedbackCount\":" + String(snap.feedbackCount) + ",";
  json += "\"hubConnected\":" + String(camNetHubConnected() ? "true" : "false") + ",";
  json += "\"fallbackAp\":" + String(camNetFallbackApActive() ? "true" : "false") + ",";
  json += "\"ip\":\"" + camNetIpString() + "\",";
  json += "\"streamPort\":" + String(STREAM_PORT) + ",";
  json += "\"frameWidth\":" + String(snap.frameWidth) + ",";
  json += "\"frameHeight\":" + String(snap.frameHeight) + ",";
  json += "\"faceDetected\":" + String(snap.faceDetected ? "true" : "false") + ",";
  json += "\"motionDetected\":" + String(snap.motionDetected ? "true" : "false") + ",";
  json += "\"faceQuality\":\"" + String(snap.faceQuality) + "\",";
  json += "\"faceBox\":[" + String(snap.faceBox[0]) + "," + String(snap.faceBox[1]) + "," +
          String(snap.faceBox[2]) + "," + String(snap.faceBox[3]) + "],";
  json += "\"enrollPending\":" + String(snap.enrollPending ? "true" : "false") + ",";
  json += "\"enrollRemainingMs\":" + String(snap.enrollRemainingMs) + ",";
  json += "\"enrollTargetName\":\"" + smartHomeJsonEscape(snap.enrollTargetName) + "\",";
  json += "\"enrollMessage\":\"" + smartHomeJsonEscape(snap.enrollMessage) + "\",";
  json += "\"pendingAction\":\"" + smartHomeJsonEscape(snap.pendingAction) + "\",";
  json += "\"pendingActionSeq\":" + String(snap.pendingActionSeq) + ",";
  json += "\"pendingDeleteId\":" + String(snap.pendingDeleteId) + ",";
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
  json += "\"unknownConfirm\":" + String(snap.cfg.unknownConfirm) + ",";
  json += "\"flashDuty\":" + String(snap.cfg.flashDuty) + ",";
  json += "\"flashAuto\":" + String(snap.cfg.flashAuto ? "true" : "false") + ",";
  json += "\"hmirror\":" + String(snap.cfg.hmirror ? "true" : "false") + ",";
  json += "\"vflip\":" + String(snap.cfg.vflip ? "true" : "false") + ",";
  json += "\"frameRes\":" + String(snap.cfg.frameRes) + ",";
  json += "\"jpegQuality\":" + String(snap.cfg.jpegQuality);
  json += "}}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
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
    sanitizeText(rawName, g_enrollName, sizeof(g_enrollName), "Person");
    g_enrollPending = true;
    g_enrollStartMs = millis();
    g_status.enrollPending = true;
    g_status.enrollRemainingMs = ENROLL_TIMEOUT_MS;
    snprintf(g_status.enrollTargetName, sizeof(g_status.enrollTargetName), "%s", g_enrollName);
    snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
             "Waiting for Laptop to capture \"%s\"...", g_enrollName);
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
    strcpy(g_status.pendingAction, "delete");
    g_status.pendingActionSeq++;
    g_status.pendingDeleteId = id;
    auto &vec = g_status.enrolled;
    vec.erase(std::remove_if(vec.begin(), vec.end(),
                             [id](const EnrolledEntry &e) { return e.id == id; }),
              vec.end());
    g_status.enrolledCount = static_cast<int>(vec.size());
    snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
             "Requested Laptop to delete ID #%d", id);
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"queued\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"invalid id\"}");
  }
}

void handleClear() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    strcpy(g_status.pendingAction, "clear");
    g_status.pendingActionSeq++;
    g_status.pendingDeleteId = -1;
    g_status.enrolled.clear();
    g_status.enrolledCount = 0;
    snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
             "Requested Laptop to clear all faces");
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"queued\":true}");
  } else {
    server.send(503, "application/json", "{\"queued\":false}");
  }
}

void handleConfig() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    StreamConfig next = g_applyCfgPending ? g_pendingCfg : g_status.cfg;
    if (server.hasArg("armed")) next.armed = (server.arg("armed").toInt() != 0);
    if (server.hasArg("uconf")) next.unknownConfirm = constrain(server.arg("uconf").toInt(), 1, 5);
    if (server.hasArg("flash")) next.flashDuty = static_cast<uint8_t>(constrain(server.arg("flash").toInt(), 0, 180));
    if (server.hasArg("flash_auto")) next.flashAuto = (server.arg("flash_auto").toInt() != 0);
    if (server.hasArg("hmir")) next.hmirror = (server.arg("hmir").toInt() != 0);
    if (server.hasArg("vflip")) next.vflip = (server.arg("vflip").toInt() != 0);
    if (server.hasArg("res")) next.frameRes = static_cast<uint8_t>(constrain(server.arg("res").toInt(), 0, 2));
    if (server.hasArg("qual")) next.jpegQuality = static_cast<uint8_t>(constrain(server.arg("qual").toInt(), 8, 30));
    g_pendingCfg = next;
    g_status.cfg = next;
    g_applyCfgPending = true;
    g_sensorCfgPending = true;
    xSemaphoreGive(stateMutex);
    saveConfigToNvs(next);
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

void handlePreview() {
  camHwHandlePreviewRequest(server);
}

void handleRoot() {
  static const char HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Smart Camera 1 — High-FPS Stream &amp; Laptop AI</title>
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
  .wrap { max-width: 1060px; margin: 0 auto; }
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
  h1 { font-size: 1.2rem; font-weight: 700; margin: 0; display: flex; align-items: center; gap: 8px; }
  .pulse-dot {
    width: 10px; height: 10px; border-radius: 50%; background: var(--cyan);
    box-shadow: 0 0 10px var(--cyan); animation: pulse 1.8s infinite;
  }
  @keyframes pulse { 0%, 100% { opacity: 1; transform: scale(1); } 50% { opacity: .4; transform: scale(.85); } }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(330px, 1fr)); gap: 16px; }
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
  .player-box {
    position: relative;
    width: 100%;
    max-width: 480px;
    margin: 0 auto 12px;
    border-radius: var(--radius);
    overflow: hidden;
    background: #020617;
    border: 1px solid var(--card-border);
    aspect-ratio: 4 / 3;
    box-shadow: 0 8px 24px rgba(0,0,0,.5);
  }
  .player-box img {
    width: 100%;
    height: 100%;
    object-fit: cover;
    display: block;
    background: #000;
  }
  .bbox-overlay {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 100%;
    pointer-events: none;
  }
  .reticle {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    width: 44%;
    height: 70%;
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
  .reticle.recognized { border: 3px solid var(--success); box-shadow: 0 0 18px rgba(16, 185, 129, 0.45); }
  .reticle.unknown { border: 3px solid var(--danger); box-shadow: 0 0 18px rgba(239, 68, 68, 0.45); }
  @keyframes reticlePulse {
    0%, 100% { transform: translate(-50%, -50%) scale(1); opacity: 1; }
    50% { transform: translate(-50%, -50%) scale(1.04); opacity: .7; }
  }
  .hud-top {
    position: absolute; top: 10px; left: 10px; right: 10px;
    display: flex; justify-content: space-between; pointer-events: none;
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
    position: absolute; bottom: 0; left: 0; right: 0;
    background: linear-gradient(to top, rgba(2, 6, 23, 0.95), rgba(2, 6, 23, 0));
    padding: 24px 12px 10px;
    display: flex; justify-content: space-between; align-items: flex-end; pointer-events: none;
  }
  .hud-banner-title { font-size: .92rem; font-weight: 700; }
  .hud-banner-sub { font-size: .75rem; color: var(--text-muted); }

  button {
    background: var(--primary); color: #fff; border: 0; border-radius: 8px;
    padding: 8px 14px; font-size: .88rem; font-weight: 600; cursor: pointer;
    transition: all .15s ease; display: inline-flex; align-items: center; gap: 6px;
  }
  button:hover { background: var(--primary-hover); transform: translateY(-1px); }
  button.danger { background: var(--danger); }
  button.danger:hover { background: #dc2626; }
  button.success { background: var(--success); }
  button.success:hover { background: #059669; }
  button.outline { background: transparent; border: 1px solid var(--card-border); color: var(--text-muted); }
  button.outline:hover { background: var(--card-hover); color: var(--text); }
  button:disabled { opacity: .5; cursor: not-allowed; transform: none; }

  input[type="text"], input[type="number"], select {
    background: #0b1322; color: var(--text); border: 1px solid var(--card-border);
    border-radius: 8px; padding: 8px 12px; font-size: .9rem; width: 100%; outline: none;
  }
  .row {
    display: flex; justify-content: space-between; align-items: center;
    padding: 7px 0; border-bottom: 1px solid rgba(255, 255, 255, 0.05); font-size: .88rem;
  }
  .row:last-child { border-bottom: 0; }
  .badge {
    padding: 2px 8px; border-radius: 999px; font-size: .74rem; font-weight: 700; letter-spacing: .3px;
  }
  .badge.ok { background: rgba(16, 185, 129, 0.2); color: #6ee7b7; border: 1px solid rgba(16, 185, 129, 0.35); }
  .badge.bad { background: rgba(239, 68, 68, 0.2); color: #fca5a5; border: 1px solid rgba(239, 68, 68, 0.35); }
  .badge.warn { background: rgba(245, 158, 11, 0.2); color: #fde68a; border: 1px solid rgba(245, 158, 11, 0.35); }
  .badge.info { background: rgba(6, 182, 212, 0.2); color: #a5f3fc; border: 1px solid rgba(6, 182, 212, 0.35); }
  .badge.off { background: rgba(148, 163, 184, 0.15); color: #94a3b8; }

  .progress-wrap {
    height: 8px; background: #0b1322; border-radius: 999px; overflow: hidden;
    margin: 8px 0; display: none; border: 1px solid var(--card-border);
  }
  .progress-bar { height: 100%; width: 0%; background: linear-gradient(90deg, var(--cyan), var(--primary)); }
  table { width: 100%; border-collapse: collapse; margin-top: 8px; font-size: .88rem; }
  th, td { padding: 8px; text-align: left; border-bottom: 1px solid var(--card-border); }
  th { color: var(--text-muted); font-weight: 600; font-size: .75rem; text-transform: uppercase; }

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

  .api-box {
    background: #0b1322; border: 1px solid var(--card-border); border-radius: 8px;
    padding: 10px 12px; font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    font-size: .76rem; color: #93c5fd; margin-top: 10px; line-height: 1.55; word-break: break-all;
  }
  #toast {
    position: fixed; bottom: 20px; right: 20px; background: #1e293b; border: 1px solid var(--card-border);
    color: #fff; padding: 10px 18px; border-radius: 8px; box-shadow: 0 6px 18px rgba(0,0,0,.5);
    transform: translateY(100px); opacity: 0; transition: all .25s ease; z-index: 999; font-size: .88rem;
  }
  #toast.show { transform: translateY(0); opacity: 1; }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1><span class="pulse-dot"></span> Camera 1 — Clean Stream + Laptop AI</h1>
    <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap">
      <span id="laptop-badge" class="badge warn">LAPTOP AI: WAITING</span>
      <span id="hub-badge" class="badge off">CHECKING HUB</span>
      <span id="fps-badge" class="badge info">0.0 FPS</span>
    </div>
  </header>

  <div class="grid">
    <div class="card">
      <div class="card-title">
        <span>Clean Raw Camera Feed</span>
        <button class="outline" style="padding:4px 8px;font-size:.78rem" onclick="toggleStreamMode()" id="stream-mode-btn">Mode: MJPEG</button>
      </div>

      <div class="player-box" id="player-container">
        <img id="cam" alt="Camera Live Stream">
        <svg id="bbox-svg" class="bbox-overlay" viewBox="0 0 640 480" preserveAspectRatio="none">
          <rect id="bbox-rect" x="0" y="0" width="0" height="0" fill="none" stroke="#10b981" stroke-width="4" rx="6" style="display:none"/>
        </svg>
        <div id="reticle" class="reticle"></div>

        <div class="hud-top">
          <div class="hud-pill">
            <span style="display:inline-block;width:7px;height:7px;border-radius:50%;background:var(--cyan)"></span>
            <span id="hud-stream-text">STREAMING :81</span>
          </div>
          <div class="hud-pill badge off" id="hud-quality-pill">NO FACE</div>
        </div>

        <div class="hud-bottom">
          <div>
            <div class="hud-banner-title" id="hud-name">---</div>
            <div class="hud-banner-sub" id="hud-guide">Waiting for Laptop analyzer feedback</div>
          </div>
          <div style="text-align:right">
            <div id="hud-match-score" style="font-weight:700;font-size:1.1rem;color:#fff">0%</div>
            <div style="font-size:.72rem;color:var(--text-muted)" id="hud-ms">0 ms</div>
          </div>
        </div>
      </div>

      <div class="row">
        <span id="flash-label">Flash LED Brightness</span>
        <div style="display:flex;align-items:center;gap:8px;width:55%">
          <input id="flash-slider" type="range" min="0" max="180" value="0" style="flex:1" oninput="updateFlashPreview(this.value)" disabled>
          <span id="flash-val" style="font-size:.8rem;width:28px;text-align:right">0</span>
        </div>
      </div>

      <div class="row">
        <span>Auto Flash (Darkness + Motion, 5s Off)</span>
        <label class="switch"><input id="cfg-flash-auto" type="checkbox" disabled><span class="slider"></span></label>
      </div>

      <div class="row">
        <span>Pipeline Telemetry</span>
        <span id="status-storage-info" style="font-size:.8rem;color:var(--text-muted)">Offloaded AI | 0 updates</span>
      </div>

      <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px">
        <button class="success" onclick="testDoor(1)" style="justify-content:center">Test Door Open</button>
        <button class="danger" onclick="testDoor(0)" style="justify-content:center">Test Intruder Siren</button>
      </div>

      <div class="api-box">
        <div><strong>Stream URL:</strong> <span id="api-stream-url">-</span></div>
        <div><strong>Feedback API:</strong> POST <span id="api-fb-url">-</span></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">
        <span>Remote Laptop Face Enrollment</span>
        <span id="face-count-badge" class="badge off">0 Enrolled</span>
      </div>

      <div style="margin-bottom:12px">
        <label style="font-size:.82rem;color:var(--text-muted);display:block;margin-bottom:4px">Person Name (Captured by Laptop)</label>
        <div style="display:flex;gap:8px">
          <input id="ename" type="text" maxlength="20" placeholder="e.g. Alice, Khaled" value="User">
          <button id="enroll-btn" onclick="startEnrollment()" style="white-space:nowrap">Enroll Face</button>
          <button id="cancel-btn" class="danger" onclick="cancelEnrollment()" style="display:none;white-space:nowrap">Cancel</button>
        </div>
      </div>

      <div class="progress-wrap" id="prog-wrap">
        <div class="progress-bar" id="prog-bar"></div>
      </div>

      <div style="background:#0b1322;border:1px solid var(--card-border);border-radius:8px;padding:9px 12px;font-size:.84rem;color:#93c5fd;margin-bottom:12px;display:flex;justify-content:space-between;align-items:center">
        <span id="enroll-msg">Idle: Ready for Laptop enrollment</span>
        <span id="enroll-timer" style="font-weight:700"></span>
      </div>

      <div style="display:flex;justify-content:space-between;align-items:center;margin-top:14px">
        <span style="font-weight:700;font-size:.9rem">Enrolled Faces (Laptop Directory)</span>
        <button class="outline danger" style="padding:4px 8px;font-size:.78rem" onclick="clearAllFaces()">Clear All</button>
      </div>

      <div style="max-height:200px;overflow-y:auto;margin-top:6px">
        <table>
          <thead><tr><th>ID</th><th>Name</th><th style="text-align:right">Action</th></tr></thead>
          <tbody id="face-table-body">
            <tr><td colspan="3" style="text-align:center;color:var(--text-muted)">Waiting for Laptop face list sync...</td></tr>
          </tbody>
        </table>
      </div>

      <hr style="border:0;border-top:1px solid var(--card-border);margin:16px 0 12px">

      <div class="card-title" style="margin-bottom:8px">
        <span>Stream &amp; Security Settings</span>
      </div>

      <div class="row">
        <span>Intruder Siren Armed</span>
        <label class="switch"><input id="cfg-armed" type="checkbox"><span class="slider"></span></label>
      </div>

      <div class="row">
        <span>Stream Resolution</span>
        <select id="cfg-res" style="width:55%">
          <option value="0">QVGA (320x240 - Max FPS)</option>
          <option value="1">HVGA (480x320 - Balanced)</option>
          <option value="2">VGA (640x480 - Best Detail)</option>
        </select>
      </div>

      <div class="row">
        <span>JPEG Quality (<span id="qual-val">12</span> = sharp)</span>
        <div style="display:flex;align-items:center;gap:6px;width:55%">
          <input id="cfg-qual" type="range" min="8" max="30" step="1" value="12" style="flex:1" oninput="document.getElementById('qual-val').textContent=this.value">
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
let currentMode = 'mjpeg';
let mjpegFails = 0;
let snapshotActive = false;
let mjpegRetryTimer = null;
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

function initCamera() {
  const img = document.getElementById('cam');
  const baseHost = window.location.protocol + '//' + window.location.hostname;
  document.getElementById('api-stream-url').textContent = baseHost + ':81/stream';
  document.getElementById('api-fb-url').textContent = baseHost + '/api/recognition/result';

  img.onload = () => {
    mjpegFails = 0;
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
      if (mjpegFails <= 3) {
        setTimeout(() => {
          if (currentMode === 'mjpeg') {
            img.src = baseHost + ':81/stream?t=' + Date.now();
          }
        }, 600);
      } else {
        setStreamMode('snapshot');
        mjpegRetryTimer = setTimeout(() => setStreamMode('mjpeg'), 10000);
      }
    }
  };

  setStreamMode('mjpeg');
}

function setStreamMode(mode) {
  clearTimeout(mjpegRetryTimer);
  currentMode = mode;
  const img = document.getElementById('cam');
  const btn = document.getElementById('stream-mode-btn');
  const pill = document.getElementById('hud-stream-text');
  if (mode === 'mjpeg') {
    mjpegFails = 0;
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

let flashTimer = null;
function updateFlashPreview(val) {
  document.getElementById('flash-val').textContent = val;
  clearTimeout(flashTimer);
  flashTimer = setTimeout(async () => {
    const q = new URLSearchParams({ flash: val });
    await fetch('/api/config?' + q.toString(), { method: 'POST' });
  }, 120);
}

async function startEnrollment() {
  const name = document.getElementById('ename').value.trim() || 'User';
  document.getElementById('enroll-btn').disabled = true;
  document.getElementById('cancel-btn').style.display = 'inline-flex';
  document.getElementById('prog-wrap').style.display = 'block';
  await fetch('/api/enroll?name=' + encodeURIComponent(name), { method: 'POST' });
  refreshStatus();
}

async function cancelEnrollment() {
  await fetch('/api/enroll/cancel', { method: 'POST' });
  document.getElementById('enroll-btn').disabled = false;
  document.getElementById('cancel-btn').style.display = 'none';
  document.getElementById('prog-wrap').style.display = 'none';
  refreshStatus();
}

async function refreshStatus() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();

    const laptopBadge = document.getElementById('laptop-badge');
    if (d.laptopConnected) {
      laptopBadge.textContent = 'LAPTOP AI: ' + (d.laptopModel || 'ONLINE');
      laptopBadge.className = 'badge ok';
    } else {
      laptopBadge.textContent = 'LAPTOP AI: WAITING';
      laptopBadge.className = 'badge warn';
    }

    const hub = document.getElementById('hub-badge');
    hub.textContent = d.hubConnected ? ('HUB: ' + d.ip) : (d.fallbackAp ? 'STANDALONE AP' : 'CONNECTING...');
    hub.className = 'badge ' + (d.hubConnected ? 'ok' : 'warn');

    document.getElementById('status-storage-info').textContent =
      (d.frameWidth || 640) + 'x' + (d.frameHeight || 480) + ' | ' + d.fps.toFixed(1) + ' FPS | ' + d.feedbackCount + ' Laptop events';
    document.getElementById('face-count-badge').textContent = d.enrolledCount + ' Enrolled';

    const reticle = document.getElementById('reticle');
    const qPill = document.getElementById('hud-quality-pill');
    const nameBanner = document.getElementById('hud-name');
    const guideBanner = document.getElementById('hud-guide');
    const scoreBanner = document.getElementById('hud-match-score');
    document.getElementById('hud-ms').textContent = d.inferenceMs + ' ms';

    // Keep player container aspect ratio synced to active resolution
    const fw = d.frameWidth || 640;
    const fh = d.frameHeight || 480;
    const svg = document.getElementById('bbox-svg');
    svg.setAttribute('viewBox', '0 0 ' + fw + ' ' + fh);
    const rect = document.getElementById('bbox-rect');
    const bx = d.faceBox || [0, 0, 0, 0];
    if (d.faceDetected && bx[2] > bx[0] && bx[3] > bx[1]) {
      rect.setAttribute('x', bx[0]);
      rect.setAttribute('y', bx[1]);
      rect.setAttribute('width', bx[2] - bx[0]);
      rect.setAttribute('height', bx[3] - bx[1]);
      rect.setAttribute('stroke', d.lastResult === 'RECOGNIZED' ? '#10b981' : (d.lastResult === 'UNKNOWN' ? '#ef4444' : '#f59e0b'));
      rect.style.display = 'block';
    } else {
      rect.style.display = 'none';
    }

    if (!d.laptopConnected) {
      qPill.textContent = 'LAPTOP OFFLINE';
      qPill.className = 'hud-pill badge warn';
      reticle.className = 'reticle';
      guideBanner.textContent = 'Connect Laptop AI script to :81/stream';
    } else if (!d.faceDetected) {
      qPill.textContent = 'NO FACE';
      qPill.className = 'hud-pill badge off';
      reticle.className = 'reticle';
      guideBanner.textContent = 'Stand in front of camera';
    } else {
      const qual = d.faceQuality || 'READY';
      qPill.textContent = qual === 'READY' ? 'FACE CLEAR' : qual.replace('_', ' ');
      qPill.className = 'hud-pill badge ' + (qual === 'READY' ? 'ok' : 'warn');
      guideBanner.textContent = qual === 'READY' ? 'Analyzing via Laptop AI...' : 'Adjust position: ' + qual.replace('_', ' ');
    }

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
      guideBanner.textContent = 'Identity Verified (Laptop AI -> Hub)';
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
      progBar.style.width = Math.max(0, Math.min(100, Math.round((d.enrollRemainingMs / 12000) * 100))) + '%';
      timerText.textContent = Math.ceil(d.enrollRemainingMs / 1000) + 's';
      msgText.textContent = d.enrollMessage;
    } else {
      progWrap.style.display = 'none';
      enrollBtn.disabled = false;
      cancelBtn.style.display = 'none';
      timerText.textContent = '';
      msgText.textContent = d.enrollMessage;
    }

    renderFaceTable(d.enrolled || [], d.lastMatchId);
    document.getElementById('flash-label').textContent = d.flashOk ? 'Flash LED Brightness' : 'Flash output unavailable';
    document.getElementById('flash-slider').disabled = !d.flashOk;
    document.getElementById('cfg-flash-auto').disabled = !d.flashOk;

    if (!cfgLoaded && d.cfg) {
      cfgLoaded = true;
      document.getElementById('cfg-armed').checked = d.cfg.armed;
      document.getElementById('cfg-uconf').value = d.cfg.unknownConfirm;
      document.getElementById('flash-slider').value = d.cfg.flashDuty;
      document.getElementById('flash-val').textContent = d.cfg.flashDuty;
      document.getElementById('cfg-flash-auto').checked = d.cfg.flashAuto;
      document.getElementById('cfg-hmir').checked = d.cfg.hmirror;
      document.getElementById('cfg-vflip').checked = d.cfg.vflip;
      document.getElementById('cfg-res').value = String(d.cfg.frameRes);
      document.getElementById('cfg-qual').value = String(d.cfg.jpegQuality);
      document.getElementById('qual-val').textContent = String(d.cfg.jpegQuality);
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
    tbody.innerHTML = '<tr><td colspan="3" style="text-align:center;color:var(--text-muted);padding:14px">No enrolled faces synced from Laptop yet.</td></tr>';
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
  if (confirm('Delete face #' + id + ' ("' + name + '") on Laptop?')) {
    await fetch('/api/delete?id=' + id, { method: 'POST' });
    showToast('Requested delete for #' + id);
    refreshStatus();
  }
}

async function clearAllFaces() {
  if (confirm('Clear ALL enrolled faces from Laptop directory?')) {
    await fetch('/api/clear', { method: 'POST' });
    showToast('Requested clear all faces');
    refreshStatus();
  }
}

async function saveSettings() {
  const q = new URLSearchParams({
    armed: document.getElementById('cfg-armed').checked ? 1 : 0,
    uconf: document.getElementById('cfg-uconf').value,
    flash: document.getElementById('flash-slider').value,
    flash_auto: document.getElementById('cfg-flash-auto').checked ? 1 : 0,
    hmir: document.getElementById('cfg-hmir').checked ? 1 : 0,
    vflip: document.getElementById('cfg-vflip').checked ? 1 : 0,
    res: document.getElementById('cfg-res').value,
    qual: document.getElementById('cfg-qual').value
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

initCamera();
refreshStatus();
setInterval(refreshStatus, 800);
</script>
</body>
</html>)HTML";
  server.send_P(200, "text/html", HTML);
}

} // namespace

void setup() {
  camHwInitLeds();

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Smart Home Camera 1 (Temp Stream + Remote Laptop AI) ===");

  stateMutex = xSemaphoreCreateMutex();
  g_motionJpgBuf = static_cast<uint8_t *>(ps_malloc(MAX_MOTION_JPG_BUF));
  if (!g_motionJpgBuf) {
    g_motionJpgBuf = static_cast<uint8_t *>(malloc(MAX_MOTION_JPG_BUF));
  }
  camHwInitPreview();

  StreamConfig cfg;
  loadConfigFromNvs(cfg);

  // Initialize Wi-Fi + ESP-NOW before camera sensor setup so RF calibration finishes cleanly.
  camNetInit(SmartHomeNet::CAM_SOURCE_FACE);

  bool camOk = camHwInitCamera(PIXFORMAT_JPEG,
                               toFrameSize(cfg.frameRes),
                               cfg.jpegQuality,
                               cfg.hmirror,
                               cfg.vflip);
  if (camOk) {
    camHwSetFlashLed(cfg.flashAuto ? 0 : cfg.flashDuty);
    Serial.println("[STREAM] Camera initialized successfully.");
  } else {
    Serial.println("[STREAM] ERROR: Camera initialization failed!");
  }

  int rw = 640, rh = 480;
  resolutionDims(cfg.frameRes, rw, rh);
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    g_status.cfg = cfg;
    g_status.cameraReady = camOk;
    g_status.frameWidth = rw;
    g_status.frameHeight = rh;
    xSemaphoreGive(stateMutex);
  }

  camHwStartStreamServer(STREAM_PORT);

  // Dashboard & camera preview routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/preview.jpg", HTTP_GET, handlePreview);
  server.on("/stream", HTTP_GET, []() {
    String host = server.hostHeader();
    int colon = host.indexOf(':');
    if (colon > 0) host = host.substring(0, colon);
    server.sendHeader("Location", "http://" + host + ":81/stream");
    server.send(302, "text/plain", "Redirecting to :81/stream");
  });

  // Status, configuration & manual test routes
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/test", HTTP_POST, handleTest);
  server.on("/api/enroll", HTTP_POST, handleEnroll);
  server.on("/api/enroll/cancel", HTTP_POST, handleEnrollCancel);
  server.on("/api/enroll/cancel", HTTP_GET, handleEnrollCancel);
  server.on("/api/delete", HTTP_POST, handleDelete);
  server.on("/api/clear", HTTP_POST, handleClear);

  // Remote Laptop AI feedback & synchronization API
  server.on("/api/recognition/result", HTTP_POST, handleRecognitionResult);
  server.on("/api/feedback", HTTP_POST, handleRecognitionResult);
  server.on("/api/laptop/heartbeat", HTTP_POST, handleLaptopHeartbeat);
  server.on("/api/laptop/enrolled", HTTP_POST, handleLaptopEnrolled);
  server.on("/api/enroll/ack", HTTP_POST, handleEnrollAck);
  server.on("/api/command/ack", HTTP_POST, handleCommandAck);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not Found");
  });
  server.begin();

  vTaskPrioritySet(nullptr, 1);

  if (camOk) {
    xTaskCreatePinnedToCore(camCaptureTask,
                            "cam_capture",
                            8192,
                            nullptr,
                            1,
                            nullptr,
                            1);
    Serial.println("[STREAM] Capture task + MJPEG stream ready.");
  }
}

void loop() {
  camNetLoop();
  server.handleClient();
  delay(8);
}
