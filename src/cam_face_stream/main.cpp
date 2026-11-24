// ESP32-S3 face camera — high-FPS stream + remote laptop AI feedback
// Focuses exclusively on:
//   1. Real-time hardware MJPEG live video streaming (:81/stream & /preview.jpg)
//   2. Darkness + Motion Auto-Flash (5-second auto-off)
//   3. Accepting laptop recognition feedback (/api/recognition/result)
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
  uint8_t frameRes = 0;
  uint8_t jpegQuality = 16;
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
  cfg.frameRes = static_cast<uint8_t>(constrain(prefs.getUChar("res", cfg.frameRes), 0, 2));
  cfg.jpegQuality = static_cast<uint8_t>(constrain(prefs.getUChar("qual", cfg.jpegQuality), 8, 30));
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
  unsigned int framesToDiscard = 0;
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
          if (s->set_framesize(s, toFrameSize(nextCfg.frameRes)) == 0) {
            // The DMA queue may still contain frames captured at the old size.
            framesToDiscard = 3;
          } else {
            Serial.println("[STREAM] Sensor rejected resolution change");
            nextCfg.frameRes = capCfg.frameRes;
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

    if (framesToDiscard > 0) {
      --framesToDiscard;
      esp_camera_fb_return(fb);
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
      bool shouldSampleMotion = capCfg.flashAuto && g_motionJpgBuf &&
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
      bool shouldSampleMotion = capCfg.flashAuto &&
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

    if (triggerHub && !camNetHubConnected()) {
      hubAction = "HUB_OFFLINE";
    } else if (triggerHub && (forceDispatch || g_lastRecognizedSentMs == 0 ||
                              now - g_lastRecognizedSentMs >= RECOGNIZED_COOLDOWN_MS)) {
      g_lastRecognizedSentMs = now;
      hubDispatched = camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED,
                                      static_cast<int16_t>(matchId),
                                      confPct,
                                      cleanName);
      hubAction = "DOOR_OPEN";
      Serial.printf("[REMOTE] RECOGNIZED id=%d name=\"%s\" sim=%.2f -> Hub Door Open (%s)\n",
                    matchId, cleanName, sim, hubDispatched ? "OK" : "ERR");
    } else if (triggerHub) {
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
      if (triggerHub && activeCfg.armed && !camNetHubConnected()) {
        hubAction = "HUB_OFFLINE";
      } else if (triggerHub && activeCfg.armed &&
                 (forceDispatch || g_lastUnknownSentMs == 0 ||
                  now - g_lastUnknownSentMs >= UNKNOWN_COOLDOWN_MS)) {
        g_lastUnknownSentMs = now;
        hubDispatched = camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN,
                                        -1,
                                        confPct,
                                        "UNKNOWN");
        hubAction = "INTRUDER_ALARM";
        Serial.printf("[REMOTE] UNKNOWN INTRUDER (streak=%d sim=%.2f) -> Hub Alarm (%s)\n",
                      g_unknownStreak, sim, hubDispatched ? "OK" : "ERR");
      } else if (triggerHub) {
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
  bool triggerHub = true;
  readBoolParam(body, "triggerHub", triggerHub);

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

  if (ok && newId >= 0 && triggerHub && camNetHubConnected()) {
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
    StreamConfig next = g_status.cfg;
    if (server.hasArg("armed")) next.armed = (server.arg("armed").toInt() != 0);
    if (server.hasArg("uconf")) next.unknownConfirm = constrain(server.arg("uconf").toInt(), 1, 5);
    if (server.hasArg("flash")) next.flashDuty = static_cast<uint8_t>(constrain(server.arg("flash").toInt(), 0, 180));
    if (server.hasArg("flash_auto")) next.flashAuto = (server.arg("flash_auto").toInt() != 0);
    if (server.hasArg("hmir")) next.hmirror = (server.arg("hmir").toInt() != 0);
    if (server.hasArg("vflip")) next.vflip = (server.arg("vflip").toInt() != 0);
    if (server.hasArg("res")) next.frameRes = static_cast<uint8_t>(constrain(server.arg("res").toInt(), 0, 2));
    if (server.hasArg("qual")) next.jpegQuality = static_cast<uint8_t>(constrain(server.arg("qual").toInt(), 8, 30));
    if (next.frameRes > 0 && !psramFound()) {
      xSemaphoreGive(stateMutex);
      server.send(409, "application/json", "{\"error\":\"Higher resolutions require PSRAM\"}");
      return;
    }
    g_status.cfg = next;
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

void redirectToStream() {
  String host = server.hostHeader();
  int colon = host.indexOf(':');
  if (colon > 0) host = host.substring(0, colon);
  if (host.isEmpty()) host = camNetIpString();
  server.sendHeader("Location", "http://" + host + ":" + String(STREAM_PORT) + "/stream");
  server.send(302, "text/plain", "Redirecting to the camera stream");
}

} // namespace

void setup() {
  camHwInitLeds();

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Smart Home Camera 1 (Temp Stream + Remote Laptop AI) ===");

  stateMutex = xSemaphoreCreateMutex();
  camHwInitPreview();
  if (camHwFlashAvailable() && psramFound()) {
    g_motionJpgBuf = static_cast<uint8_t *>(ps_malloc(MAX_MOTION_JPG_BUF));
  }

  StreamConfig cfg;
  loadConfigFromNvs(cfg);
  if (!psramFound()) {
    cfg.frameRes = 0;
    Serial.println("[STREAM] PSRAM unavailable; using QVGA");
  }

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

  server.on("/", HTTP_GET, redirectToStream);
  server.on("/stream", HTTP_GET, redirectToStream);
  server.on("/preview.jpg", HTTP_GET, handlePreview);

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
