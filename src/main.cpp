// Smart Home Automation, Safety & Access System (ESP32 Main Hub)
// Features:
// 1- Flame detection (analog flame sensor AO -> GPIO34)
// 2- Smoke & combustible gas detection (analog MQ-2 AO -> GPIO35)
// 3- Motion detection (PIR digital sensor -> GPIO27)
// 4- Non-blocking ultrasonic obstacle detection (HC-SR04 -> TRIG: GPIO32, ECHO: GPIO33)
// 5- Dual-servo car gate control (Left: GPIO18, Right: GPIO19)
// 6- Single-servo pedestrian door control (GPIO26)
// 7- High-decibel swept-frequency siren alarm (passive buzzer -> GPIO25 via LEDC)
// 8- High-power Wi-Fi SoftAP ("SmartHome-Net" 192.168.4.1) + optional Home Wi-Fi STA uplink
// 9- Wireless ESP-NOW receiver for 2x ESP32-CAM nodes:
//    - Camera 1 (Face Recognition @ 192.168.4.101) -> triggers door servo or intruder siren
//    - Camera 2 (Plate OCR @ 192.168.4.102)        -> triggers car gate servos or intruder siren
// 10- Modern Responsive Web Admin Panel:
//    - Live side-by-side video feeds from both cameras (direct or Hub proxy)
//    - Real-time gauge meters and status badges for all sensors
//    - Interactive gate, door, and siren controls
//    - Persistent settings in NVS (Preferences) for thresholds, timers, and Wi-Fi credentials
//    - Live rotating system event log

#include <Arduino.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <sys/time.h>
#include <time.h>

#include "smart_home_json.h"
#include "smart_home_protocol.h"

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
constexpr int FIRE_SENSOR_PIN     = 34; // Flame AO (ADC1, input-only)
constexpr int MQ2_SENSOR_PIN      = 35; // MQ-2 AO (ADC1, input-only)
constexpr int PIR_SENSOR_PIN      = 27; // PIR digital OUT (active-HIGH)
constexpr int ULTRASONIC_TRIG_PIN = 32; // HC-SR04 TRIG (digital OUT)
constexpr int ULTRASONIC_ECHO_PIN = 33; // HC-SR04 ECHO (digital IN; level shifted 5V->3.3V)
constexpr int BUZZER_PIN          = 25; // Siren buzzer output (LEDC tone)
constexpr int GATE_SERVO_1_PIN    = 18; // Left car gate servo
constexpr int GATE_SERVO_2_PIN    = 19; // Right car gate servo
constexpr int DOOR_SERVO_3_PIN    = 26; // Pedestrian door servo
constexpr int LIGHT_PIN           = 4;  // Smart Light transistor driver (digital OUT)

// ---------------------------------------------------------------------------
// Servos & PWM Configuration
// ---------------------------------------------------------------------------
constexpr int SERVO_PWM_FREQ_HZ   = 50;
constexpr int SERVO_PWM_RES_BITS  = 16;
constexpr int SERVO_MIN_US        = 500;
constexpr int SERVO_MAX_US        = 2500;

constexpr int SERVO1_CLOSED_ANGLE = 0;
constexpr int SERVO1_OPEN_ANGLE   = 160; // Safety limit (reduced from 180° to avoid binding/stalling)
constexpr int SERVO2_CLOSED_ANGLE = 0;
constexpr int SERVO2_OPEN_ANGLE   = 160; // Safety limit (reduced from 180° to avoid binding/stalling)
constexpr int DOOR3_CLOSED_ANGLE  = 0;
constexpr int DOOR3_OPEN_ANGLE    = 160; // Safety limit (reduced from 180° to avoid binding/stalling)

constexpr int SERVO_STEP_DEG      = 2;
constexpr unsigned long SERVO_STEP_INTERVAL_MS = 20;

// ---------------------------------------------------------------------------
// Siren LEDC (Channel 4 avoids conflict with ESP32Servo timers 0-1)
// ---------------------------------------------------------------------------
constexpr int SIREN_LEDC_CHANNEL  = 4;
constexpr int SIREN_RESOLUTION_BITS = 8;
constexpr int SIREN_LOW_HZ        = 700;
constexpr int SIREN_HIGH_HZ       = 1800;
constexpr int SIREN_STEP_HZ       = 60;
constexpr unsigned long SIREN_STEP_MS = 20;

// ---------------------------------------------------------------------------
// Ultrasonic Timing Constants
// ---------------------------------------------------------------------------
constexpr unsigned long ULTRASONIC_INTERVAL_MS = 60;
constexpr unsigned long ULTRASONIC_TIMEOUT_US  = 20000; // ~3.4m timeout
constexpr int ULTRASONIC_CONFIRM_READS         = 2;
constexpr unsigned long ULTRASONIC_ALARM_HOLD_MS = 3000;
constexpr unsigned long CAM_ONLINE_TIMEOUT_MS  = 15000;

// ---------------------------------------------------------------------------
// RFID Reader (MFRC522 / RC522) on SPI — non-conflicting pins for the Hub
//   SS/SDA  -> GPIO5   SCK  -> GPIO14   MOSI -> GPIO13   MISO -> GPIO12   RST -> GPIO22
// ---------------------------------------------------------------------------
constexpr uint8_t RFID_SS_PIN   = 5;
constexpr uint8_t RFID_RST_PIN  = 22;
constexpr uint8_t RFID_SCK_PIN  = 14;
constexpr uint8_t RFID_MOSI_PIN = 13;
constexpr uint8_t RFID_MISO_PIN = 12;

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
unsigned long lastRfidPrintMs = 0;
constexpr unsigned long RFID_PRINT_INTERVAL_MS = 2000;
String lastRfidUid;

// Accepted RFID card UID — triggers the car gate (Servos 1 & 2 to 160°) when presented.
constexpr char ACCEPTED_RFID_UID[] = "52411902";

// ---------------------------------------------------------------------------
// Persistent Hub Configuration (Stored in NVS "hubcfg")
// ---------------------------------------------------------------------------
struct HubConfig {
  int fireThreshold        = 500;   // AO drops below this when flame detected
  int smokeThreshold       = 2000;  // AO rises above this for smoke
  float ultrasonicDistCm   = 25.0f; // Proximity trigger threshold
  unsigned long gateHoldMs = 5000;  // Duration gate stays open after recognized plate
  unsigned long doorHoldMs = 3000;  // Duration door stays open after recognized face
  unsigned long camAlarmMs = 4000;  // Intruder alarm siren duration
  bool motionTriggersSiren = true;  // Trigger buzzer on PIR motion
  bool ultraTriggersSiren  = true;  // Trigger buzzer on ultrasonic proximity
  bool sirenMuted          = false; // Global siren mute switch
  bool pirScheduleEnabled  = true;  // Filter PIR siren by night window (midnight to 7 AM)
  int pirStartHour         = 0;     // Midnight (00:00)
  int pirStartMin          = 0;
  int pirEndHour           = 7;     // 7:00 AM (07:00)
  int pirEndMin            = 0;
  char staSsid[33]         = "";    // Optional Home Wi-Fi SSID
  char staPass[65]         = "";    // Optional Home Wi-Fi Password
};

HubConfig g_cfg;
Preferences prefs;

// ---------------------------------------------------------------------------
// Rotating Event Log
// ---------------------------------------------------------------------------
constexpr size_t MAX_EVENT_LOG = 16;
struct LogItem {
  uint32_t uptimeSec;
  char timeStr[12];
  char type[12]; // "danger", "warning", "success", "info"
  char msg[72];
};

LogItem eventLog[MAX_EVENT_LOG];
size_t eventLogCount = 0;
size_t eventLogHead = 0;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

void formatUptime(uint32_t sec, char *out, size_t outSize) {
  uint32_t h = sec / 3600;
  uint32_t m = (sec % 3600) / 60;
  uint32_t s = sec % 60;
  snprintf(out, outSize, "%02u:%02u:%02u", h, m, s);
}

void addEventLog(const char *type, const char *msg) {
  uint32_t sec = millis() / 1000;
  char timeBuf[12];
  formatUptime(sec, timeBuf, sizeof(timeBuf));

  portENTER_CRITICAL(&logMux);
  size_t idx = (eventLogHead + eventLogCount) % MAX_EVENT_LOG;
  if (eventLogCount == MAX_EVENT_LOG) {
    eventLogHead = (eventLogHead + 1) % MAX_EVENT_LOG;
    idx = (eventLogHead + MAX_EVENT_LOG - 1) % MAX_EVENT_LOG;
  } else {
    eventLogCount++;
  }
  eventLog[idx].uptimeSec = sec;
  strncpy(eventLog[idx].timeStr, timeBuf, sizeof(eventLog[idx].timeStr) - 1);
  eventLog[idx].timeStr[sizeof(eventLog[idx].timeStr) - 1] = '\0';
  strncpy(eventLog[idx].type, type, sizeof(eventLog[idx].type) - 1);
  eventLog[idx].type[sizeof(eventLog[idx].type) - 1] = '\0';
  strncpy(eventLog[idx].msg, msg, sizeof(eventLog[idx].msg) - 1);
  eventLog[idx].msg[sizeof(eventLog[idx].msg) - 1] = '\0';
  portEXIT_CRITICAL(&logMux);

  Serial.printf("[%s] [%s] %s\n", timeBuf, type, msg);
}

// ---------------------------------------------------------------------------
// Hardware & State Variables
// ---------------------------------------------------------------------------
Servo gateServo1;
Servo gateServo2;
Servo doorServo3;
int currentServo1Angle = SERVO1_CLOSED_ANGLE;
int currentServo2Angle = SERVO2_CLOSED_ANGLE;
int currentDoor3Angle  = DOOR3_CLOSED_ANGLE;
int targetServo1Angle  = SERVO1_CLOSED_ANGLE;
int targetServo2Angle  = SERVO2_CLOSED_ANGLE;
int targetDoor3Angle   = DOOR3_CLOSED_ANGLE;
unsigned long lastServoStepMs = 0;

int sirenHz = SIREN_LOW_HZ;
int sirenDirection = 1;
unsigned long lastSirenMs = 0;
bool sirenActive = false;
bool buzzerAttached = false;
unsigned long sirenTestUntilMs = 0;

unsigned long lastUltrasonicMs = 0;
unsigned long ultrasonicAlarmUntilMs = 0;
unsigned long plateGateUntilMs = 0;
unsigned long doorOpenUntilMs = 0;
unsigned long camAlarmUntilMs = 0;

int ultrasonicConfirmCount = 0;
float lastDistanceCm = -1.0f;
bool lastUltrasonicDetected = false;
bool gateOpen = false;
bool doorOpen = false;

int lastFlameAdc = 4095;
int lastSmokeAdc = 0;
bool lastFireDetected = false;
bool lastSmokeDetected = false;
bool lastMotionDetected = false;
bool lastAlarmActive = false;
unsigned long lastPrintMs = 0;

bool lightState = false;
bool timeSynchronized = false;

// ---------------------------------------------------------------------------
// Timekeeping & Smart Lighting Helpers
// ---------------------------------------------------------------------------
void initDefaultSystemTime() {
  struct tm tmBoot = {};
  tmBoot.tm_year = 2026 - 1900;
  tmBoot.tm_mon  = 0;  // January
  tmBoot.tm_mday = 1;
  tmBoot.tm_hour = 12; // 12:00 PM default as requested
  tmBoot.tm_min  = 0;
  tmBoot.tm_sec  = 0;
  time_t bootEpoch = mktime(&tmBoot);
  struct timeval tvBoot = { .tv_sec = bootEpoch, .tv_usec = 0 };
  settimeofday(&tvBoot, nullptr);
  timeSynchronized = false;
}

void setSystemEpochTime(uint32_t epochSec) {
  struct timeval tv = { .tv_sec = static_cast<time_t>(epochSec), .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  timeSynchronized = true;

  char timeBuf[32];
  time_t nowTime = time(nullptr);
  struct tm ti;
  localtime_r(&nowTime, &ti);
  strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &ti);
  char msg[72];
  snprintf(msg, sizeof(msg), "System clock updated to %s", timeBuf);
  addEventLog("info", msg);
}

bool isPirScheduleActive() {
  time_t now = time(nullptr);
  struct tm ti;
  localtime_r(&now, &ti);
  int curMinutes = ti.tm_hour * 60 + ti.tm_min;
  int startMinutes = g_cfg.pirStartHour * 60 + g_cfg.pirStartMin;
  int endMinutes = g_cfg.pirEndHour * 60 + g_cfg.pirEndMin;

  if (startMinutes == endMinutes) {
    return true; // 24-hr active
  }
  if (startMinutes < endMinutes) {
    // Same-day window, e.g. 00:00 (0m) to 07:00 (420m)
    return (curMinutes >= startMinutes && curMinutes < endMinutes);
  } else {
    // Overnight wrap-around window, e.g. 23:00 to 07:00
    return (curMinutes >= startMinutes || curMinutes < endMinutes);
  }
}

void setLightState(bool on) {
  if (lightState == on) return;
  lightState = on;
  digitalWrite(LIGHT_PIN, lightState ? HIGH : LOW);
  addEventLog("info", lightState ? "Smart Light turned ON (GPIO4)" : "Smart Light turned OFF (GPIO4)");
}

// Non-blocking interrupt-driven HC-SR04 state
portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;
volatile int64_t echoRiseUs = 0;
volatile int64_t echoPulseUs = 0;
volatile bool echoMeasuring = false;
volatile bool echoComplete = false;
bool echoWaiting = false;
int64_t echoTriggerUs = 0;

// Camera Telemetry
struct CamNodeTelemetry {
  bool seen = false;
  unsigned long lastSeenMs = 0;
  unsigned long lastEventMs = 0;
  uint8_t lastEvent = 0;
  int16_t lastMatchId = -1;
  uint8_t lastConfidence = 0;
  uint32_t lastSeq = 0;
  char lastLabel[24] = "none";
};

CamNodeTelemetry faceCamState;
CamNodeTelemetry plateCamState;

// ESP-NOW Rx Queue
struct IncomingCamPacket {
  SmartHomeNet::CamEventPacket pkt;
  uint8_t srcMac[6];
};

constexpr size_t RX_QUEUE_CAP = 8;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
IncomingCamPacket rxQueue[RX_QUEUE_CAP];
volatile size_t rxHead = 0;
volatile size_t rxTail = 0;
volatile uint32_t rxDropped = 0;
bool espNowReady = false;

WebServer server(80);

// ---------------------------------------------------------------------------
// NVS Configuration Management
// ---------------------------------------------------------------------------
void loadConfigFromNvs() {
  prefs.begin("hubcfg", true);
  g_cfg.fireThreshold        = prefs.getInt("fire_th", 500);
  g_cfg.smokeThreshold       = prefs.getInt("smoke_th", 2000);
  g_cfg.ultrasonicDistCm     = prefs.getFloat("ultra_cm", 25.0f);
  g_cfg.gateHoldMs           = prefs.getULong("gate_ms", 5000);
  g_cfg.doorHoldMs           = prefs.getULong("door_ms", 3000);
  g_cfg.camAlarmMs           = prefs.getULong("cam_ms", 4000);
  g_cfg.motionTriggersSiren  = prefs.getBool("mot_siren", true);
  g_cfg.ultraTriggersSiren   = prefs.getBool("ult_siren", true);
  g_cfg.sirenMuted           = prefs.getBool("muted", false);
  g_cfg.pirScheduleEnabled  = prefs.getBool("pir_sched", true);
  g_cfg.pirStartHour         = prefs.getInt("pir_sh", 0);
  g_cfg.pirStartMin          = prefs.getInt("pir_sm", 0);
  g_cfg.pirEndHour           = prefs.getInt("pir_eh", 7);
  g_cfg.pirEndMin            = prefs.getInt("pir_em", 0);

  String savedSsid = prefs.getString("sta_ssid", "");
  String savedPass = prefs.getString("sta_pass", "");
  strncpy(g_cfg.staSsid, savedSsid.c_str(), sizeof(g_cfg.staSsid) - 1);
  g_cfg.staSsid[sizeof(g_cfg.staSsid) - 1] = '\0';
  strncpy(g_cfg.staPass, savedPass.c_str(), sizeof(g_cfg.staPass) - 1);
  g_cfg.staPass[sizeof(g_cfg.staPass) - 1] = '\0';
  prefs.end();

  g_cfg.fireThreshold = constrain(g_cfg.fireThreshold, 100, 3900);
  g_cfg.smokeThreshold = constrain(g_cfg.smokeThreshold, 200, 4000);
  g_cfg.ultrasonicDistCm = constrain(g_cfg.ultrasonicDistCm, 5.0f, 150.0f);
  g_cfg.gateHoldMs = constrain(g_cfg.gateHoldMs, 1000UL, 30000UL);
  g_cfg.doorHoldMs = constrain(g_cfg.doorHoldMs, 1000UL, 30000UL);
  g_cfg.camAlarmMs = constrain(g_cfg.camAlarmMs, 1000UL, 20000UL);
  g_cfg.pirStartHour = constrain(g_cfg.pirStartHour, 0, 23);
  g_cfg.pirStartMin = constrain(g_cfg.pirStartMin, 0, 59);
  g_cfg.pirEndHour = constrain(g_cfg.pirEndHour, 0, 23);
  g_cfg.pirEndMin = constrain(g_cfg.pirEndMin, 0, 59);
}

void saveConfigToNvs() {
  prefs.begin("hubcfg", false);
  prefs.putInt("fire_th", g_cfg.fireThreshold);
  prefs.putInt("smoke_th", g_cfg.smokeThreshold);
  prefs.putFloat("ultra_cm", g_cfg.ultrasonicDistCm);
  prefs.putULong("gate_ms", g_cfg.gateHoldMs);
  prefs.putULong("door_ms", g_cfg.doorHoldMs);
  prefs.putULong("cam_ms", g_cfg.camAlarmMs);
  prefs.putBool("mot_siren", g_cfg.motionTriggersSiren);
  prefs.putBool("ult_siren", g_cfg.ultraTriggersSiren);
  prefs.putBool("muted", g_cfg.sirenMuted);
  prefs.putBool("pir_sched", g_cfg.pirScheduleEnabled);
  prefs.putInt("pir_sh", g_cfg.pirStartHour);
  prefs.putInt("pir_sm", g_cfg.pirStartMin);
  prefs.putInt("pir_eh", g_cfg.pirEndHour);
  prefs.putInt("pir_em", g_cfg.pirEndMin);
  prefs.putString("sta_ssid", g_cfg.staSsid);
  prefs.putString("sta_pass", g_cfg.staPass);
  prefs.end();
}

// ---------------------------------------------------------------------------
// ESP-NOW Receive Callback (Core 2.0.x API)
// ---------------------------------------------------------------------------
void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (!data || len != static_cast<int>(sizeof(SmartHomeNet::CamEventPacket))) {
    return;
  }
  SmartHomeNet::CamEventPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != SmartHomeNet::PACKET_MAGIC) {
    return;
  }
  pkt.label[sizeof(pkt.label) - 1] = '\0';

  portENTER_CRITICAL(&rxMux);
  size_t nextHead = (rxHead + 1) % RX_QUEUE_CAP;
  if (nextHead == rxTail) {
    rxDropped++;
  } else {
    rxQueue[rxHead].pkt = pkt;
    if (mac_addr) {
      memcpy(rxQueue[rxHead].srcMac, mac_addr, 6);
    } else {
      memset(rxQueue[rxHead].srcMac, 0, 6);
    }
    rxHead = nextHead;
  }
  portEXIT_CRITICAL(&rxMux);
}

bool popCamPacket(IncomingCamPacket &out) {
  bool hasItem = false;
  portENTER_CRITICAL(&rxMux);
  if (rxTail != rxHead) {
    out = rxQueue[rxTail];
    rxTail = (rxTail + 1) % RX_QUEUE_CAP;
    hasItem = true;
  }
  portEXIT_CRITICAL(&rxMux);
  return hasItem;
}

// ---------------------------------------------------------------------------
// Servo Helper Functions
// ---------------------------------------------------------------------------
int servoAngleToPulseUs(int angle) {
  // Clamped to 160° maximum to maintain safety margin against motor stall and gear breakage
  int clamped = constrain(angle, 0, 160);
  long span = static_cast<long>(SERVO_MAX_US - SERVO_MIN_US);
  return SERVO_MIN_US + static_cast<int>((clamped * span + 90L) / 180L);
}

bool attachGateServo(Servo &servo, int pin, int initialAngle, const char *label) {
  servo.setPeriodHertz(SERVO_PWM_FREQ_HZ);
  servo.attach(pin, SERVO_MIN_US, SERVO_MAX_US);
  if (!servo.attached()) {
    Serial.printf("WARNING: Failed to attach %s on GPIO%d\n", label, pin);
    return false;
  }
  servo.setTimerWidth(SERVO_PWM_RES_BITS);
  servo.writeMicroseconds(servoAngleToPulseUs(initialAngle));
  return true;
}

int stepServoAngle(int current, int target, int maxStep) {
  if (current < target) return min(current + maxStep, target);
  if (current > target) return max(current - maxStep, target);
  return current;
}

void setGateOpen(bool open) {
  if (gateOpen == open) return;
  gateOpen = open;
  targetServo1Angle = gateOpen ? SERVO1_OPEN_ANGLE : SERVO1_CLOSED_ANGLE;
  targetServo2Angle = gateOpen ? SERVO2_OPEN_ANGLE : SERVO2_CLOSED_ANGLE;
  addEventLog("info", gateOpen ? "Car gate opening (Servos 1 & 2)" : "Car gate closing");
}

void setDoorOpen(bool open) {
  if (doorOpen == open) return;
  doorOpen = open;
  targetDoor3Angle = doorOpen ? DOOR3_OPEN_ANGLE : DOOR3_CLOSED_ANGLE;
  addEventLog("info", doorOpen ? "Face door unlocked (Servo 3)" : "Face door locked");
}

void updateAllServos(unsigned long now) {
  if (currentServo1Angle == targetServo1Angle &&
      currentServo2Angle == targetServo2Angle &&
      currentDoor3Angle == targetDoor3Angle) {
    return;
  }
  if (now - lastServoStepMs < SERVO_STEP_INTERVAL_MS) {
    return;
  }
  lastServoStepMs = now;

  int nextAngle1 = stepServoAngle(currentServo1Angle, targetServo1Angle, SERVO_STEP_DEG);
  int nextAngle2 = stepServoAngle(currentServo2Angle, targetServo2Angle, SERVO_STEP_DEG);
  int nextAngle3 = stepServoAngle(currentDoor3Angle, targetDoor3Angle, SERVO_STEP_DEG);

  if (nextAngle1 != currentServo1Angle) {
    currentServo1Angle = nextAngle1;
    if (gateServo1.attached()) {
      gateServo1.writeMicroseconds(servoAngleToPulseUs(currentServo1Angle));
    }
  }
  if (nextAngle2 != currentServo2Angle) {
    currentServo2Angle = nextAngle2;
    if (gateServo2.attached()) {
      gateServo2.writeMicroseconds(servoAngleToPulseUs(currentServo2Angle));
    }
  }
  if (nextAngle3 != currentDoor3Angle) {
    currentDoor3Angle = nextAngle3;
    if (doorServo3.attached()) {
      doorServo3.writeMicroseconds(servoAngleToPulseUs(currentDoor3Angle));
    }
  }
}

// ---------------------------------------------------------------------------
// Non-Blocking Ultrasonic Sensor
// ---------------------------------------------------------------------------
void IRAM_ATTR onUltrasonicEchoChange() {
  int level = digitalRead(ULTRASONIC_ECHO_PIN);
  int64_t nowUs = esp_timer_get_time();

  portENTER_CRITICAL_ISR(&echoMux);
  if (level == HIGH) {
    echoRiseUs = nowUs;
    echoMeasuring = true;
    echoComplete = false;
  } else if (echoMeasuring) {
    echoPulseUs = nowUs - echoRiseUs;
    echoMeasuring = false;
    echoComplete = true;
  }
  portEXIT_CRITICAL_ISR(&echoMux);
}

void recordUltrasonicSample(float distCm, unsigned long now) {
  lastDistanceCm = distCm;
  bool objectNear = (lastDistanceCm > 0.0f && lastDistanceCm <= g_cfg.ultrasonicDistCm);
  if (objectNear) {
    if (ultrasonicConfirmCount < ULTRASONIC_CONFIRM_READS) {
      ultrasonicConfirmCount++;
    }
    if (ultrasonicConfirmCount >= ULTRASONIC_CONFIRM_READS) {
      ultrasonicAlarmUntilMs = now + ULTRASONIC_ALARM_HOLD_MS;
    }
  } else {
    ultrasonicConfirmCount = 0;
  }
}

void updateUltrasonicSensor(unsigned long now) {
  int64_t nowUs = esp_timer_get_time();

  if (echoWaiting) {
    bool completed = false;
    int64_t pulseUs = 0;
    portENTER_CRITICAL(&echoMux);
    if (echoComplete) {
      completed = true;
      pulseUs = echoPulseUs;
      echoComplete = false;
    }
    portEXIT_CRITICAL(&echoMux);

    if (completed) {
      echoWaiting = false;
      float distCm = (pulseUs > 0 && pulseUs <= static_cast<int64_t>(ULTRASONIC_TIMEOUT_US))
                         ? (static_cast<float>(pulseUs) * 0.0343f) / 2.0f
                         : -1.0f;
      recordUltrasonicSample(distCm, now);
    } else if (nowUs - echoTriggerUs > static_cast<int64_t>(ULTRASONIC_TIMEOUT_US)) {
      portENTER_CRITICAL(&echoMux);
      echoMeasuring = false;
      echoComplete = false;
      portEXIT_CRITICAL(&echoMux);
      echoWaiting = false;
      recordUltrasonicSample(-1.0f, now);
    }
  }

  if (!echoWaiting && (now - lastUltrasonicMs >= ULTRASONIC_INTERVAL_MS)) {
    lastUltrasonicMs = now;
    if (digitalRead(ULTRASONIC_ECHO_PIN) == HIGH) {
      recordUltrasonicSample(-1.0f, now);
    } else {
      portENTER_CRITICAL(&echoMux);
      echoMeasuring = false;
      echoComplete = false;
      portEXIT_CRITICAL(&echoMux);

      digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

      echoTriggerUs = esp_timer_get_time();
      echoWaiting = true;
    }
  }

  if (ultrasonicAlarmUntilMs != 0 && static_cast<long>(ultrasonicAlarmUntilMs - now) > 0) {
    lastUltrasonicDetected = true;
  } else {
    ultrasonicAlarmUntilMs = 0;
    lastUltrasonicDetected = false;
  }
}

void updateServosControl(unsigned long now) {
  if (gateOpen) {
    if (plateGateUntilMs == 0 || static_cast<long>(plateGateUntilMs - now) <= 0) {
      plateGateUntilMs = 0;
      setGateOpen(false);
    }
  }

  if (doorOpen) {
    if (doorOpenUntilMs == 0 || static_cast<long>(doorOpenUntilMs - now) <= 0) {
      doorOpenUntilMs = 0;
      setDoorOpen(false);
    }
  }

  updateAllServos(now);
}

// ---------------------------------------------------------------------------
// Siren & Buzzer Control
// ---------------------------------------------------------------------------
void startSiren(unsigned long now) {
  if (sirenActive) return;
  sirenHz = SIREN_LOW_HZ;
  sirenDirection = 1;
  lastSirenMs = now;
  if (buzzerAttached && !g_cfg.sirenMuted) {
    ledcWriteTone(SIREN_LEDC_CHANNEL, sirenHz);
  }
  sirenActive = true;
}

void stopSiren() {
  if (!sirenActive) return;
  if (buzzerAttached) {
    ledcWriteTone(SIREN_LEDC_CHANNEL, 0);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
  sirenHz = SIREN_LOW_HZ;
  sirenDirection = 1;
  sirenActive = false;
}

void updateSiren(unsigned long now) {
  if (!buzzerAttached) return;
  if (g_cfg.sirenMuted) {
    ledcWriteTone(SIREN_LEDC_CHANNEL, 0);
    return;
  }
  if (now - lastSirenMs < SIREN_STEP_MS) return;
  lastSirenMs = now;

  sirenHz += sirenDirection * SIREN_STEP_HZ;
  if (sirenHz >= SIREN_HIGH_HZ) {
    sirenHz = SIREN_HIGH_HZ;
    sirenDirection = -1;
  } else if (sirenHz <= SIREN_LOW_HZ) {
    sirenHz = SIREN_LOW_HZ;
    sirenDirection = 1;
  }
  ledcWriteTone(SIREN_LEDC_CHANNEL, sirenHz);
}

const char *eventTypeName(uint8_t ev) {
  switch (ev) {
    case SmartHomeNet::CAM_EVENT_RECOGNIZED: return "RECOGNIZED";
    case SmartHomeNet::CAM_EVENT_UNKNOWN:    return "UNKNOWN";
    case SmartHomeNet::CAM_EVENT_HEARTBEAT:  return "HEARTBEAT";
    default:                                 return "NONE";
  }
}

// ---------------------------------------------------------------------------
// Camera Event Dispatcher
// ---------------------------------------------------------------------------
void handleCamEvent(const IncomingCamPacket &item, unsigned long now) {
  const auto &pkt = item.pkt;
  CamNodeTelemetry *telemetry = nullptr;
  const char *srcName = "UNKNOWN";

  if (pkt.source == SmartHomeNet::CAM_SOURCE_FACE) {
    telemetry = &faceCamState;
    srcName = "FACE-CAM";
  } else if (pkt.source == SmartHomeNet::CAM_SOURCE_PLATE) {
    // Car plate camera is disabled; RFID controls the car gate
    return;
  } else {
    return;
  }

  telemetry->seen = true;
  telemetry->lastSeenMs = now;

  if (pkt.event == SmartHomeNet::CAM_EVENT_HEARTBEAT) {
    return;
  }

  telemetry->lastEventMs = now;
  telemetry->lastEvent = pkt.event;
  telemetry->lastMatchId = pkt.matchId;
  telemetry->lastConfidence = pkt.confidence;
  telemetry->lastSeq = pkt.seq;
  strncpy(telemetry->lastLabel, pkt.label, sizeof(telemetry->lastLabel) - 1);
  telemetry->lastLabel[sizeof(telemetry->lastLabel) - 1] = '\0';

  char logMsg[72];
  if (pkt.event == SmartHomeNet::CAM_EVENT_RECOGNIZED) {
    camAlarmUntilMs = 0; // Cancel camera alarm on valid authorization
    if (pkt.source == SmartHomeNet::CAM_SOURCE_FACE) {
      doorOpenUntilMs = now + g_cfg.doorHoldMs;
      setDoorOpen(true);
      snprintf(logMsg, sizeof(logMsg), "Face recognized: \"%s\" (%u%%) -> Door unlocked",
               telemetry->lastLabel, pkt.confidence);
      addEventLog("success", logMsg);
    }
  } else if (pkt.event == SmartHomeNet::CAM_EVENT_UNKNOWN) {
    camAlarmUntilMs = now + g_cfg.camAlarmMs;
    snprintf(logMsg, sizeof(logMsg), "INTRUDER ALERT: Unrecognized face detected!");
    addEventLog("danger", logMsg);
  }
}

// ---------------------------------------------------------------------------
// Camera Image Proxy (Forward frames from camera nodes on SoftAP to client)
// ---------------------------------------------------------------------------
void handleCamProxy(int camNum) {
  if (camNum != 1) {
    server.send(404, "text/plain", "Plate camera disabled (RFID in use)");
    return;
  }
  unsigned long now = millis();
  bool faceOnline = faceCamState.seen && (now - faceCamState.lastSeenMs <= CAM_ONLINE_TIMEOUT_MS);
  if (!faceOnline) {
    server.send(503, "text/plain", "Face camera offline (no telemetry)");
    return;
  }

  const char *host = "192.168.4.101";
  WiFiClient client;
  if (!client.connect(host, 80, 200)) {
    server.send(503, "text/plain", "Face camera unreachable");
    return;
  }

  client.print("GET /preview.jpg HTTP/1.1\r\nHost: ");
  client.print(host);
  client.print("\r\nConnection: close\r\n\r\n");

  unsigned long startMs = millis();
  while (!client.available() && (millis() - startMs < 250)) {
    delay(5);
  }
  if (!client.available()) {
    client.stop();
    server.send(504, "text/plain", "Face camera timeout");
    return;
  }

  String statusLine = client.readStringUntil('\n');
  if (!statusLine.startsWith("HTTP/1.1 200") && !statusLine.startsWith("HTTP/1.0 200")) {
    client.stop();
    server.send(502, "text/plain", "Camera HTTP error");
    return;
  }

  int contentLength = 0;
  while (client.connected() && (millis() - startMs < 500)) {
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) break; // End of HTTP headers
    if (line.startsWith("Content-Length:") || line.startsWith("content-length:")) {
      contentLength = line.substring(15).toInt();
    }
  }

  if (contentLength <= 0 || contentLength > 100000) {
    client.stop();
    server.send(502, "text/plain", "Invalid image payload");
    return;
  }

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(contentLength);
  server.send(200, "image/jpeg", "");

  WiFiClient webClient = server.client();
  uint8_t buf[1024];
  int remaining = contentLength;
  while (remaining > 0 && client.connected() && (millis() - startMs < 1200)) {
    int toRead = min(remaining, static_cast<int>(sizeof(buf)));
    int bytesRead = client.read(buf, toRead);
    if (bytesRead > 0) {
      webClient.write(buf, bytesRead);
      remaining -= bytesRead;
    } else {
      delay(1);
    }
    // Keep critical actuators and siren sweep alive during proxy transfers
    unsigned long curMs = millis();
    updateAllServos(curMs);
    updateSiren(curMs);
  }
  client.stop();
}

// ---------------------------------------------------------------------------
// Web API: Dual-Path Camera Event Endpoint (/api/cam/event)
// Receives JSON access control events from camera nodes as an HTTP backup to ESP-NOW
// ---------------------------------------------------------------------------
void handleApiCamEvent() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }
  String body = server.arg("plain");

  int source = SmartHomeNet::CAM_SOURCE_FACE;
  int event = SmartHomeNet::CAM_EVENT_HEARTBEAT;
  int matchId = -1;
  int confidence = 100;
  char label[24] = "none";

  int intVal = 0;
  if (smartHomeJsonGetInt(body, "source", intVal)) source = intVal;
  if (smartHomeJsonGetInt(body, "event", intVal)) event = intVal;
  if (smartHomeJsonGetInt(body, "matchId", intVal)) matchId = intVal;
  if (smartHomeJsonGetInt(body, "confidence", intVal)) confidence = intVal;
  smartHomeJsonGetString(body, "label", label, sizeof(label));

  SmartHomeNet::CamEventPacket pkt = {};
  pkt.magic = SmartHomeNet::PACKET_MAGIC;
  pkt.source = static_cast<uint8_t>(source);
  pkt.event = static_cast<uint8_t>(event);
  pkt.matchId = static_cast<int16_t>(matchId);
  pkt.confidence = static_cast<uint8_t>(constrain(confidence, 0, 100));
  strncpy(pkt.label, label, sizeof(pkt.label) - 1);
  pkt.label[sizeof(pkt.label) - 1] = '\0';

  IncomingCamPacket inPkt;
  inPkt.pkt = pkt;
  memset(inPkt.srcMac, 0, 6); // HTTP origin
  handleCamEvent(inPkt, millis());

  server.send(200, "application/json", "{\"status\":\"ok\",\"path\":\"http\"}");
}

// ---------------------------------------------------------------------------
// Web API: Status Endpoint (/api/status)
// ---------------------------------------------------------------------------
void handleApiStatus() {
  unsigned long now = millis();
  bool camAlarmActive = (camAlarmUntilMs != 0 && static_cast<long>(camAlarmUntilMs - now) > 0);
  bool sirenTestActive = (sirenTestUntilMs != 0 && static_cast<long>(sirenTestUntilMs - now) > 0);
  bool faceOnline = faceCamState.seen && (now - faceCamState.lastSeenMs <= CAM_ONLINE_TIMEOUT_MS);
  bool plateOnline = plateCamState.seen && (now - plateCamState.lastSeenMs <= CAM_ONLINE_TIMEOUT_MS);

  time_t nowClock = time(nullptr);
  struct tm ti;
  localtime_r(&nowClock, &ti);
  char clockStr[32];
  strftime(clockStr, sizeof(clockStr), "%Y-%m-%d %H:%M:%S", &ti);
  char timeOnly[16];
  strftime(timeOnly, sizeof(timeOnly), "%H:%M:%S", &ti);
  bool pirSchedActive = g_cfg.pirScheduleEnabled ? isPirScheduleActive() : true;

  uint32_t droppedSnapshot = 0;
  portENTER_CRITICAL(&rxMux);
  droppedSnapshot = rxDropped;
  portEXIT_CRITICAL(&rxMux);

  char uptimeBuf[16];
  formatUptime(now / 1000, uptimeBuf, sizeof(uptimeBuf));

  String json = "{";
  json += "\"uptimeSec\":" + String(now / 1000) + ",";
  json += "\"uptimeStr\":\"" + String(uptimeBuf) + "\",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";

  // System Time & Smart Light
  json += "\"lightState\":" + String(lightState ? "true" : "false") + ",";
  json += "\"pirSchedActive\":" + String(pirSchedActive ? "true" : "false") + ",";
  json += "\"time\":{";
  json += "\"clock\":\"" + String(clockStr) + "\",";
  json += "\"timeOnly\":\"" + String(timeOnly) + "\",";
  json += "\"epoch\":" + String(nowClock) + ",";
  json += "\"hour\":" + String(ti.tm_hour) + ",";
  json += "\"min\":" + String(ti.tm_min) + ",";
  json += "\"synced\":" + String(timeSynchronized ? "true" : "false");
  json += "},";

  // Safety sensors
  json += "\"flameAdc\":" + String(lastFlameAdc) + ",";
  json += "\"fire\":" + String(lastFireDetected ? "true" : "false") + ",";
  json += "\"smokeAdc\":" + String(lastSmokeAdc) + ",";
  json += "\"smoke\":" + String(lastSmokeDetected ? "true" : "false") + ",";
  json += "\"pir\":" + String(lastMotionDetected ? "true" : "false") + ",";
  json += "\"ultra\":" + String(lastUltrasonicDetected ? "true" : "false") + ",";
  json += "\"distCm\":" + String(lastDistanceCm, 1) + ",";

  // Siren and alarms
  json += "\"alarm\":" + String(lastAlarmActive || sirenTestActive ? "true" : "false") + ",";
  json += "\"camAlarm\":" + String(camAlarmActive ? "true" : "false") + ",";
  json += "\"sirenActive\":" + String(sirenActive ? "true" : "false") + ",";
  json += "\"sirenMuted\":" + String(g_cfg.sirenMuted ? "true" : "false") + ",";
  json += "\"sirenHz\":" + String((sirenActive && !g_cfg.sirenMuted) ? sirenHz : 0) + ",";

  // Servos
  json += "\"gateOpen\":" + String(gateOpen ? "true" : "false") + ",";
  json += "\"servo1\":" + String(currentServo1Angle) + ",";
  json += "\"servo2\":" + String(currentServo2Angle) + ",";
  json += "\"doorOpen\":" + String(doorOpen ? "true" : "false") + ",";
  json += "\"door3\":" + String(currentDoor3Angle) + ",";
  json += "\"rxDropped\":" + String(droppedSnapshot) + ",";

  // Wi-Fi Telemetry
  bool staConnected = (WiFi.status() == WL_CONNECTED);
  json += "\"wifi\":{";
  json += "\"apSsid\":\"" + String(SmartHomeNet::AP_SSID) + "\",";
  json += "\"apIp\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"apStations\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"staSsid\":\"" + smartHomeJsonEscape(g_cfg.staSsid) + "\",";
  json += "\"staConnected\":" + String(staConnected ? "true" : "false") + ",";
  json += "\"staIp\":\"" + (staConnected ? WiFi.localIP().toString() : String("---")) + "\",";
  json += "\"staRssi\":" + String(staConnected ? WiFi.RSSI() : 0);
  json += "},";

  // Cameras
  json += "\"faceCam\":{\"online\":" + String(faceOnline ? "true" : "false") +
          ",\"event\":\"" + String(eventTypeName(faceCamState.lastEvent)) + "\"" +
          ",\"id\":" + String(faceCamState.lastMatchId) +
          ",\"conf\":" + String(faceCamState.lastConfidence) +
          ",\"label\":\"" + smartHomeJsonEscape(faceCamState.lastLabel) + "\"" +
          ",\"ageSec\":" + String(faceCamState.seen ? ((now - faceCamState.lastEventMs) / 1000) : 9999) + "},";

  json += "\"plateCam\":{\"online\":" + String(plateOnline ? "true" : "false") +
          ",\"event\":\"" + String(eventTypeName(plateCamState.lastEvent)) + "\"" +
          ",\"id\":" + String(plateCamState.lastMatchId) +
          ",\"conf\":" + String(plateCamState.lastConfidence) +
          ",\"label\":\"" + smartHomeJsonEscape(plateCamState.lastLabel) + "\"" +
          ",\"ageSec\":" + String(plateCamState.seen ? ((now - plateCamState.lastEventMs) / 1000) : 9999) + "},";

  // RFID Telemetry
  json += "\"rfid\":{";
  json += "\"lastUid\":\"" + lastRfidUid + "\",";
  json += "\"acceptedUid\":\"" + String(ACCEPTED_RFID_UID) + "\",";
  json += "\"lastScanSec\":" + String(lastRfidPrintMs > 0 ? ((now - lastRfidPrintMs) / 1000) : 9999);
  json += "},";

  // Active Configuration
  json += "\"cfg\":{";
  json += "\"fireTh\":" + String(g_cfg.fireThreshold) + ",";
  json += "\"smokeTh\":" + String(g_cfg.smokeThreshold) + ",";
  json += "\"ultraCm\":" + String(g_cfg.ultrasonicDistCm, 1) + ",";
  json += "\"gateMs\":" + String(g_cfg.gateHoldMs) + ",";
  json += "\"doorMs\":" + String(g_cfg.doorHoldMs) + ",";
  json += "\"camMs\":" + String(g_cfg.camAlarmMs) + ",";
  json += "\"motSiren\":" + String(g_cfg.motionTriggersSiren ? "true" : "false") + ",";
  json += "\"ultSiren\":" + String(g_cfg.ultraTriggersSiren ? "true" : "false") + ",";
  json += "\"pirSched\":" + String(g_cfg.pirScheduleEnabled ? "true" : "false") + ",";
  json += "\"pirSH\":" + String(g_cfg.pirStartHour) + ",";
  json += "\"pirSM\":" + String(g_cfg.pirStartMin) + ",";
  json += "\"pirEH\":" + String(g_cfg.pirEndHour) + ",";
  json += "\"pirEM\":" + String(g_cfg.pirEndMin);
  json += "},";

  // Event log items
  json += "\"events\":[";
  portENTER_CRITICAL(&logMux);
  for (size_t i = 0; i < eventLogCount; i++) {
    size_t idx = (eventLogHead + i) % MAX_EVENT_LOG;
    if (i > 0) json += ",";
    json += "{\"time\":\"" + String(eventLog[idx].timeStr) + "\",";
    json += "\"type\":\"" + String(eventLog[idx].type) + "\",";
    json += "\"msg\":\"" + smartHomeJsonEscape(eventLog[idx].msg) + "\"}";
  }
  portEXIT_CRITICAL(&logMux);
  json += "]";

  json += "}";

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(200, "application/json", json);
}

// ---------------------------------------------------------------------------
// Web API: Action & Manual Controls (/api/control)
// ---------------------------------------------------------------------------
void handleApiControl() {
  if (!server.hasArg("action")) {
    server.send(400, "application/json", "{\"error\":\"missing action\"}");
    return;
  }
  String action = server.arg("action");
  unsigned long now = millis();

  if (action == "gate_open") {
    plateGateUntilMs = now + g_cfg.gateHoldMs;
    setGateOpen(true);
    addEventLog("info", "Manual control: Car gate opened");
  } else if (action == "gate_close") {
    plateGateUntilMs = 0;
    setGateOpen(false);
    addEventLog("info", "Manual control: Car gate closed");
  } else if (action == "door_open") {
    doorOpenUntilMs = now + g_cfg.doorHoldMs;
    setDoorOpen(true);
    addEventLog("info", "Manual control: Face door unlocked");
  } else if (action == "door_close") {
    doorOpenUntilMs = 0;
    setDoorOpen(false);
    addEventLog("info", "Manual control: Face door locked");
  } else if (action == "siren_test") {
    sirenTestUntilMs = now + 3000;
    startSiren(now);
    addEventLog("warning", "Manual control: Siren test activated (3s)");
  } else if (action == "siren_silence") {
    sirenTestUntilMs = 0;
    camAlarmUntilMs = 0;
    ultrasonicAlarmUntilMs = 0;
    stopSiren();
    addEventLog("info", "Manual control: Alarm silenced");
  } else if (action == "siren_toggle_mute") {
    g_cfg.sirenMuted = !g_cfg.sirenMuted;
    if (g_cfg.sirenMuted) {
      stopSiren();
      addEventLog("warning", "Siren buzzer MUTED by user");
    } else {
      addEventLog("info", "Siren buzzer UNMUTED by user");
    }
    saveConfigToNvs();
  } else if (action == "light_on") {
    setLightState(true);
  } else if (action == "light_off") {
    setLightState(false);
  } else if (action == "light_toggle") {
    setLightState(!lightState);
  } else {
    server.send(400, "application/json", "{\"error\":\"unknown action\"}");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------------------------------------------------------------------
// Web API: Set System Clock (/api/time)
// ---------------------------------------------------------------------------
void handleApiTime() {
  if (server.hasArg("epoch")) {
    int64_t ep = atoll(server.arg("epoch").c_str());
    if (server.hasArg("tz_offset")) {
      ep += server.arg("tz_offset").toInt();
    }
    setSystemEpochTime(static_cast<uint32_t>(ep));
    server.send(200, "application/json", "{\"ok\":true,\"synced\":true}");
    return;
  }
  if (server.hasArg("hour") && server.hasArg("min")) {
    time_t now = time(nullptr);
    struct tm ti;
    localtime_r(&now, &ti);
    ti.tm_hour = constrain(server.arg("hour").toInt(), 0, 23);
    ti.tm_min  = constrain(server.arg("min").toInt(), 0, 59);
    ti.tm_sec  = server.hasArg("sec") ? constrain(server.arg("sec").toInt(), 0, 59) : 0;
    if (server.hasArg("year")) {
      ti.tm_year = constrain(server.arg("year").toInt() - 1900, 70, 200);
    }
    if (server.hasArg("mon")) {
      ti.tm_mon = constrain(server.arg("mon").toInt() - 1, 0, 11);
    }
    if (server.hasArg("mday")) {
      ti.tm_mday = constrain(server.arg("mday").toInt(), 1, 31);
    }
    time_t newEpoch = mktime(&ti);
    setSystemEpochTime(static_cast<uint32_t>(newEpoch));
    server.send(200, "application/json", "{\"ok\":true,\"synced\":true}");
    return;
  }
  server.send(400, "application/json", "{\"error\":\"missing epoch or time parameters\"}");
}

// ---------------------------------------------------------------------------
// Web API: Update Settings (/api/config)
// ---------------------------------------------------------------------------
void handleApiConfig() {
  if (server.hasArg("fire_th"))   g_cfg.fireThreshold = constrain(server.arg("fire_th").toInt(), 100, 3900);
  if (server.hasArg("smoke_th"))  g_cfg.smokeThreshold = constrain(server.arg("smoke_th").toInt(), 200, 4000);
  if (server.hasArg("ultra_cm"))  g_cfg.ultrasonicDistCm = constrain(server.arg("ultra_cm").toFloat(), 5.0f, 150.0f);
  if (server.hasArg("gate_ms"))   g_cfg.gateHoldMs = constrain(server.arg("gate_ms").toInt(), 1000, 30000);
  if (server.hasArg("door_ms"))   g_cfg.doorHoldMs = constrain(server.arg("door_ms").toInt(), 1000, 30000);
  if (server.hasArg("cam_ms"))    g_cfg.camAlarmMs = constrain(server.arg("cam_ms").toInt(), 1000, 20000);
  if (server.hasArg("mot_siren")) g_cfg.motionTriggersSiren = (server.arg("mot_siren").toInt() != 0);
  if (server.hasArg("ult_siren")) g_cfg.ultraTriggersSiren = (server.arg("ult_siren").toInt() != 0);
  if (server.hasArg("pir_sched")) g_cfg.pirScheduleEnabled = (server.arg("pir_sched").toInt() != 0);
  if (server.hasArg("pir_sh"))    g_cfg.pirStartHour = constrain(server.arg("pir_sh").toInt(), 0, 23);
  if (server.hasArg("pir_sm"))    g_cfg.pirStartMin  = constrain(server.arg("pir_sm").toInt(), 0, 59);
  if (server.hasArg("pir_eh"))    g_cfg.pirEndHour   = constrain(server.arg("pir_eh").toInt(), 0, 23);
  if (server.hasArg("pir_em"))    g_cfg.pirEndMin    = constrain(server.arg("pir_em").toInt(), 0, 59);

  saveConfigToNvs();
  addEventLog("info", "System settings updated and saved to NVS");
  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------------------------------------------------------------------
// Web API: Update Home Wi-Fi Uplink (/api/wifi)
// ---------------------------------------------------------------------------
void handleApiWifi() {
  if (server.hasArg("disconnect") && server.arg("disconnect").toInt() == 1) {
    g_cfg.staSsid[0] = '\0';
    g_cfg.staPass[0] = '\0';
    saveConfigToNvs();
    WiFi.disconnect(true);
    addEventLog("warning", "Home Wi-Fi cleared. Running in SoftAP-only mode.");
    server.send(200, "application/json", "{\"disconnected\":true}");
    return;
  }

  if (server.hasArg("ssid")) {
    String newSsid = server.arg("ssid");
    String newPass = server.hasArg("pass") ? server.arg("pass") : "";
    strncpy(g_cfg.staSsid, newSsid.c_str(), sizeof(g_cfg.staSsid) - 1);
    g_cfg.staSsid[sizeof(g_cfg.staSsid) - 1] = '\0';
    strncpy(g_cfg.staPass, newPass.c_str(), sizeof(g_cfg.staPass) - 1);
    g_cfg.staPass[sizeof(g_cfg.staPass) - 1] = '\0';
    saveConfigToNvs();

    if (strlen(g_cfg.staSsid) > 0) {
      WiFi.mode(WIFI_AP_STA);
      WiFi.begin(g_cfg.staSsid, g_cfg.staPass);
      char msg[72];
      snprintf(msg, sizeof(msg), "Connecting to Home Wi-Fi: \"%s\"...", g_cfg.staSsid);
      addEventLog("info", msg);
    }
    server.send(200, "application/json", "{\"connecting\":true}");
    return;
  }

  server.send(400, "application/json", "{\"error\":\"missing ssid\"}");
}

// ---------------------------------------------------------------------------
// Responsive Web Admin Panel HTML/CSS/JS (Single-page App)
// ---------------------------------------------------------------------------
void handleRoot() {
  static const char HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Home Security & Access Hub</title>
<style>
  :root{
    --bg:#0b1120;--surface:#1e293b;--surface2:#334155;--border:#475569;
    --text:#f8fafc;--muted:#94a3b8;--primary:#3b82f6;--primary-hover:#2563eb;
    --success:#10b981;--warning:#f59e0b;--danger:#ef4444;--danger-bg:#7f1d1d;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--text);padding:14px;line-height:1.45}
  .wrap{max-width:1120px;margin:0 auto}

  /* Header Bar */
  header{background:var(--surface);border:1px solid var(--border);border-radius:12px;padding:12px 16px;margin-bottom:14px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px}
  .h-title{display:flex;align-items:center;gap:10px}
  .h-title h1{font-size:1.25rem;font-weight:700;letter-spacing:-.02em}
  .h-status{display:flex;gap:8px;align-items:center;flex-wrap:wrap}

  /* Badges */
  .badge{display:inline-flex;align-items:center;gap:5px;padding:3px 10px;border-radius:999px;font-size:.76rem;font-weight:700;text-transform:uppercase;letter-spacing:.03em}
  .b-ok{background:#064e3b;color:#6ee7b7;border:1px solid #059669}
  .b-warn{background:#78350f;color:#fde68a;border:1px solid #d97706}
  .b-bad{background:#7f1d1d;color:#fca5a5;border:1px solid #dc2626;animation:pulse 1.2s infinite}
  .b-off{background:#334155;color:#cbd5e1;border:1px solid #64748b}

  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.6}}

  /* Grid Layouts */
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px;margin-bottom:14px}
  .grid-2col{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:14px;margin-bottom:14px}
  .card{background:var(--surface);border:1px solid var(--border);border-radius:12px;padding:14px}
  .card-title{font-size:.95rem;font-weight:700;color:var(--muted);text-transform:uppercase;letter-spacing:.05em;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center}

  /* Camera Viewports */
  .cam-box{position:relative;background:#020617;border-radius:10px;border:1px solid var(--border);overflow:hidden;aspect-ratio:4/3;display:flex;align-items:center;justify-content:center}
  .cam-img{width:100%;height:100%;object-fit:cover;display:block}
  .cam-overlay{position:absolute;top:8px;left:8px;right:8px;display:flex;justify-content:space-between;align-items:center;pointer-events:none}
  .cam-overlay>*{pointer-events:auto}
  .cam-meta{margin:10px 0;font-size:.88rem}
  .cam-actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}

  /* Key-Value Rows */
  .row{display:flex;justify-content:space-between;align-items:center;padding:7px 0;border-bottom:1px solid var(--surface2);font-size:.9rem}
  .row:last-child{border-bottom:0}
  .k{color:var(--muted)}
  .v{font-weight:600}

  /* Progress / Gauge Bars */
  .bar-bg{height:8px;background:var(--surface2);border-radius:999px;overflow:hidden;margin-top:4px}
  .bar-fill{height:100%;background:var(--primary);border-radius:999px;transition:width .3s}
  .bar-danger{background:var(--danger)}
  .bar-warn{background:var(--warning)}

  /* Buttons */
  button,.btn{background:var(--primary);color:#fff;border:0;border-radius:8px;padding:7px 14px;font-size:.85rem;font-weight:600;cursor:pointer;text-decoration:none;display:inline-flex;align-items:center;gap:6px;transition:all .15s}
  button:hover,.btn:hover{background:var(--primary-hover)}
  button.danger{background:var(--danger)}
  button.danger:hover{background:#dc2626}
  button.warning{background:var(--warning);color:#0f172a}
  button.warning:hover{background:#d97706}
  button.secondary{background:var(--surface2);color:var(--text)}
  button.secondary:hover{background:var(--border)}

  /* Form Elements */
  input[type=text],input[type=password],input[type=number],select{background:#0f172a;border:1px solid var(--border);border-radius:6px;color:var(--text);padding:6px 10px;font-size:.88rem}
  input[type=number]{width:90px}
  .form-row{display:flex;align-items:center;justify-content:space-between;padding:8px 0;gap:8px}

  /* Event Log Console */
  .log-box{background:#020617;border:1px solid var(--surface2);border-radius:8px;padding:10px;height:180px;overflow-y:auto;font-family:monospace;font-size:.82rem}
  .log-line{padding:3px 0;border-bottom:1px solid #1e293b;display:flex;gap:8px}
  .log-time{color:var(--muted);min-width:65px}
  .log-msg{flex:1;word-break:break-all}
  .c-danger{color:#f87171}
  .c-warning{color:#fbbf24}
  .c-success{color:#34d399}
  .c-info{color:#93c5fd}

  /* Accordion Settings */
  details{background:var(--surface);border:1px solid var(--border);border-radius:12px;padding:12px 16px;margin-bottom:14px}
  summary{font-weight:700;cursor:pointer;color:var(--muted);text-transform:uppercase;letter-spacing:.05em;display:flex;justify-content:space-between;align-items:center}
  summary:hover{color:var(--text)}
</style>
</head>
<body>
<div class="wrap">

  <!-- Header -->
  <header>
    <div class="h-title">
      <span style="font-size:1.6rem">🛡️</span>
      <div>
        <h1>Smart Home Hub Admin</h1>
        <div style="font-size:.78rem;color:var(--muted)">Dual-ESP32-CAM Access & Safety Hub</div>
      </div>
    </div>
    <div class="h-status">
      <span id="sysClockBadge" class="badge b-warn" style="font-family:monospace;font-size:.8rem">12:00:00 PM</span>
      <button class="secondary" style="font-size:.76rem;padding:4px 8px" onclick="syncBrowserTime()" title="Sync clock with browser">⏱️ Sync Time</button>
      <span id="badgeAlarm" class="badge b-ok">SYSTEM NORMAL</span>
      <span id="badgeMute" class="badge b-off">SIREN ARMED</span>
      <button id="btnMute" class="secondary" onclick="toggleMute()">Mute Siren</button>
      <button class="danger" onclick="triggerAction('siren_test')">Test Siren (3s)</button>
    </div>
  </header>

  <!-- Live Camera Streams Section -->
  <div class="card-title" style="margin-bottom:8px">📹 Live Access Cameras Output</div>
  <div class="grid-2col">
    <!-- Camera 1: Face Recognition -->
    <div class="card">
      <div class="card-title">
        <span>Camera 1: Face Recognition (HW-297)</span>
        <span id="cam1Status" class="badge b-off">CONNECTING...</span>
      </div>
      <div class="cam-box">
        <img id="cam1Img" class="cam-img" alt="Camera 1 Face Recognition Stream">
        <div class="cam-overlay">
          <span id="cam1Tag" class="badge b-off">NO FACE</span>
          <span id="cam1Fps" style="background:rgba(0,0,0,0.6);padding:2px 6px;border-radius:4px;font-size:.72rem">-- ms</span>
        </div>
      </div>
      <div class="cam-meta">
        <div class="row"><span class="k">Identified Person</span><span id="cam1Who" class="v">---</span></div>
        <div class="row"><span class="k">Link / Last Seen</span><span id="cam1Link" class="v">---</span></div>
      </div>
      <div class="cam-actions">
        <button onclick="triggerAction('door_open')">🔓 Unlock Door (Servo 3)</button>
        <button class="secondary" onclick="triggerAction('door_close')">🔒 Lock Door</button>
        <a id="cam1LinkBtn" class="btn secondary" href="http://192.168.4.101/" target="_blank">Open Face UI ↗</a>
      </div>
    </div>

    <!-- RFID Car Gate Access Control (Replaces Plate Cam) -->
    <div class="card">
      <div class="card-title">
        <span>🚗 RFID Car Gate Access (MFRC522)</span>
        <span id="rfidStatus" class="badge b-ok">ACTIVE</span>
      </div>
      <div class="rfid-box" style="padding:16px 12px;text-align:center;background:#0d1527;border-radius:8px;margin-bottom:12px;border:1px dashed #23324d">
        <div style="font-size:1.8rem;margin-bottom:4px">💳</div>
        <div style="font-size:.82rem;color:var(--muted)">Present RFID card to open Car Gate (Servos 1&amp;2 to 160°)</div>
        <div style="margin-top:8px;font-family:monospace;font-size:1.15rem;font-weight:700;color:#06b6d4" id="rfidUidText">NO CARD SCANNED</div>
      </div>
      <div class="cam-meta">
        <div class="row"><span class="k">Accepted Card UID</span><span id="rfidAccepted" class="v" style="font-family:monospace">52411902</span></div>
        <div class="row"><span class="k">Last Scan Activity</span><span id="rfidLastSeen" class="v">---</span></div>
      </div>
      <div class="cam-actions">
        <button onclick="triggerAction('gate_open')">🚗 Open Car Gate (Servos 1&amp;2)</button>
        <button class="secondary" onclick="triggerAction('gate_close')">⛔ Close Gate</button>
      </div>
    </div>
  </div>

  <!-- Sensors & Safety Telemetry -->
  <div class="card-title" style="margin-bottom:8px">⚡ Environmental & Access Sensors</div>
  <div class="grid">
    <!-- Flame Sensor -->
    <div class="card">
      <div class="card-title">
        <span>🔥 Flame Sensor (GPIO34)</span>
        <span id="flameBadge" class="badge b-ok">SAFE</span>
      </div>
      <div class="row"><span class="k">Raw ADC (Drops on Fire)</span><span id="flameVal" class="v">---</span></div>
      <div class="row"><span class="k">Alarm Threshold</span><span id="flameThText" class="v">&le; 500</span></div>
      <div class="bar-bg"><div id="flameBar" class="bar-fill" style="width:0%"></div></div>
    </div>

    <!-- Smoke MQ-2 Sensor -->
    <div class="card">
      <div class="card-title">
        <span>💨 Smoke MQ-2 (GPIO35)</span>
        <span id="smokeBadge" class="badge b-ok">CLEAN</span>
      </div>
      <div class="row"><span class="k">Raw Gas/Smoke ADC</span><span id="smokeVal" class="v">---</span></div>
      <div class="row"><span class="k">Alarm Threshold</span><span id="smokeThText" class="v">&ge; 2000</span></div>
      <div class="bar-bg"><div id="smokeBar" class="bar-fill" style="width:0%"></div></div>
    </div>

    <!-- PIR Motion Sensor -->
    <div class="card">
      <div class="card-title">
        <span>🚶 PIR Motion (GPIO27)</span>
        <span id="pirBadge" class="badge b-ok">CLEAR</span>
      </div>
      <div class="row"><span class="k">Motion State</span><span id="pirVal" class="v">No Motion</span></div>
      <div class="row"><span class="k">Siren Guard Window</span><span id="pirWindowText" class="v">00:00 - 07:00</span></div>
      <div class="row"><span class="k">Night Siren Guard</span><span id="pirGuardBadge" class="badge b-off">QUIET HOURS</span></div>
      <div style="font-size:.82rem;color:var(--muted);margin-top:6px">Active-HIGH sensor; alarm armed midnight-7am</div>
    </div>

    <!-- Ultrasonic Proximity -->
    <div class="card">
      <div class="card-title">
        <span>📏 Ultrasonic (GPIO32/33)</span>
        <span id="ultraBadge" class="badge b-ok">CLEAR</span>
      </div>
      <div class="row"><span class="k">Obstacle Distance</span><span id="ultraVal" class="v">---</span></div>
      <div class="row"><span class="k">Proximity Threshold</span><span id="ultraThText" class="v">&le; 25.0 cm</span></div>
      <div class="bar-bg"><div id="ultraBar" class="bar-fill" style="width:0%"></div></div>
    </div>
  </div>

  <!-- Actuators & Hardware State -->
  <div class="grid">
    <!-- Smart Lighting (GPIO4) -->
    <div class="card">
      <div class="card-title">
        <span>💡 Smart Lighting</span>
        <span id="lightBadge" class="badge b-off">LIGHT OFF</span>
      </div>
      <div class="row"><span class="k">Output Control</span><span class="v">GPIO 4 (Transistor Drive)</span></div>
      <div class="row"><span class="k">Current State</span><span id="lightStateText" class="v">OFF (0V)</span></div>
      <div class="cam-actions" style="margin-top:10px">
        <button id="btnLightToggle" onclick="triggerAction('light_toggle')">Toggle Light</button>
        <button class="secondary" onclick="triggerAction('light_on')">Turn ON</button>
        <button class="secondary" onclick="triggerAction('light_off')">Turn OFF</button>
      </div>
    </div>

    <!-- Servos State -->
    <div class="card">
      <div class="card-title">Servos &amp; Barriers</div>
      <div class="row"><span class="k">Car Gate (Servos 1 &amp; 2)</span><span id="gateState" class="v">CLOSED (0°)</span></div>
      <div class="row"><span class="k">Face Door (Servo 3)</span><span id="doorState" class="v">CLOSED (0°)</span></div>
      <div class="row"><span class="k">Siren Tone (GPIO25)</span><span id="sirenHz" class="v">OFF</span></div>
    </div>

    <!-- Wi-Fi & Network -->
    <div class="card">
      <div class="card-title">Network &amp; SoftAP</div>
      <div class="row"><span class="k">SoftAP Hub</span><span class="v">SmartHome-Net (192.168.4.1)</span></div>
      <div class="row"><span class="k">Connected Devices</span><span id="wifiClients" class="v">0 clients</span></div>
      <div class="row"><span class="k">Home Wi-Fi (Uplink)</span><span id="staStatus" class="v">Disabled</span></div>
    </div>

    <!-- Hub Health -->
    <div class="card">
      <div class="card-title">System Diagnostics</div>
      <div class="row"><span class="k">System Clock</span><span id="sysClockText" class="v">12:00:00 PM</span></div>
      <div class="row"><span class="k">System Uptime</span><span id="sysUptime" class="v">00:00:00</span></div>
      <div class="row"><span class="k">Free Heap Memory</span><span id="sysHeap" class="v">-- KB</span></div>
      <div class="row"><span class="k">ESP-NOW Dropped</span><span id="rxDropped" class="v">0</span></div>
    </div>
  </div>

  <!-- Live System Event Log -->
  <div class="card" style="margin-bottom:14px">
    <div class="card-title">
      <span>📋 Live Event Log</span>
      <button class="secondary" style="font-size:.76rem;padding:4px 8px" onclick="refreshStatus()">Refresh Now</button>
    </div>
    <div id="logBox" class="log-box"></div>
  </div>

  <!-- Accordion: Wi-Fi & Threshold Settings -->
  <details>
    <summary>⚙️ Hub Configuration &amp; Home Wi-Fi Setup</summary>
    <div style="margin-top:14px" class="grid-2col">

      <!-- Threshold Adjustments -->
      <div>
        <strong style="color:var(--text);font-size:.9rem">Alarm Thresholds &amp; Timers</strong>
        <div class="form-row">
          <span>Flame ADC Threshold (&le; Trigger)</span>
          <input id="cfgFire" type="number" min="100" max="3900" step="50">
        </div>
        <div class="form-row">
          <span>Smoke MQ-2 Threshold (&ge; Trigger)</span>
          <input id="cfgSmoke" type="number" min="200" max="4000" step="50">
        </div>
        <div class="form-row">
          <span>Ultrasonic Proximity Distance (cm)</span>
          <input id="cfgUltra" type="number" min="5" max="150" step="1">
        </div>
        <div class="form-row">
          <span>Car Gate Hold Open (ms)</span>
          <input id="cfgGate" type="number" min="1000" max="30000" step="500">
        </div>
        <div class="form-row">
          <span>Door Hold Open (ms)</span>
          <input id="cfgDoor" type="number" min="1000" max="30000" step="500">
        </div>
        <div class="form-row">
          <span>Sound Siren on PIR Motion</span>
          <input id="cfgMotSiren" type="checkbox">
        </div>
        <div class="form-row">
          <span>Sound Siren on Ultrasonic Range</span>
          <input id="cfgUltSiren" type="checkbox">
        </div>
        <div class="form-row">
          <span>PIR Night-Only Siren Guard</span>
          <input id="cfgPirSched" type="checkbox">
        </div>
        <div class="form-row">
          <span>PIR Guard Window (Midnight - 7 AM)</span>
          <div style="display:flex;gap:4px;align-items:center">
            <input id="cfgPirSH" type="number" min="0" max="23" style="width:55px" placeholder="00"> :
            <input id="cfgPirSM" type="number" min="0" max="59" style="width:55px" placeholder="00">
            <span style="font-size:.8rem;color:var(--muted);margin:0 2px">to</span>
            <input id="cfgPirEH" type="number" min="0" max="23" style="width:55px" placeholder="07"> :
            <input id="cfgPirEM" type="number" min="0" max="59" style="width:55px" placeholder="00">
          </div>
        </div>
        <div class="form-row">
          <span>Set Hub Clock (HH:MM)</span>
          <div style="display:flex;gap:4px;align-items:center">
            <input id="manHour" type="number" min="0" max="23" style="width:55px" placeholder="HH"> :
            <input id="manMin" type="number" min="0" max="59" style="width:55px" placeholder="MM">
            <button class="secondary" style="padding:4px 8px;font-size:.76rem" onclick="setManualTime()">Set</button>
          </div>
        </div>
        <div style="margin-top:10px;display:flex;gap:8px">
          <button onclick="saveSettings()">Save Thresholds</button>
          <button class="secondary" onclick="syncBrowserTime()">⏱️ Sync Browser Time</button>
        </div>
      </div>

      <!-- Home Wi-Fi Uplink Configuration -->
      <div>
        <strong style="color:var(--text);font-size:.9rem">Home Wi-Fi Router Connection (Optional)</strong>
        <p style="font-size:.82rem;color:var(--muted);margin:6px 0 10px">Connect the Hub to your home router so you can access this panel from your home network while keeping the dedicated 192.168.4.1 SoftAP active for the cameras.</p>
        <div class="form-row">
          <span>Router SSID</span>
          <input id="staSsid" type="text" maxlength="32" placeholder="e.g. MyHomeWiFi" style="width:160px">
        </div>
        <div class="form-row">
          <span>Password</span>
          <input id="staPass" type="password" maxlength="64" placeholder="••••••••" style="width:160px">
        </div>
        <div style="margin-top:10px;display:flex;gap:8px">
          <button onclick="saveWifi(false)">Connect to Home Wi-Fi</button>
          <button class="danger" onclick="saveWifi(true)">Disconnect / Forget</button>
        </div>
      </div>

    </div>
  </details>

</div>

<script>
let cfgLoaded = false;

// Auto-refresh camera frames with direct stream support
function streamCam(camId, imgId, badgeId, perfId, tagId){
  const img = document.getElementById(imgId);
  if (camId === 1) {
    // Attempt direct real-time MJPEG stream first
    const testImg = new Image();
    testImg.onload = () => {
      img.src = "http://192.168.4.101:81/stream";
      document.getElementById(badgeId).textContent = "LIVE (MJPEG)";
      document.getElementById(badgeId).className = "badge b-ok";
      document.getElementById(perfId).textContent = "Real-time";
    };
    testImg.onerror = () => {
      pollSnapshot(camId, imgId, badgeId, perfId);
    };
    testImg.src = "http://192.168.4.101:81/stream";
    return;
  }
}

function pollSnapshot(camId, imgId, badgeId, perfId){
  const img = document.getElementById(imgId);
  const next = new Image();
  const t0 = performance.now();
  next.onload = ()=>{
    img.src = next.src;
    document.getElementById(badgeId).textContent = "LIVE (SNAPSHOT)";
    document.getElementById(badgeId).className = "badge b-ok";
    document.getElementById(perfId).textContent = Math.round(performance.now() - t0) + " ms";
    setTimeout(()=>pollSnapshot(camId, imgId, badgeId, perfId), 350);
  };
  next.onerror = ()=>{
    document.getElementById(badgeId).textContent = "OFFLINE";
    document.getElementById(badgeId).className = "badge b-bad";
    setTimeout(()=>pollSnapshot(camId, imgId, badgeId, perfId), 1600);
  };
  next.src = "/api/cam/" + camId + "/preview.jpg?t=" + Date.now();
}

async function refreshStatus(){
  try{
    const r = await fetch('/api/status');
    const d = await r.json();

    // Master Alarms
    const bAlarm = document.getElementById('badgeAlarm');
    if(d.alarm){
      bAlarm.textContent = d.camAlarm ? "INTRUDER ALERT" : "ALARM ACTIVE";
      bAlarm.className = "badge b-bad";
    } else {
      bAlarm.textContent = "SYSTEM NORMAL";
      bAlarm.className = "badge b-ok";
    }

    const bMute = document.getElementById('badgeMute');
    bMute.textContent = d.sirenMuted ? "MUTED" : "SIREN ARMED";
    bMute.className = "badge " + (d.sirenMuted ? "b-warn" : "b-ok");
    document.getElementById('btnMute').textContent = d.sirenMuted ? "Unmute Siren" : "Mute Siren";

    // Sensors: Flame
    const fVal = document.getElementById('flameVal');
    fVal.textContent = d.flameAdc;
    const fPct = Math.min(100, Math.max(0, Math.round(((4095 - d.flameAdc)/4095)*100)));
    document.getElementById('flameBar').style.width = fPct + "%";
    document.getElementById('flameBar').className = "bar-fill " + (d.fire ? "bar-danger" : "");
    const fBadge = document.getElementById('flameBadge');
    fBadge.textContent = d.fire ? "FIRE ALERT" : "SAFE";
    fBadge.className = "badge " + (d.fire ? "b-bad" : "b-ok");

    // Sensors: Smoke
    document.getElementById('smokeVal').textContent = d.smokeAdc;
    const sPct = Math.min(100, Math.max(0, Math.round((d.smokeAdc/4095)*100)));
    document.getElementById('smokeBar').style.width = sPct + "%";
    document.getElementById('smokeBar').className = "bar-fill " + (d.smoke ? "bar-danger" : (d.smokeAdc>1200?"bar-warn":""));
    const sBadge = document.getElementById('smokeBadge');
    sBadge.textContent = d.smoke ? "SMOKE ALERT" : "CLEAN";
    sBadge.className = "badge " + (d.smoke ? "b-bad" : "b-ok");

    // Sensors: PIR
    const pBadge = document.getElementById('pirBadge');
    pBadge.textContent = d.pir ? "MOTION" : "CLEAR";
    pBadge.className = "badge " + (d.pir ? "b-warn" : "b-ok");
    document.getElementById('pirVal').textContent = d.pir ? "MOTION DETECTED!" : "No Motion";

    // Sensors: Ultrasonic
    document.getElementById('ultraVal').textContent = d.distCm>=0 ? (d.distCm.toFixed(1) + " cm") : "Out of Range";
    const uPct = d.distCm>=0 ? Math.min(100, Math.max(0, Math.round((d.distCm/100)*100))) : 100;
    document.getElementById('ultraBar').style.width = uPct + "%";
    document.getElementById('ultraBar').className = "bar-fill " + (d.ultra ? "bar-danger" : "");
    const uBadge = document.getElementById('ultraBadge');
    uBadge.textContent = d.ultra ? "PROXIMITY" : "CLEAR";
    uBadge.className = "badge " + (d.ultra ? "b-warn" : "b-ok");

    // Servos & Siren
    document.getElementById('gateState').textContent = (d.gateOpen?"OPEN ":"CLOSED ") + "(" + d.servo1 + "°/" + d.servo2 + "°)";
    document.getElementById('doorState').textContent = (d.doorOpen?"OPEN ":"CLOSED ") + "(" + d.door3 + "°)";
    document.getElementById('sirenHz').textContent = d.sirenActive ? (d.sirenHz + " Hz") : "OFF";

    // System Clock HUD
    if (d.time) {
      document.getElementById('sysClockBadge').textContent = (d.time.synced ? "🕒 " : "⏱️ [12PM] ") + d.time.timeOnly;
      document.getElementById('sysClockBadge').className = "badge " + (d.time.synced ? "b-ok" : "b-warn");
      document.getElementById('sysClockText').textContent = d.time.clock + (d.time.synced ? " (Synced)" : " (Default 12PM)");
    }

    // Smart Lighting
    const lBadge = document.getElementById('lightBadge');
    const lText = document.getElementById('lightStateText');
    if (d.lightState) {
      lBadge.textContent = "LIGHT ON";
      lBadge.className = "badge b-ok";
      lText.textContent = "ON (3.3V Active)";
      document.getElementById('btnLightToggle').textContent = "Turn OFF";
    } else {
      lBadge.textContent = "LIGHT OFF";
      lBadge.className = "badge b-off";
      lText.textContent = "OFF (0V Idle)";
      document.getElementById('btnLightToggle').textContent = "Turn ON";
    }

    // PIR Schedule Status
    if (d.cfg) {
      const sh = String(d.cfg.pirSH).padStart(2,'0');
      const sm = String(d.cfg.pirSM).padStart(2,'0');
      const eh = String(d.cfg.pirEH).padStart(2,'0');
      const em = String(d.cfg.pirEM).padStart(2,'0');
      document.getElementById('pirWindowText').textContent = sh + ":" + sm + " - " + eh + ":" + em;
      const pGuard = document.getElementById('pirGuardBadge');
      if (!d.cfg.motSiren) {
        pGuard.textContent = "SIREN MUTED";
        pGuard.className = "badge b-off";
      } else if (!d.cfg.pirSched) {
        pGuard.textContent = "ARMED 24/7";
        pGuard.className = "badge b-bad";
      } else if (d.pirSchedActive) {
        pGuard.textContent = "ARMED (NIGHT)";
        pGuard.className = "badge b-bad";
      } else {
        pGuard.textContent = "QUIET HOURS";
        pGuard.className = "badge b-off";
      }
    }

    // Network & Diags
    document.getElementById('wifiClients').textContent = d.wifi.apStations + " device(s) connected";
    document.getElementById('staStatus').textContent = d.wifi.staConnected ? (d.wifi.staSsid + " (" + d.wifi.staIp + ", " + d.wifi.staRssi + " dBm)") : (d.wifi.staSsid ? "Connecting..." : "Disabled");
    document.getElementById('sysUptime').textContent = d.uptimeStr;
    document.getElementById('sysHeap').textContent = Math.round(d.freeHeap/1024) + " KB";
    document.getElementById('rxDropped').textContent = d.rxDropped;

    // Camera 1 (Face Cam) Telemetry
    const c1Tag = document.getElementById('cam1Tag');
    c1Tag.textContent = d.faceCam.event;
    c1Tag.className = "badge " + (d.faceCam.event==='RECOGNIZED'?'b-ok':(d.faceCam.event==='UNKNOWN'?'b-bad':'b-off'));
    document.getElementById('cam1Who').textContent = d.faceCam.label + " (" + d.faceCam.conf + "%)";
    document.getElementById('cam1Link').textContent = d.faceCam.online ? ("Online (" + d.faceCam.ageSec + "s ago)") : "Offline";

    // RFID Car Gate Telemetry (Replaces plateCam)
    if(d.rfid){
      document.getElementById('rfidUidText').textContent = d.rfid.lastUid ? d.rfid.lastUid : "NO CARD SCANNED";
      document.getElementById('rfidAccepted').textContent = d.rfid.acceptedUid;
      document.getElementById('rfidLastSeen').textContent = d.rfid.lastScanSec < 9999 ? (d.rfid.lastScanSec + "s ago") : "---";
      const isAuth = (d.rfid.lastUid === d.rfid.acceptedUid);
      document.getElementById('rfidStatus').textContent = isAuth ? "AUTHORIZED" : (d.rfid.lastUid ? "DENIED" : "ACTIVE");
      document.getElementById('rfidStatus').className = "badge " + (isAuth ? "b-ok" : (d.rfid.lastUid ? "b-bad" : "b-ok"));
    }

    // Populate Settings once
    if(!cfgLoaded && d.cfg){
      cfgLoaded = true;
      document.getElementById('cfgFire').value = d.cfg.fireTh;
      document.getElementById('cfgSmoke').value = d.cfg.smokeTh;
      document.getElementById('cfgUltra').value = d.cfg.ultraCm;
      document.getElementById('cfgGate').value = d.cfg.gateMs;
      document.getElementById('cfgDoor').value = d.cfg.doorMs;
      document.getElementById('cfgMotSiren').checked = d.cfg.motSiren;
      document.getElementById('cfgUltSiren').checked = d.cfg.ultSiren;
      document.getElementById('cfgPirSched').checked = d.cfg.pirSched;
      document.getElementById('cfgPirSH').value = d.cfg.pirSH;
      document.getElementById('cfgPirSM').value = d.cfg.pirSM;
      document.getElementById('cfgPirEH').value = d.cfg.pirEH;
      document.getElementById('cfgPirEM').value = d.cfg.pirEM;
      document.getElementById('staSsid').value = d.wifi.staSsid;
    }

    // Populate Event Log
    const box = document.getElementById('logBox');
    box.innerHTML = '';
    d.events.slice().reverse().forEach(ev=>{
      const line = document.createElement('div');
      line.className = 'log-line';
      line.innerHTML = '<span class="log-time">' + ev.time + '</span><span class="log-msg c-' + ev.type + '">' + ev.msg + '</span>';
      box.appendChild(line);
    });

  }catch(e){}
}

async function triggerAction(act){
  await fetch('/api/control?action=' + act, {method:'POST'});
  refreshStatus();
}

async function toggleMute(){
  await triggerAction('siren_toggle_mute');
}

async function saveSettings(){
  const p = new URLSearchParams({
    fire_th: document.getElementById('cfgFire').value,
    smoke_th: document.getElementById('cfgSmoke').value,
    ultra_cm: document.getElementById('cfgUltra').value,
    gate_ms: document.getElementById('cfgGate').value,
    door_ms: document.getElementById('cfgDoor').value,
    mot_siren: document.getElementById('cfgMotSiren').checked ? 1 : 0,
    ult_siren: document.getElementById('cfgUltSiren').checked ? 1 : 0,
    pir_sched: document.getElementById('cfgPirSched').checked ? 1 : 0,
    pir_sh: document.getElementById('cfgPirSH').value,
    pir_sm: document.getElementById('cfgPirSM').value,
    pir_eh: document.getElementById('cfgPirEH').value,
    pir_em: document.getElementById('cfgPirEM').value
  });
  await fetch('/api/config?' + p.toString(), {method:'POST'});
  alert('Settings saved to NVS!');
  cfgLoaded = false;
  refreshStatus();
}

async function syncBrowserTime(){
  const now = new Date();
  const epoch = Math.floor(now.getTime() / 1000);
  await fetch('/api/time?epoch=' + epoch, {method:'POST'});
  alert('System clock synchronized with browser: ' + now.toLocaleTimeString());
  refreshStatus();
}

async function setManualTime(){
  const h = document.getElementById('manHour').value;
  const m = document.getElementById('manMin').value;
  if(h === '' || m === ''){ alert('Please enter hour and minute'); return; }
  await fetch('/api/time?hour=' + h + '&min=' + m, {method:'POST'});
  alert('System clock updated to ' + h.padStart(2,'0') + ':' + m.padStart(2,'0'));
  refreshStatus();
}

async function saveWifi(disconnect){
  if(disconnect){
    if(confirm('Disconnect from Home Wi-Fi?')){
      await fetch('/api/wifi?disconnect=1', {method:'POST'});
      cfgLoaded = false;
      refreshStatus();
    }
    return;
  }
  const ssid = document.getElementById('staSsid').value.trim();
  const pass = document.getElementById('staPass').value;
  if(!ssid){ alert('Please enter Wi-Fi SSID'); return; }
  const p = new URLSearchParams({ ssid: ssid, pass: pass });
  await fetch('/api/wifi?' + p.toString(), {method:'POST'});
  alert('Connecting to ' + ssid + '... Check status in a few seconds.');
  cfgLoaded = false;
  refreshStatus();
}

// Start live streams & polling
streamCam(1, 'cam1Img', 'cam1Status', 'cam1Fps', 'cam1Tag');
refreshStatus();
setInterval(refreshStatus, 1000);
</script>
</body>
</html>)HTML";

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send_P(200, "text/html", HTML);
}

// ---------------------------------------------------------------------------
// Wi-Fi & ESP-NOW Initialization
// ---------------------------------------------------------------------------
void onWiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Device connected to Hub SoftAP (%d active)", WiFi.softAPgetStationNum() + 1);
    addEventLog("info", msg);
  } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Device disconnected from Hub SoftAP (%d active)", WiFi.softAPgetStationNum());
    addEventLog("info", msg);
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Home Wi-Fi connected! Assigned IP: %s", WiFi.localIP().toString().c_str());
    addEventLog("success", msg);
    configTime(0, 0, "pool.ntp.org", "time.google.com");
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    static unsigned long lastStaDiscMs = 0;
    if (millis() - lastStaDiscMs >= 10000) {
      lastStaDiscMs = millis();
      addEventLog("warning", "Home Wi-Fi station link disconnected");
    }
  }
}

void initHubWiFi() {
  WiFi.persistent(false);
  WiFi.onEvent(onWiFiEvent);

  bool hasSta = (strlen(g_cfg.staSsid) > 0);
  if (hasSta) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }

  // Maximize TX power (19.5 dBm) for maximum wireless coverage to nodes
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);

  IPAddress apIp(SmartHomeNet::HUB_IP[0], SmartHomeNet::HUB_IP[1],
                 SmartHomeNet::HUB_IP[2], SmartHomeNet::HUB_IP[3]);
  IPAddress subnet(SmartHomeNet::SUBNET_MASK[0], SmartHomeNet::SUBNET_MASK[1],
                   SmartHomeNet::SUBNET_MASK[2], SmartHomeNet::SUBNET_MASK[3]);
  WiFi.softAPConfig(apIp, apIp, subnet);
  WiFi.softAP(SmartHomeNet::AP_SSID, SmartHomeNet::AP_PASS, SmartHomeNet::WIFI_CHANNEL, 0, 8);

  if (hasSta) {
    Serial.printf("[WIFI] Connecting to Home Wi-Fi: %s ...\n", g_cfg.staSsid);
    WiFi.begin(g_cfg.staSsid, g_cfg.staPass);
  }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW);

  pinMode(FIRE_SENSOR_PIN, INPUT);
  pinMode(MQ2_SENSOR_PIN, INPUT);
  pinMode(PIR_SENSOR_PIN, INPUT_PULLDOWN);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN), onUltrasonicEchoChange, CHANGE);

  Serial.begin(115200);
  delay(200);

  initDefaultSystemTime();
  loadConfigFromNvs();

  // Servo timers (allocate all 4 timers to ensure clean channel allocation)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  attachGateServo(gateServo1, GATE_SERVO_1_PIN, currentServo1Angle, "gate servo 1");
  attachGateServo(gateServo2, GATE_SERVO_2_PIN, currentServo2Angle, "gate servo 2");
  attachGateServo(doorServo3, DOOR_SERVO_3_PIN, currentDoor3Angle, "face door servo 3");

  // Siren LEDC Setup (Channel 4)
  ledcSetup(SIREN_LEDC_CHANNEL, SIREN_LOW_HZ, SIREN_RESOLUTION_BITS);
  ledcAttachPin(BUZZER_PIN, SIREN_LEDC_CHANNEL);
  buzzerAttached = true;
  ledcWriteTone(SIREN_LEDC_CHANNEL, 0);

  // RFID reader init (MFRC522)
  SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);
  rfid.PCD_Init();
  Serial.println("[RFID] Reader initialized. Present a card to read its UID.");

  // Network initialization
  initHubWiFi();

  // ESP-NOW
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);
    espNowReady = true;
  } else {
    Serial.println("ERROR: esp_now_init failed on Hub!");
  }

  // Web Server Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/control", HTTP_POST, handleApiControl);
  server.on("/api/config", HTTP_POST, handleApiConfig);
  server.on("/api/time", HTTP_POST, handleApiTime);
  server.on("/api/wifi", HTTP_POST, handleApiWifi);
  server.on("/api/cam/event", HTTP_POST, handleApiCamEvent);
  server.on("/api/cam/1/preview.jpg", HTTP_GET, []() { handleCamProxy(1); });
  server.on("/api/cam/2/preview.jpg", HTTP_GET, []() { handleCamProxy(2); });
  server.begin();

  addEventLog("info", "Smart Home Hub started. Web Admin ready.");
  Serial.println("=== Smart Home Hub Initialized ===");
  Serial.printf("SoftAP: %s (IP: 192.168.4.1, ch=%d)\n", SmartHomeNet::AP_SSID, SmartHomeNet::WIFI_CHANNEL);
}

// ---------------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1. Process incoming ESP-NOW camera events
  IncomingCamPacket item;
  while (popCamPacket(item)) {
    handleCamEvent(item, now);
  }

  // 2. Service WebServer requests
  server.handleClient();

  // 3. Read analog sensors
  lastFlameAdc = analogRead(FIRE_SENSOR_PIN);
  lastSmokeAdc = analogRead(MQ2_SENSOR_PIN);
  lastMotionDetected = (digitalRead(PIR_SENSOR_PIN) == HIGH);

  lastFireDetected = (lastFlameAdc <= g_cfg.fireThreshold);
  lastSmokeDetected = (lastSmokeAdc >= g_cfg.smokeThreshold);

  // 4. Update non-blocking ultrasonic sensor
  updateUltrasonicSensor(now);

  // 4b. RFID card reader — controls main car gate (Servos 1 & 2 to 160°)
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    String uid;
    for (byte i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] < 0x10) uid += "0";
      uid += String(rfid.uid.uidByte[i], HEX);
    }
    uid.toUpperCase();
    if (uid != lastRfidUid || (now - lastRfidPrintMs >= RFID_PRINT_INTERVAL_MS)) {
      lastRfidUid = uid;
      lastRfidPrintMs = now;
      Serial.print("[RFID] Card UID: ");
      Serial.println(uid);

      if (uid == ACCEPTED_RFID_UID) {
        plateGateUntilMs = now + g_cfg.gateHoldMs;
        setGateOpen(true);
        char msg[72];
        snprintf(msg, sizeof(msg), "RFID Authorized [%s] -> Car Gate opened", uid.c_str());
        addEventLog("success", msg);
        Serial.println("[RFID] Accepted card -> Gate opened (Servos 1 & 2 to 160°)");
      } else {
        camAlarmUntilMs = now + g_cfg.camAlarmMs;
        startSiren(now);
        char msg[72];
        snprintf(msg, sizeof(msg), "RFID Unknown [%s] -> Intruder alarm", uid.c_str());
        addEventLog("danger", msg);
        Serial.println("[RFID] Unknown card -> Intruder alarm started");
      }
    }
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }

  // 5. Evaluate camera intruder alert timeout
  bool camAlarmActive = false;
  if (camAlarmUntilMs != 0) {
    if (static_cast<long>(camAlarmUntilMs - now) > 0) {
      camAlarmActive = true;
    } else {
      camAlarmUntilMs = 0;
    }
  }

  // 6. Evaluate siren test timeout
  bool sirenTestActive = false;
  if (sirenTestUntilMs != 0) {
    if (static_cast<long>(sirenTestUntilMs - now) > 0) {
      sirenTestActive = true;
    } else {
      sirenTestUntilMs = 0;
    }
  }

  // 7. Master alarm evaluation
  bool pirScheduleActive = true;
  if (g_cfg.pirScheduleEnabled) {
    pirScheduleActive = isPirScheduleActive();
  }
  bool motionAlarm = (lastMotionDetected && g_cfg.motionTriggersSiren && pirScheduleActive);
  bool ultraAlarm  = (lastUltrasonicDetected && g_cfg.ultraTriggersSiren);

  lastAlarmActive = lastFireDetected || lastSmokeDetected || motionAlarm ||
                    ultraAlarm || camAlarmActive || sirenTestActive;

  if (lastAlarmActive) {
    startSiren(now);
    updateSiren(now);
  } else {
    stopSiren();
  }

  // 8. Servo barrier movement & holding timers
  updateServosControl(now);

  // 9. Serial debug monitor
  if (now - lastPrintMs >= 500) {
    lastPrintMs = now;
    time_t printTime = time(nullptr);
    struct tm printTm;
    localtime_r(&printTime, &printTm);
    char timePrintBuf[16];
    strftime(timePrintBuf, sizeof(timePrintBuf), "%H:%M:%S", &printTm);

    Serial.printf("[%s] Sensors: Flame=%d (%s) | Smoke=%d (%s) | PIR=%s (Guard:%s) | Ultra=%s | Light=%s | Alarm=%s | Gate=%s | Door=%s\n",
                  timePrintBuf,
                  lastFlameAdc, lastFireDetected ? "FIRE!" : "ok",
                  lastSmokeAdc, lastSmokeDetected ? "SMOKE!" : "ok",
                  lastMotionDetected ? "MOTION" : "clear",
                  pirScheduleActive ? "ARMED" : "QUIET",
                  lastUltrasonicDetected ? "NEAR" : "clear",
                  lightState ? "ON" : "OFF",
                  lastAlarmActive ? (g_cfg.sirenMuted ? "MUTED" : "ACTIVE") : "off",
                  gateOpen ? "OPEN" : "closed",
                  doorOpen ? "OPEN" : "closed");
  }

  delay(2);
}
