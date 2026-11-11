#include <unity.h>
#include <Arduino.h>
#include "smart_home_protocol.h"
#include "smart_home_json.h"

// Replicate the exact servo math from src/main.cpp to test independently
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;

int testServoAngleToPulseUs(int angle) {
  int clamped = constrain(angle, 0, 160);
  long span = static_cast<long>(SERVO_MAX_US - SERVO_MIN_US);
  return SERVO_MIN_US + static_cast<int>((clamped * span + 90L) / 180L);
}

int testStepServoAngle(int current, int target, int maxStep) {
  if (current < target) return min(current + maxStep, target);
  if (current > target) return max(current - maxStep, target);
  return current;
}

void test_servo_clamping_and_pulse_us() {
  // 0° must equal 500 µs
  TEST_ASSERT_EQUAL_INT(500, testServoAngleToPulseUs(0));

  // 160° must equal 2278 µs (500 + round(160 * 2000 / 180) = 500 + 1778 = 2278)
  TEST_ASSERT_EQUAL_INT(2278, testServoAngleToPulseUs(160));

  // Any angle exceeding 160° must be clamped strictly to 160° (safety margin)
  TEST_ASSERT_EQUAL_INT(2278, testServoAngleToPulseUs(180));
  TEST_ASSERT_EQUAL_INT(2278, testServoAngleToPulseUs(220));

  // Negative angles must clamp to 0° (500 µs)
  TEST_ASSERT_EQUAL_INT(500, testServoAngleToPulseUs(-10));
}

void test_servo_stepping() {
  int current = 0;
  int target = 160;
  int step = 2;

  // Step from 0 towards 160
  current = testStepServoAngle(current, target, step);
  TEST_ASSERT_EQUAL_INT(2, current);

  // Step from 159 towards 160
  current = testStepServoAngle(159, target, step);
  TEST_ASSERT_EQUAL_INT(160, current);

  // Step down from 160 towards 0
  current = testStepServoAngle(160, 0, step);
  TEST_ASSERT_EQUAL_INT(158, current);
}

void test_cam_event_packet_size_and_packing() {
  TEST_ASSERT_EQUAL_UINT32(37, sizeof(SmartHomeNet::CamEventPacket));

  SmartHomeNet::CamEventPacket pkt = {};
  pkt.magic = SmartHomeNet::PACKET_MAGIC;
  pkt.source = SmartHomeNet::CAM_SOURCE_FACE;
  pkt.event = SmartHomeNet::CAM_EVENT_RECOGNIZED;
  pkt.matchId = 3;
  pkt.confidence = 94;
  pkt.seq = 42;
  strncpy(pkt.label, "Khaled", sizeof(pkt.label) - 1);

  TEST_ASSERT_EQUAL_UINT32(0x53484331, pkt.magic);
  TEST_ASSERT_EQUAL_UINT8(SmartHomeNet::CAM_SOURCE_FACE, pkt.source);
  TEST_ASSERT_EQUAL_UINT8(SmartHomeNet::CAM_EVENT_RECOGNIZED, pkt.event);
  TEST_ASSERT_EQUAL_INT16(3, pkt.matchId);
  TEST_ASSERT_EQUAL_UINT8(94, pkt.confidence);
  TEST_ASSERT_EQUAL_UINT32(42, pkt.seq);
  TEST_ASSERT_EQUAL_STRING("Khaled", pkt.label);
}

void test_json_escaping() {
  String escaped = smartHomeJsonEscape("Hello \"World\" \\ Test");
  TEST_ASSERT_EQUAL_STRING("Hello \\\"World\\\" \\\\ Test", escaped.c_str());

  String clean = smartHomeJsonEscape("NormalText123");
  TEST_ASSERT_EQUAL_STRING("NormalText123", clean.c_str());
}

void setup() {
  delay(1000);
  UNITY_BEGIN();
  RUN_TEST(test_servo_clamping_and_pulse_us);
  RUN_TEST(test_servo_stepping);
  RUN_TEST(test_cam_event_packet_size_and_packing);
  RUN_TEST(test_json_escaping);
  UNITY_END();
}

void loop() {
  // Empty loop for unit test runner
}
