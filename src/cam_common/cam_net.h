#pragma once

#include <Arduino.h>
#include "smart_home_protocol.h"

void camNetInit(SmartHomeNet::CamSource source);
void camNetLoop();

bool camNetSendEvent(SmartHomeNet::CamEventType event,
                     int16_t matchId,
                     uint8_t confidence,
                     const char *label);

bool camNetPostHttpEvent(SmartHomeNet::CamEventType event,
                         int16_t matchId,
                         uint8_t confidence,
                         const char *label);

bool camNetHubConnected();
bool camNetFallbackApActive();
bool camNetLastSendOk();
uint32_t camNetSentCount();
String camNetIpString();
String camNetFallbackApSsid();
String camNetFallbackApIp();
String camNetMacString();
