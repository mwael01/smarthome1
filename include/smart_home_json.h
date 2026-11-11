#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

inline String smartHomeJsonEscape(const char *s) {
  String out;
  if (!s) return out;
  for (size_t i = 0; s[i] != '\0'; ++i) {
    char c = s[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<uint8_t>(c) >= 0x20) {
      out += c;
    }
  }
  return out;
}

// Extract integer value from JSON string by key, tolerant of whitespace around colons
inline bool smartHomeJsonGetInt(const String &json, const char *key, int &outVal) {
  if (!key) return false;
  int keyLen = strlen(key);
  int searchPos = 0;
  while (searchPos < static_cast<int>(json.length())) {
    int idx = json.indexOf(key, searchPos);
    if (idx < 0) return false;
    // Check that key is preceded and followed by '"'
    if (idx > 0 && json[idx - 1] == '"' && json[idx + keyLen] == '"') {
      int p = idx + keyLen + 1;
      while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
      if (p < static_cast<int>(json.length()) && json[p] == ':') {
        p++;
        while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
        outVal = json.substring(p).toInt();
        return true;
      }
    }
    searchPos = idx + keyLen;
  }
  return false;
}

// Extract string value from JSON string by key, tolerant of whitespace around colons
inline bool smartHomeJsonGetString(const String &json, const char *key, char *outBuf, size_t outSize) {
  if (!key || !outBuf || outSize == 0) return false;
  int keyLen = strlen(key);
  int searchPos = 0;
  while (searchPos < static_cast<int>(json.length())) {
    int idx = json.indexOf(key, searchPos);
    if (idx < 0) return false;
    if (idx > 0 && json[idx - 1] == '"' && json[idx + keyLen] == '"') {
      int p = idx + keyLen + 1;
      while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
      if (p < static_cast<int>(json.length()) && json[p] == ':') {
        p++;
        while (p < static_cast<int>(json.length()) && (json[p] == ' ' || json[p] == '\t')) p++;
        if (p < static_cast<int>(json.length()) && json[p] == '"') {
          p++;
          int endQuote = json.indexOf('"', p);
          if (endQuote >= p) {
            String val = json.substring(p, endQuote);
            strncpy(outBuf, val.c_str(), outSize - 1);
            outBuf[outSize - 1] = '\0';
            return true;
          }
        }
      }
    }
    searchPos = idx + keyLen;
  }
  return false;
}
