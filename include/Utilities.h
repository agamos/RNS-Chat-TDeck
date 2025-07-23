#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>

void bytes_to_hex(const uint8_t* bytes, size_t len, char* output) {
  const char hex[] = "0123456789abcdef";
  for (size_t i = 0; i < len; i++) {
    output[i * 2] = hex[(bytes[i] >> 4) & 0xF];
    output[i * 2 + 1] = hex[bytes[i] & 0xF];
  }
  output[len * 2] = '\0';
}

#endif