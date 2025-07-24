#ifndef FRAMING_H
#define FRAMING_H

#define FRAME_TYPE_DATA 0x01

struct Frame {
  uint8_t type;
  uint8_t senderHash[16];
  uint8_t content[100];
  uint16_t contentLength;
};

#endif
