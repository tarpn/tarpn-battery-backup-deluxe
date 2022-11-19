#ifndef SER_H
#define SER_H

#include <stdint.h>

enum SerialCmd {
  SER_UNKNOWN,
  SER_PRINT,
  SER_HEARTBEAT,
  SER_NET,
  SER_GET,
  SER_SET,
  SER_TEST,
  SER_SAVE_EEPROM,
  SER_LOAD_EEPROM,
  SER_RESET_EEPROM,
  SER_VERSION
};

const char serial_cmd_types[] = {
  '?', 'P', 'H', 'N', 'G', 'S', 'T', 'E', 'L', 'R', 'V'
};

typedef struct {
  bool in_frame;
  SerialCmd cmd;
  uint8_t bytes;    // Size of buffer
  char buffer[21];  // Bytes read after the command code and a space
} SerialFrame;

void Ser_Init(SerialFrame * ser);

bool Ser_Feed(SerialFrame * ser, int byte);

void Ser_On_Complete(SerialFrame * ser, unsigned long now);

#endif