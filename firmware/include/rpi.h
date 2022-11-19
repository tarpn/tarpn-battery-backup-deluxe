#ifndef RPI_H
#define RPI_H

#include "stdint.h"

enum RPiShutdownState {
  RPI_POWERED_ON,               // Default state, assume RPi is running
  RPI_KEEPALIVE_POWERED_ON,     // 
  RPI_KEEPALIVE_POWEROFF_SEEN,  // Saw spontaneous poweroff signal from RPi. Try to reboot it.
  RPI_KEEPALIVE_POWERED_OFF,    // Switch is open, waiting some time before reboot (in case of user maintenance).
  RPI_SHUTDOWN_EVENT_SENT,      // User initiated shutdown. Sent serial message to RPi to start shutdown
  RPI_SHUTDOWN_POWEROFF_SEEN,   // Saw the RPi poweroff signal after sending shutdown.
  RPI_SHUTDOWN_POWERED_OFF,     // RPi switch open, to cycle the RPi
};

typedef struct {
  unsigned long last_state_change;
  RPiShutdownState state;
  bool rpi_poweroff_active;
  bool switch_state;
  uint8_t ip_address[4];
} RPi; 

void RPi_Init(RPi * rpi);

void RPi_Update(RPi * rpi, unsigned long now);

void RPi_Shutdown(RPi * rpi, unsigned long now);

#endif