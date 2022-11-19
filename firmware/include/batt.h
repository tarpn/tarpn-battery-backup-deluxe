#ifndef BATT_H
#define BATT_H

const uint32_t MIN_CHARGE_STATE_MS = 120000;

enum ChargingState {
  CHARGE_OFF,
  CHARGE_INIT,
  CHARGE_CC,
  CHARGE_CC_HOLD,
  CHARGE_CV
};

typedef struct {
  ChargingState state;
  unsigned long state_ms;
} ChargeState;

#endif