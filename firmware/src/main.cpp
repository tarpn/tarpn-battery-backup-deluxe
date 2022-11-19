#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SleepyDog.h>
#include <Bounce2.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

#include <LowPower.h>
#include <U8g2lib.h>

#include "config.h"
#include "voltage_read.h"
#include "ui.h"
#include "pid.h"
#include "free_memory.h"
#include "ser.h"
#include "rpi.h"
#include "metrics.h"
#include "batt.h"

#define RPI_POWEROFF_PIN 4  // pulled high, low, high when RPi is ready for power off 
#define STATUS_LED 7
#define RPI_PWR_PIN 8
#define BATTERY_PIN 5
#define SUPPLY_PIN 19
#define BUTTON_PIN 6

#define ADC_WARMUP_TIME_MS            2000    // Collect samples from the ADC for this long after startup
#define ADC_SAMPLE_INTERVAL_US         100    // How often to read from the ADC
#define SERIAL_BAUD_RATE             38400

#define STANDBY_LOW_VOLTAGE           12.5    // Below this PSU voltage, connect the battery
#define BATTERY_HIGH_VOLTAGE_CUTOFF   15.0    // At this voltage, disconnect the battery to protect loads
#define BATTERY_HIGH_VOLTAGE_RECOVERY 14.0    // At this voltage, reconnect the battery after high cutoff
#define BATTERY_LOW_VOLTAGE_CUTOFF    11.5    // At this voltage, disconnect the battery to protect battery
#define BATTERY_LOW_VOLTAGE_RECOVERY  12.0    // At this voltage, reconnect the battery after low cutoff
//#define BATTERY_LOW_VOLTAGE_SHUTDOWN  10.0    // At this voltage, the system will shutdown the RPI (TODO sleep?)
#define BATTERY_HIGH_VOLTAGE_TRIP_MS  10
#define BATTERY_LOW_VOLTAGE_TRIP_MS   3000
#define BATTERY_RECOVERY_MS           10000
#define OVER_TEMP_CUTOFF              60      // At this temperature in celsius, disconnect battery
#define OVER_TEMP_RECOVERY            50      // 
#define OVER_TEMP_RECOVERY_MS         10000   // 
#define BATTERY_FLOAT_VOLTAGE         12.5    // Default to a low float voltage so its disabled.

#define ABSOLUTE_MAX_VOLTAGE 18.0
#define ABSOLUTE_MAX_TEMP 100.0
//#define EEPROM_RESET 1

#define ADC_ZENER_VOLTAGE     4.90    // Measured from D1
#define VOLTAGE_FACTOR        0.267   // For 10k 27.4k resistor divider use 0.267.
#define VOLTAGE_OFFSET        0.000   // a fixed offset for voltages seen at ADC to account for signal path loss or ground offset
#define CURRENT_FACTOR        0.040   // Shunt resistor value times gain factor (0.002 * 20 = 0.04)
#define CURRENT_OFFSET        2.486   // Op amp uses a 2.4V reference to allow for negative voltages


struct StatefulConfigs {
  // Tunables
  float reference_voltage;
  float voltage_factor;
  float voltage_offset;
  float current_factor;
  float current_offset;
  float batt_current_factor;
  float batt_current_offset;
  // Configurables
  float standby_voltage;
  float batt_high_voltage_cutoff;
  float batt_high_voltage_recovery;
  float batt_high_voltage_trip_ms;
  float batt_low_voltage_cutoff;
  float batt_low_voltage_recovery;
  float batt_low_voltage_trip_ms;
  float batt_recovery_ms;
  float overtemp_cutoff;
  float overtemp_recovery;
  float overtemp_recovery_ms;
  float batt_float_voltage;
} config_defaults = {
  // Tunables
  ADC_ZENER_VOLTAGE, 
  VOLTAGE_FACTOR, 
  VOLTAGE_OFFSET,
  CURRENT_FACTOR, 
  CURRENT_OFFSET,
  CURRENT_FACTOR, 
  CURRENT_OFFSET,
  // Configurables
  STANDBY_LOW_VOLTAGE,
  BATTERY_HIGH_VOLTAGE_CUTOFF,
  BATTERY_HIGH_VOLTAGE_RECOVERY,
  BATTERY_HIGH_VOLTAGE_TRIP_MS,
  BATTERY_LOW_VOLTAGE_CUTOFF,
  BATTERY_LOW_VOLTAGE_RECOVERY,
  BATTERY_LOW_VOLTAGE_TRIP_MS,
  BATTERY_RECOVERY_MS,
  OVER_TEMP_CUTOFF,
  OVER_TEMP_RECOVERY,
  OVER_TEMP_RECOVERY_MS,
  BATTERY_FLOAT_VOLTAGE
};

StatefulConfigs configs;

enum State {
  Null,             // 0
  Initializing,     // 1
  Standby,          // 2
  Backup,           // 3
  Recovery,         // 4
  BatteryLow,       // 5
  BatteryLowTrip,   // 6
  BatteryHigh,      // 7
  BatteryHighTrip,  // 8
  OverTemp,         // 9
  OverTempRecovery, // 10
  BeginShutdown,    // 11
  ShutdownComplete, // 12
  SelfTest          // 13
};

const char state_0[] PROGMEM = "NULL";
const char state_1[] PROGMEM = "INIT";
const char state_2[] PROGMEM = "STANDBY";
const char state_3[] PROGMEM = "BACKUP";
const char state_4[] PROGMEM = "RECOVERY";
const char state_5[] PROGMEM = "BATT_LOW";
const char state_6[] PROGMEM = "BATT_LOW_TRIP";
const char state_7[] PROGMEM = "BATT_HIGH";
const char state_8[] PROGMEM = "BATT_HIGH_TRIP";
const char state_9[] PROGMEM = "OVER_TEMP";
const char state_10[] PROGMEM = "OVER_TEMP_RECOVER";
const char state_11[] PROGMEM = "BEGIN_SHUTDOWN";
const char state_12[] PROGMEM = "SHUTDOWN_COMPLETE";
const char state_13[] PROGMEM = "SELF_TEST";

const char* const states[] PROGMEM = {
  state_0, state_1, state_2, state_3, state_4, state_5,
  state_6, state_7, state_8, state_9, state_10, state_11, 
  state_12, state_13
};

char label_buffer[16];

PID pid_cc;
PID pid_cv;
PID * pid = &pid_cc; // points to the active PID

float ONE = 1.0;

SensorRead batteryVoltage;
SensorRead supplyVoltage;
SensorRead tempSense;
SensorRead currentSense;
SensorRead chargeSense;

/**
 * A nicely packed structure for storing the configurations in the EEPROM.
 * All values are scaled up by 1000 (except timing values) and stored as 16-bit unsigned ints.
 */
struct PackedConfig {
  // v1
  uint16_t reference_voltage;
  uint16_t voltage_factor;
  uint16_t current_factor;
  uint16_t current_offset;  
  uint16_t standby_voltage;
  uint16_t batt_high_voltage_cutoff;
  uint16_t batt_high_voltage_recovery;
  uint16_t batt_high_voltage_trip_ms;
  uint16_t batt_low_voltage_cutoff;
  uint16_t batt_low_voltage_recovery;
  uint16_t batt_low_voltage_trip_ms;
  uint16_t batt_recovery_ms;
  uint16_t overtemp_cutoff;
  uint16_t overtemp_recovery;
  uint16_t overtemp_recovery_ms;
  // v2
  int16_t voltage_offset;   // Signed int because we can have negative offset
  // v3
  uint16_t batt_float_voltage;
  // v4
  uint16_t batt_current_factor;
  uint16_t batt_current_offset; 
};

#define LATEST_EEPROM_VERSION 4

void toPackedConfig(StatefulConfigs * configs, PackedConfig * packed) {
  packed->reference_voltage = configs->reference_voltage * 1000;
  packed->voltage_factor = configs->voltage_factor * 1000;
  packed->current_factor = configs->current_factor * 1000;
  packed->current_offset = configs->current_offset * 1000;
  packed->standby_voltage = configs->standby_voltage * 1000;
  packed->batt_high_voltage_cutoff = configs->batt_high_voltage_cutoff * 1000;
  packed->batt_high_voltage_recovery = configs->batt_high_voltage_recovery * 1000;
  packed->batt_high_voltage_trip_ms = configs->batt_high_voltage_trip_ms;
  packed->batt_low_voltage_cutoff = configs->batt_low_voltage_cutoff * 1000;
  packed->batt_low_voltage_recovery = configs->batt_low_voltage_recovery * 1000;
  packed->batt_low_voltage_trip_ms = configs->batt_low_voltage_trip_ms;
  packed->batt_recovery_ms = configs->batt_recovery_ms;
  packed->overtemp_cutoff = configs->overtemp_cutoff;
  packed->overtemp_recovery = configs->overtemp_recovery;
  packed->overtemp_recovery_ms = configs->overtemp_recovery_ms;
  // v2
  packed->voltage_offset = configs->voltage_offset * 1000;
  // v3
  packed->batt_float_voltage = configs->batt_float_voltage * 1000;
  // v4
  packed->batt_current_factor = configs->batt_current_factor * 1000;
  packed->batt_current_offset = configs->batt_current_offset * 1000;
}

void fromPackedConfig(uint8_t version, StatefulConfigs * configs, PackedConfig * packed) {
  // v4
  if (version > 3) {
    configs->batt_current_factor = packed->batt_current_factor / 1000.0;
    configs->batt_current_offset = packed->batt_current_offset / 1000.0;
  } else {
    configs->batt_current_factor = config_defaults.batt_current_factor;
    configs->batt_current_offset = config_defaults.batt_current_offset;
  }
  
  // v3
  if (version > 2) {
    configs->batt_float_voltage = packed->batt_float_voltage / 1000.0;
  } else {
    configs->batt_float_voltage = config_defaults.batt_float_voltage;
  }
  
  // v2
  if (version > 1) {
    configs->voltage_offset = packed->voltage_offset / 1000.0;
  } else {
    configs->voltage_offset = config_defaults.voltage_offset;
  }
  
  if (version > 0) {
    // v1
    configs->reference_voltage = packed->reference_voltage / 1000.0;
    configs->voltage_factor = packed->voltage_factor / 1000.0;
    configs->current_factor = packed->current_factor / 1000.0;
    configs->current_offset = packed->current_offset / 1000.0;
    configs->standby_voltage = packed->standby_voltage / 1000.0;
    configs->batt_high_voltage_cutoff = min(ABSOLUTE_MAX_VOLTAGE, packed->batt_high_voltage_cutoff / 1000.0);
    configs->batt_high_voltage_recovery = packed->batt_high_voltage_recovery / 1000.0;
    configs->batt_high_voltage_trip_ms = packed->batt_high_voltage_trip_ms;
    configs->batt_low_voltage_cutoff = packed->batt_low_voltage_cutoff / 1000.0;
    configs->batt_low_voltage_recovery = packed->batt_low_voltage_recovery / 1000.0;
    configs->batt_low_voltage_trip_ms = packed->batt_low_voltage_trip_ms;
    configs->batt_recovery_ms = packed->batt_recovery_ms;
    configs->overtemp_cutoff = min(ABSOLUTE_MAX_TEMP, packed->overtemp_cutoff);
    configs->overtemp_recovery = packed->overtemp_recovery;
    configs->overtemp_recovery_ms = packed->overtemp_recovery_ms;
  }
}

void save_config() {
  PackedConfig to_store;
  toPackedConfig(&configs, &to_store);
  EEPROM.put(0x00, LATEST_EEPROM_VERSION);
  EEPROM.put(0x01, to_store);
}

void reset_config() {
  // Zero out everything
  for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.update(i, 0);
  }
  // replace it with the defaults
  EEPROM.put(0x00, LATEST_EEPROM_VERSION);
  PackedConfig to_store;
  toPackedConfig(&config_defaults, &to_store);
  EEPROM.put(0x01, to_store);
}

void load_config() {
  PackedConfig packed;
  uint8_t version = EEPROM.read(0x00);
  EEPROM.get(0x01, packed);
  fromPackedConfig(version, &configs, &packed);
}

unsigned long start_time;
State state = Initializing;
unsigned long last_state_change;
double temperature;
unsigned long last_adc_read;
unsigned long last_serial_write;
unsigned long last_serial_read;
unsigned long last_heartbeat;
bool bpq_running;

uint32_t self_test_ms = 0;

Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);
int last_encoder_pos = 0;
Bounce user_sw;

U8G2_SSD1309_128X64_NONAME0_2_4W_HW_SPI u8g2(U8G2_R0, 10, 9);

Metrics metrics;
MetricsReport report;
RPi rpi;
ChargeState charger;

void RPi_Init(RPi * rpi) {
  rpi->state = RPI_POWERED_ON;
  rpi->switch_state = LOW;
  rpi->rpi_poweroff_active = false;
}

void RPi_Shutdown(RPi * rpi, unsigned long now) {
  Serial.println(F("EVENT SHUTDOWN_RPI"));
  rpi->last_state_change = now;
  rpi->state = RPI_SHUTDOWN_EVENT_SENT;
}

void RPi_Update(RPi * rpi, unsigned long now) {
  // poweroff pin is low when the RPi is powered off. 
  // It transitions low, high, low during poweroff
  rpi->rpi_poweroff_active = digitalRead(RPI_POWEROFF_PIN) == LOW;

  RPiShutdownState newState = rpi->state;
  switch (rpi->state) {
    case RPI_POWERED_ON:
      rpi->switch_state = LOW;
      if (rpi->rpi_poweroff_active) {
        newState = RPI_KEEPALIVE_POWEROFF_SEEN;
      }
      break;
    case RPI_KEEPALIVE_POWEROFF_SEEN:
      if ((now - rpi->last_state_change) > 3000) {
        newState = RPI_KEEPALIVE_POWERED_OFF;
      }
      break;
    case RPI_KEEPALIVE_POWERED_OFF:
      rpi->switch_state = HIGH;
      if ((now - rpi->last_state_change) > 60000) {
        newState = RPI_KEEPALIVE_POWERED_ON;
      }
      break;
    case RPI_KEEPALIVE_POWERED_ON:
      rpi->switch_state = LOW;
      // Wait for poweroff signal to go away
      if ((now - rpi->last_state_change) > 3000 && !rpi->rpi_poweroff_active) {
        newState = RPI_POWERED_ON;
      }
      break;
    case RPI_SHUTDOWN_EVENT_SENT:
      if (rpi->rpi_poweroff_active) {
        newState = RPI_SHUTDOWN_POWEROFF_SEEN;
      }
      // After 1 minute, give up and go back to default state
      if ((now - rpi->last_state_change) > 60000) {
        newState = RPI_POWERED_ON;
      }
      break;
    case RPI_SHUTDOWN_POWEROFF_SEEN:
      if ((now - rpi->last_state_change) > 3000) {
        newState = RPI_SHUTDOWN_POWERED_OFF;
      }
      break;
    case RPI_SHUTDOWN_POWERED_OFF:
      rpi->switch_state = HIGH;
      break;
  }
  if (newState != rpi->state) {
    rpi->state = newState;
    rpi->last_state_change = now;
  }
}

/**
 * Calculate the temperature using Steinhart–Hart equation. A 49.9k resistor and 10k NTC thermistor are hard coded here.
 */
double calculate_temp() {
  // We still use the EMWA filter in VoltageRead, but calculate the real value here.
  double res = (49900.0 * Sensor_adcVoltageFast(&tempSense)) / (Sensor_smoothedValueFast(&batteryVoltage) - Sensor_adcVoltageFast(&tempSense));
  double steinhart = logf(res / 10000.0);
  steinhart /= 3933;
  steinhart += (1.0 / 298.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  return steinhart;
}

/*******************************************************************************
 *                                                                             *
 *                                 State Machine                               *
 *                                                                             *
 *******************************************************************************/

/**
 * Determine if a state change is needed based on the sensor data. If a state changes is needed, this routine
 * updates the global "state" variable as well as "last_state_change". This routine does not actually change any
 * digital output signals, rather it determines the correct internal state. See "apply_state" for
 * updating the output signals based on the internal state.
 * 
 * Returns true if the state was updated, false otherwise.
 */
bool next_state(unsigned long now, double temp, UIState * ui) {
  State current_state = state;

  if (current_state != Initializing &&
      current_state != OverTemp &&
      current_state != BeginShutdown &&
      current_state != ShutdownComplete &&
      temp >= configs.overtemp_cutoff) {
    // Over temp logic applies to most states, so do it separately.
    last_state_change = now;
    ui->error_code = OverTempMessage;
    state = OverTemp;
    return true;
  }

  double load_voltage_drop_max = Sensor_instantValue(&currentSense) * 0.05;
  uint32_t current_state_ms = (uint32_t) (now - last_state_change);
  State next_state = Null;

  switch (current_state) {
  case Null:  
    // Shouldn't be possible, but just transition to Initializing
    next_state = Initializing;
    break;
  case Initializing:
    if (current_state_ms >= ADC_WARMUP_TIME_MS) {
      next_state = Standby;
    }
    break;
  case Standby:
    /*
      Note on load spikes.

      If we see a sudden voltage drop, it can be from either a supply loss or a sudden load spike.
      We can estimate a maximum voltage drop due to load by looking at the current sensor. The FETs 
      have around a 0.010 ohm resistance, so we expect around a 0.1V drop per 10A. Adding in a buffer 
      for PCB trace, connector, and wire resistance, we estimate a total system resistance of 0.05 ohm.

      While in Standby mode, the supply voltage is the load voltage. If we see supply voltage drop below
      the configured standby voltage minus the expected max voltage drop, then we are experiencing a 
      supply loss.
    */    
    if ((Sensor_smoothedValueFast(&supplyVoltage) + load_voltage_drop_max) < configs.standby_voltage) {
      ui->error_code = LowSupplyVoltageMessage;
      next_state = Backup;
    }
    // TODO if battery voltage gets too low, we'll lose the RPi. Until charging is implemented
    // it is safest to enter the shutdown sequence before the battery voltage gets critically low
    if (Sensor_smoothedValueFast(&batteryVoltage) < configs.batt_low_voltage_cutoff) {
      next_state = BatteryLow;
    }
    if (self_test_ms > 0) {
      next_state = SelfTest;
    }
    break;
  case SelfTest:
    if (current_state_ms >= self_test_ms) {
      self_test_ms = 0;
      next_state = Standby;
    }
    break;
  case Backup:
    if (Sensor_smoothedValueFast(&batteryVoltage) < configs.batt_low_voltage_cutoff) {
      next_state = BatteryLow;
    } else if (Sensor_smoothedValueFast(&batteryVoltage) >= configs.batt_high_voltage_cutoff) {
      next_state = BatteryHigh;
    } else if (Sensor_smoothedValueFast(&supplyVoltage) >= configs.standby_voltage) {
      next_state = Recovery;
    }
    break;
  case Recovery:
    if (Sensor_smoothedValueFast(&supplyVoltage) < configs.standby_voltage) {
      next_state = Backup;
    } else if (current_state_ms >= 3000) {
      ui->error_code = SupplyRecovered;
      next_state = Standby;
    } 
    break;
  case BatteryLow:
    if (current_state_ms >= configs.batt_low_voltage_trip_ms) {
      ui->error_code = LowBatteryVoltageMessage;
      next_state = BatteryLowTrip;
    } else if (Sensor_smoothedValueFast(&batteryVoltage) >= configs.batt_low_voltage_recovery) {
      next_state = Backup;
    }
    break;
  case BatteryLowTrip:
    if (current_state_ms >= 60000 && Sensor_smoothedValueFast(&batteryVoltage) < 11.0) {
      ui->error_code = ShutdownMessage;
      next_state = BeginShutdown;
      RPi_Shutdown(&rpi, now);
    } else if (Sensor_smoothedValueFast(&batteryVoltage) >= configs.batt_low_voltage_recovery) {
      next_state = Backup;
    }
    break;
  case BatteryHigh:
    if (current_state_ms >= configs.batt_high_voltage_trip_ms) {
      ui->error_code = HighBatteryVoltageMessage;
      next_state = BatteryHighTrip;
    } else if (Sensor_smoothedValueFast(&batteryVoltage) < configs.batt_high_voltage_recovery) {
      next_state = Backup;
    }
    break;
  case BatteryHighTrip:
    if (Sensor_smoothedValueFast(&batteryVoltage) < configs.batt_high_voltage_recovery) {
      next_state = Backup;
    }
    break;
  case OverTemp:
    if (temp < configs.overtemp_recovery) {
      next_state = OverTempRecovery;
    }
    break;
  case OverTempRecovery:
    // Don't need to check for over temp here since we check it at the top of this routine.
    // Just check that we're still below the recovery temperature for long enough.
    if (current_state_ms >= configs.overtemp_recovery_ms && temp < configs.overtemp_recovery) {
      next_state = Standby;
    }
    break;
  case BeginShutdown:
    // After low battery trip, we wait 10 seconds or until RPi is shutdown
    if (current_state_ms > 60000) {
      next_state = ShutdownComplete;
    }
    break;
  case ShutdownComplete:
    if (Sensor_instantValue(&batteryVoltage) >= configs.batt_low_voltage_recovery) {
      //display.dim(false);
      next_state = Initializing;
      RPi_Init(&rpi);
    }
    break;
  }

  if (next_state == Null) {
    return false;
  } else {
    last_state_change = now;
    state = next_state;
    return true;
  }
}

bool is_battery_connected(State state) {
  switch (state) {
    case Backup:
    case Recovery:
    case BatteryLow:
    case BatteryHigh:
    case SelfTest:
      return true;
    case Standby:
      return pid->pid_output > 0;
    default:
      return false;
  }
}

bool is_battery_charging(State state) {
  return state == Standby && pid->pid_output > 0;
}

bool is_supply_connected(State state) {
  switch (state) {
    case Initializing:
    case Standby:
      return true;
    default:
      return false;
  }
}

/**
 * For the current state, determine the appropriate digital outputs. 
 */
void apply_state(unsigned long now, State state) {
  bool turn_off_charging = true;
  uint8_t pwm = 255;
  switch (state) {
    case Backup:
    case Recovery:
    case BatteryLow:
    case BatteryHigh:
    case SelfTest:
      // RPi is on, battery is connected, supply is disconnected
      digitalWrite(BATTERY_PIN, LOW);
      digitalWrite(SUPPLY_PIN, HIGH);
      digitalWrite(STATUS_LED, HIGH);
      break;
    case BatteryLowTrip:
    case OverTemp:
    case OverTempRecovery:
    case BatteryHighTrip:
    case BeginShutdown:
    case ShutdownComplete:
      // RPi is on, battery is disconnected, supply is disconnected
      digitalWrite(BATTERY_PIN, HIGH);
      digitalWrite(SUPPLY_PIN, HIGH);
      digitalWrite(STATUS_LED, LOW);
      break;
    case Standby:
      // RPi is on, battery is disconnected, supply is connected, supply may be charging battery
      pwm = 255 - (uint8_t) pid->pid_output;
      turn_off_charging = false;
      // intentional fall-through
    case Initializing:
    default:
      analogWrite(BATTERY_PIN, pwm);
      digitalWrite(SUPPLY_PIN, LOW);
      digitalWrite(STATUS_LED, LOW);
      break;
  }

  // RPi switch is managed in separate state machine.
  digitalWrite(RPI_PWR_PIN, rpi.switch_state);

  // Some states disable battery charging. Even though the switch state is already correct,
  // make sure to reset the charging state machine
  if (turn_off_charging) {
    charger.state = CHARGE_OFF;
  }
}

/*********************************
 *                               *
 * Configuration and UI Routines *
 *                               *
 *********************************/

double pow10(int8_t places) {
  int decade = 1;
  bool negative = places < 0;
  places = abs(places);
  while (places > 0) {
    decade *= 10;
    places--;
  }
  if (negative) {
    return 1.0 / decade;
  } else {
    return decade;
  }
}

UIState ui;


void UI_Init(UIState * ui, unsigned long now) {
  /*
  ui->config_depth = 0;
  ui->config_mode_enabled = false;
  ui->config_selection_0 = 0;
  ui->config_offset_0 = 0;
  ui->config_selection_1 = 0;
  ui->config_selection_2 = 0;
  ui->display_choice = 0;
  ui->display_error = false;
  ui->error_code = NoMessage;
  */
  ui->last_display_update_ms = now;
  //ui->last_encoder_pos = 0;
  ui->last_error_display = now;
  //ui->event = NONE;
  ui->last_event = now;
  //ui->headless_debug_mode_enabled = false;
}

void UI_encoder_button_press(UIState * ui, unsigned long now) {
  // simple de-bounce
  if (ui->last_event == BUTTON_PRESS) {
    if ((now - ui->last_event) > 50) {
      ui->event = BUTTON_PRESS;
      ui->last_event = now;
    }
  } else {
    ui->event = BUTTON_PRESS;
    ui->last_event = now;
  }
}

void UI_encoder_position(UIState * ui, unsigned long now, int pos) {
  if (pos > ui->last_encoder_pos + 2) {
    ui->last_encoder_pos = pos;
    ui->event = SCROLL_RIGHT;
    ui->last_event = now;
  } else if (pos < ui->last_encoder_pos - 2) {
    ui->last_encoder_pos = pos;
    ui->event = SCROLL_LEFT;
    ui->last_event = now;
  }
}

UserEvent UI_next_event(UIState * ui, unsigned long now) {
  UserEvent e = ui->event;
  ui->event = NONE;
  return e;
}

void display_info(UIState * ui, unsigned long now, UserEvent event) {
  if (event == BUTTON_PRESS) {
    ui->config_depth = 0;
    return;
  }

  long uptime_sec = (long) ((now - start_time) / 1000.0);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont11_tr);

    u8g2.setCursor(0, 12);
    u8g2.print(F("TARPN BBD "));
    u8g2.print(state);
    u8g2.print(" ");
    u8g2.print(uptime_sec);
    u8g2.setCursor(0, 24);
    u8g2.print(F("David Arthur, K4DBZ"));
    u8g2.setCursor(0, 36);
    u8g2.print(BUILD_DATE);
    u8g2.setCursor(0, 48);
    u8g2.print(BUILD_VERSION);
    u8g2.setCursor(0, 60);
    u8g2.print(F("Mem "));
    u8g2.print(freeMemory());
  } while (u8g2.nextPage());
}

// These are allocated in SRAM since they need reference to global objects
struct DynamicConfig {
  float * value;
  SensorRead * derived;
};

// These are stored in Flash RAM (PROGMEM)
struct StaticConfig {
  const char name[3];
  const char label_1[15];
  const char label_2[17];
  int8_t hi;  // highest power of 10 to change
  int8_t lo;  // lowest power of 10 to change
  uint8_t decimal_places;  // digits after decimal
} staticConfigSRAM {"", "", "", 0, 0, 0};

#define NUM_CONFIGS 12
#define NUM_TUNABLES 7

struct DynamicConfig dynamicConfigs[NUM_CONFIGS + NUM_TUNABLES] = {
  {&(configs.standby_voltage), NULL},
  {&(configs.batt_recovery_ms), NULL},
  {&(configs.batt_float_voltage), NULL},
  {&(configs.batt_high_voltage_cutoff), NULL},
  {&(configs.batt_high_voltage_recovery), NULL},
  {&(configs.batt_high_voltage_trip_ms), NULL},
  {&(configs.batt_low_voltage_cutoff), NULL},
  {&(configs.batt_low_voltage_recovery), NULL},
  {&(configs.batt_low_voltage_trip_ms), NULL},
  {&(configs.overtemp_cutoff), NULL},
  {&(configs.overtemp_recovery), NULL},
  {&(configs.overtemp_recovery_ms), NULL},
  {&(configs.current_factor), &currentSense},
  {&(configs.current_offset), &currentSense},
  {&(configs.reference_voltage), &currentSense},
  {&(configs.voltage_factor), &batteryVoltage},
  {&(configs.voltage_offset), &batteryVoltage},
  {&(configs.batt_current_factor), &chargeSense},
  {&(configs.batt_current_offset), &chargeSense},
};

const struct StaticConfig staticConfigs[NUM_CONFIGS + NUM_TUNABLES] PROGMEM = {
  {"B1", "Battery", "Standby Voltage", 0, -1, 1},
  {"B2", "Battery", "Recovery ms", 3, 0, 0},
  {"B3", "Battery", "Charging Voltage", 0, -1, 1},
  {"H1", "Battery High", "Voltage Cutoff", 0, -1, 1},
  {"H2", "Battery High", "Voltage Recovery", 0, -1, 1},
  {"H3", "Battery High", "Trip Time ms", 3, 0, 0},
  {"L1", "Battery Low", "Voltage Cutoff", 0, -1, 1},
  {"L2", "Battery Low", "Voltage Recovery", 0, -1, 1},
  {"L3", "Battery Low", "Trip Time ms", 2, 0, 0},
  {"T1", "Temperature", "Cutoff Temp", 1, -1, 1},
  {"T2", "Temperature", "Recovery Temp", 1, -1, 1},
  {"T3", "Temperature", "Recovery Time ms", 3, 0, 0},
  {"X1", "Current Sense", "", -1, -3, 3},
  {"X2", "Current Offset", "", 0, -3, 3},
  {"X3", "ADC Voltage", "", 0, -3, 3},
  {"X4", "Voltage Factor", "", 0, -3, 3},
  {"X5", "Voltage Offset", "", 0, -3, 3},
  {"X6", "Batt Factor", "", 0, -3, 3},
  {"X7", "Batt Offset", "", 0, -3, 3},
};

char float_buffer[10];
void display_float(float value, uint8_t width, uint8_t precision, Print * printer) {
  // TODO dtostrf calls dtoa_prf which is 702 bytes... maybe optimize this?
  dtostrf(value, width, precision, float_buffer); 
  (*printer).print(float_buffer);
}

DynamicConfig dynamicConfig;
void adjust_set_levels(UIState * ui, unsigned long now, UserEvent event, bool levelsOrTune, uint8_t choices_len) {  
  // For each config, need the display string, valid units to cycle through, and location
  // of digit to flash.

  uint8_t idx = ui->config_selection_1;
  if (!levelsOrTune) {
    idx += NUM_CONFIGS;
  }

  DynamicConfig dynamicConfig = dynamicConfigs[idx];
  memcpy_P(&staticConfigSRAM, &staticConfigs[idx], sizeof(StaticConfig));
  double val = *(dynamicConfig.value);
  dtostrf(abs(val), 5, staticConfigSRAM.decimal_places, ui->buffer_1);

  if (dynamicConfig.derived != NULL) {
    double derived = dynamicConfig.derived->sensorValue;
    dtostrf(derived, 4, 2, ui->buffer_2);
  }

  uint8_t digit_choices = 1 + staticConfigSRAM.hi - staticConfigSRAM.lo;
  uint8_t digit_idx = ui->config_selection_2 % digit_choices;
  int8_t exponent = (staticConfigSRAM.hi - digit_idx);
  double adj = pow10(exponent);

  uint8_t last_place = 74; // width is 5 characters right aligned. 10 + 4*16
  uint8_t blink_pos = last_place - (exponent * 16);
  if (staticConfigSRAM.decimal_places > 0) {
    blink_pos -= (16 + 16 * staticConfigSRAM.decimal_places);
  }
  if (exponent < 0) {
    blink_pos += 16;
  }

  if (event == SCROLL_LEFT) {
    val -= adj;
  }
  if (event == SCROLL_RIGHT) {
    val += adj;
  }
  *(dynamicConfig.value) = val;

  if (event == BUTTON_PRESS) {
    if (ui->config_selection_2 == (digit_choices - 1)) {
      ui->config_selection_2 = 0;
      ui->config_depth = 1;
    } else {
      ui->config_selection_2++;
      return;
    }
  }

  if (dynamicConfig.derived != NULL) {
    memset(ui->buffer_2, 0x20, 21); // fill with space characters
    dtostrf(dynamicConfig.derived->sensorValue, 4, 0, &(ui->buffer_2[0]));
    dtostrf(Sensor_adcVoltageFast(dynamicConfig.derived), 4, 3, &(ui->buffer_2[6]));
    dtostrf(Sensor_smoothedValueFast(dynamicConfig.derived), 4, 2, &(ui->buffer_2[12]));
  }

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont29_mn);
    u8g2.drawStr(10, 31, ui->buffer_1);
    u8g2.setFont(u8g2_font_profont11_tr);
    u8g2.drawStr(0, 7, "Adjust Limits");	
    u8g2.drawStr(114, 7, staticConfigSRAM.name);
    u8g2.drawHLine(0, 9, 128);
    u8g2.drawHLine(blink_pos, 33, 14);
    u8g2.drawStr(10, 40, staticConfigSRAM.label_1);	
    u8g2.drawStr(10, 52, staticConfigSRAM.label_2);	
    if (dynamicConfig.derived != NULL) {
      u8g2.drawStr(10, 64, ui->buffer_2);
    }
  } while (u8g2.nextPage());
  return;
}

void display_battery(unsigned long now, State state) {
  char * batt_s = {0};
  
  u8g2.setFont(u8g2_font_battery19_tn);
  double batt_v = Sensor_smoothedValueFast(&batteryVoltage);
   if (batt_v >= configs.batt_float_voltage) { 
    batt_s[0] = 53;
    u8g2.drawStr(80, 30, batt_s);
    return;
  }
  
  float batt_step = (configs.batt_float_voltage - configs.batt_low_voltage_cutoff) / 5.0;
  char batt_ch = 48 + (char) max(0, floor((batt_v - configs.batt_low_voltage_cutoff) / batt_step));
  // Blink the next bar up if we're charging. Flash every 800ms
  if (is_battery_charging(state) && (now / 800 % 2)) {
    batt_ch++;
  }    
  batt_s[0] = batt_ch;
  u8g2.drawStr(80, 30, batt_s);
}

void display_switch_states(unsigned long now, State state) {
  // 18 pixel wide space. line of 6 px
  // 0 |.   .   .   .   .|
  // 1 | .   .   .   .   |
  // 2 |  .   .   .   .  |
  // 3 |   .   .   .   . |
  uint8_t step = (now / 500) % 4;

  if (is_supply_connected(state)) {
    for (uint8_t i=0; i<6; i++) {
      if (step > 2 && i == 5) {
        break;
      }
      u8g2.drawPixel(17 + step + i*4, 40);
    }
  }

  if (is_battery_connected(state)) {
    for (uint8_t i=0; i<5; i++) {
      if (step < 2 && i == 4) {
        break;
      }
      u8g2.drawPixel(52 + (3-step) + i*4, 40);
    }
  }
}

/**
 * Render the "Set Levels" sub-menu. The configurable values are defined in "dynamicConfigs" and "staticConfigs".
*/
void display_set_levels(UIState * ui, unsigned long now, UserEvent e, bool levelsOrTune, uint8_t choices_len) {
  if (e == SCROLL_RIGHT) {
    cycle_selection(&(ui->config_selection_1), choices_len + 1, true);
  }
  if (e == SCROLL_LEFT) {
    cycle_selection(&(ui->config_selection_1), choices_len + 1, false);
  }
  if (e == BUTTON_PRESS) {
    if (ui->config_selection_1 == choices_len) {
      ui->config_selection_1 = 0;
      ui->config_depth = 0;
      return;
    }
    
    ui->config_selection_2 = 0;
    ui->config_depth = 2;
    return;
  }

  if (ui->config_selection_1 == choices_len) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_profont11_tr);
      u8g2.drawStr(0, 7, "Set Limits");	
      //u8g2.drawStr(114, 7, staticConfigSRAM.name);
      u8g2.drawXBMP(50, 34, 7, 6, return_glyph);
      u8g2.drawHLine(0, 9, 128);
      u8g2.drawStr(10, 40, "Return");	  
    } while (u8g2.nextPage());
    return; 
  }

  // Select the correct DynamicConfig and load the StaticConfig from PROGMEM
  uint8_t idx = ui->config_selection_1;
  if (!levelsOrTune) {
    idx += NUM_CONFIGS;
  }

  DynamicConfig dynamicConfig = dynamicConfigs[idx];
  memcpy_P(&staticConfigSRAM, &staticConfigs[idx], sizeof(StaticConfig));
  dtostrf(*(dynamicConfig.value), 0, staticConfigSRAM.decimal_places, ui->buffer_1);

  if (dynamicConfig.derived != NULL) {
    double derived = Sensor_smoothedValueFast(dynamicConfig.derived);
    dtostrf(derived, 4, 2, ui->buffer_2);
  }

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont29_mn);
    u8g2.drawStr(10, 31, ui->buffer_1);
    u8g2.setFont(u8g2_font_profont11_tr);
    u8g2.drawStr(0, 7, "Set Limits");	
    u8g2.drawStr(114, 7, staticConfigSRAM.name);
    u8g2.drawHLine(0, 9, 128);
    u8g2.drawStr(10, 40, staticConfigSRAM.label_1);	
    u8g2.drawStr(10, 52, staticConfigSRAM.label_2);	
    if (dynamicConfig.derived != NULL) {
      u8g2.drawStr(10, 64, ui->buffer_2);
    }
  } while (u8g2.nextPage());
  return;
}


void display_config(UIState * ui, unsigned long now, UserEvent event) {
  if (ui->config_depth == 0) {
    if (event == SCROLL_RIGHT) {
      cycle_selection(&(ui->config_selection_0), menu_0_len, true);
    }
    if (event == SCROLL_LEFT) {
      cycle_selection(&(ui->config_selection_0), menu_0_len, false);
    }
    if (event == BUTTON_PRESS) {
      bool exit_config = false;
      switch (ui->config_selection_0) {
        case 0:
          // Set Limits
          ui->config_depth = 1;
          ui->config_selection_1 = 0;
          break;
        case 1:
          // Tune
          ui->config_depth = 1;
          ui->config_selection_1 = 0;
          break;
        case 2:
          save_config();
          ui->error_code = EEPROMSavedMessage;
          exit_config = true;
          break;
        case 3:
          reset_config();
          ui->error_code = EEPROMResetMessage;
          exit_config = true;
          break;
        case 4:
          RPi_Shutdown(&rpi, now);
          ui->error_code = RPIShutdownMessage;
          exit_config = true;
          break;
        case 5:
          // Info
          ui->config_depth = 1;
          ui->config_selection_1 = 0;
          break;
        default:
          exit_config = true;
          break;
      }
      if (exit_config) {
        // TODO move this somewhere else
        ui->config_selection_0 = 0;
        ui->config_offset_0 = 0;
        ui->config_selection_1 = 0;
        ui->config_offset_1 = 0;
        ui->config_selection_2 = 0;
        ui->config_offset_2 = 0;
        ui->config_mode_enabled = false;
      }
      return;
    }
    
    // Boundary conditions for menu_0 offset
    if (ui->config_selection_0 == 0) {
      ui->config_offset_0 = 0;
    }

    if (ui->config_selection_0 == menu_0_len - 1) {
      ui->config_offset_0 = menu_0_len - 5;
    }

    if (ui->config_selection_0 > (ui->config_offset_0 + 3)) {
      ui->config_offset_0++;
    }

    if (ui->config_selection_0 < ui->config_offset_0) {
      ui->config_offset_0--;
    }

    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_profont11_tr);
      u8g2.drawStr(0, 7, "Menu");	
      u8g2.drawHLine(0, 9, 128);

      // Display 4 items from menu_0
      for (uint8_t i = 0; i < 4; i++) {
        uint8_t idx = i + ui->config_offset_0;
        strcpy_P(ui->buffer_1, (char*)pgm_read_word(&(menu_0[idx])));
        if (ui->config_selection_0 == idx) {
          u8g2.drawStr(0, 20 + 12*i, ">");	
        }
        u8g2.drawStr(10, 20 + 12*i, ui->buffer_1);	
      }
    } while (u8g2.nextPage());

    return;
  }

  if (ui->config_depth == 1) {
    switch (ui->config_selection_0) {
      case 0:
        display_set_levels(ui, now, event, true, NUM_CONFIGS);
        break;
      case 1:
        display_set_levels(ui, now, event, false, NUM_TUNABLES);
        break;
      case 5:
        display_info(ui, now, event);
        break;
    } 
    return;
  }


  if (ui->config_depth == 2) {
    if (ui->config_selection_0 == 0) {
        adjust_set_levels(ui, now, event, true, NUM_CONFIGS);
    } else if (ui->config_selection_0 == 1) {
        adjust_set_levels(ui, now, event, false, NUM_TUNABLES);
    }
  }
}

double round_double(double val, uint8_t places) {
  double decade = pow10(places);
  //int decade = pow(10, places);
  return round(val * decade) / decade;
}

const char * DOT = ".";
unsigned long t1;
unsigned long t2;
unsigned long ui_us = 0;

void display_main(UIState * ui, unsigned long now, UserEvent event) {
  if (event == SCROLL_RIGHT) {
    ui->display_choice ++;
  } else if (event == SCROLL_LEFT) {
    ui->display_choice --;
  } else if (event == BUTTON_PRESS) {
    ui->config_mode_enabled = true;
  }

#ifdef ADC_DEBUG
  dtostrf(supplyVoltage->sensorValue, 0, 0, ui->message_line_1);
  dtostrf(supplyVoltage->filter, 0, 0, ui->message_line_2);
#endif
  uint8_t display_choice = ui->display_choice % 5;
  bool negative_current = false;
  double tmp;

  switch (display_choice) {
    case 0: // Battery
      dtostrf(Sensor_smoothedValueFast(&batteryVoltage), 4, 2, ui->buffer_1);
      break;
    case 1: // Supply
      dtostrf(Sensor_smoothedValueFast(&supplyVoltage), 4, 2, ui->buffer_1);
      break;
    case 2: // Current
      dtostrf(Sensor_smoothedValueFast(&currentSense), 4, 2, ui->buffer_1);
      break;
      case 3: // Charge
      tmp = Sensor_smoothedValueFast(&chargeSense);
      if (tmp < 0.0) {
        negative_current = true;
      }
      dtostrf(abs(tmp), 4, 2, ui->buffer_1);
      break;
    case 4: // Temp
      // TODO negative temps?
      tmp = temperature * 1.8 + 32.0;
      dtostrf(tmp, 4, 2, ui->buffer_1);
      break;
    default:
      break;
  }
  
  // Figure out what to display in lines 1 and 2.
  if (ui->display_error) {
    if ((now - ui->last_error_display) > 3000) {
      //strcpy_P(ui->message_line_1, error_0);
      memset(ui->message_line_1, 0, 21);
      memset(ui->buffer_2, 0, 21);
      for(uint8_t i=0; i<4; i++) {
        itoa(rpi.ip_address[i], ui->buffer_2, 10);
        strcat(ui->message_line_1, ui->buffer_2);
        if (i < 3) {
          strcat(ui->message_line_1, DOT);
        }
      }
      strcpy_P(ui->message_line_2, (char*)pgm_read_word(&(states[state])));
      ui->display_error = false;
    }
  } else {
    // If we're not displaying an error and one has been set, load it and display it
    if (ui->error_code != NoMessage) {
      strcpy_P(ui->message_line_1, (char*)pgm_read_word(&(error_messages[ui->error_code])));
      //strcpy_P(ui->message_line_2, error_0);
      strcpy_P(ui->message_line_2, (char*)pgm_read_word(&(states[state])));
      ui->last_error_display = now;
      ui->display_error = true;
      ui->error_code = NoMessage;
    }
  }
  
  t1 = micros();  
  u8g2.firstPage();
  do {
    // Big text
    u8g2.setFont(u8g2_font_profont29_mn);
    u8g2.drawStr(0, 30, ui->buffer_1);

    // Regular font
    u8g2.setFont(u8g2_font_profont11_tr);
    u8g2.drawStr(0, 56, ui->message_line_1);
    u8g2.drawStr(0, 64, ui->message_line_2);
    switch (display_choice) {
      case 0: // Battery
        u8g2.drawStr(0, 7, "Battery Voltage");	
        break;
      case 1: // Supply
        u8g2.drawStr(0, 7, "Supply Voltage");	
        break;
      case 2: // Current
        u8g2.drawStr(0, 7, "Load Current");	
        break;
       case 3: // Charge
        if (negative_current) {
          u8g2.drawStr(0, 7, "Charge Current");	   
        } else {
          u8g2.drawStr(0, 7, "Discharge Current");	   
        }
        break;
      case 4: // Temp
        u8g2.drawStr(0, 7, "Temperature");	
        break;
      default:
        break;
    }

    // Side info
    u8g2.drawStr(100, 7, "RPi"); // RPi power switch
    if (!rpi.switch_state) {
      u8g2.drawXBMP(122, 0, 6, 7, check_s);
    } else {
      u8g2.drawXBMP(122, 0, 6, 7, x);
    }

    u8g2.drawStr(100, 19, "OS"); // Linux running
    if (!rpi.rpi_poweroff_active) {
      u8g2.drawXBMP(122, 12, 6, 7, check_s);  
    } else {
      u8g2.drawXBMP(122, 12, 6, 7, x);
    }

    u8g2.drawStr(100, 31, "Py"); // Agent
    if (last_heartbeat == 0) {
      u8g2.drawXBMP(122, 24, 6, 7, interrogative); 
    } else if ((now - last_heartbeat) < 10000) {
      u8g2.drawXBMP(122, 24, 6, 7, check_s); 
    } else {
      u8g2.drawXBMP(122, 24, 6, 7, x); 
    }

    u8g2.drawStr(100, 43, "BPQ");
    if (last_heartbeat == 0 || (now - last_heartbeat) > 10000) {
      u8g2.drawXBMP(122, 36, 6, 8, interrogative);  // BPQ
    } else if (bpq_running) {
      u8g2.drawXBMP(122, 36, 6, 8, check_s);
    } else {
      u8g2.drawXBMP(122, 36, 6, 8, x);
    }

    //u8g2.drawStr(0, 56, "YOUR AD");
    //u8g2.drawStr(0, 64, "GOES HERE");

    u8g2.drawStr(0, 44, "PSU");
    u8g2.drawStr(70, 44, "BATT");

   
    display_switch_states(now, state);

    if (ui->headless_debug_mode_enabled) {
      u8g2.drawPixel(127, 63);
    }

    //if (is_supply_connected(state) || is_battery_connected(state)) {
    u8g2.drawXBMP(38, 32, 14, 12, radio_glyph);
    //}
    
    // Battery icons, for now display the PWM value
    display_battery(now, state);

    u8g2.drawHLine(0, 46, 128);
    u8g2.drawVLine(96, 0, 46);

    
    // Display a message
    //dtostrf(pid.pid_output, 0, 0, ui->message_line_2);
    /*
    memset(ui->message_line_2, 0, 21);
    memset(ui->buffer_1, 0, 21);

    itoa(rpi.ip_address[0], ui->buffer_1, 10);
    strcat(ui->message_line_2, ui->buffer_1);
    strcat(ui->message_line_2, DOT);
    
    itoa(rpi.ip_address[1], ui->buffer_1, 10);
    strcat(ui->message_line_2, ui->buffer_1);
    strcat(ui->message_line_2, DOT);
    
    itoa(rpi.ip_address[2], ui->buffer_1, 10);
    strcat(ui->message_line_2, ui->buffer_1);
    strcat(ui->message_line_2, DOT);
    
    itoa(rpi.ip_address[3], ui->buffer_1, 10);
    strcat(ui->message_line_2, ui->buffer_1);    
    */
  } while (u8g2.nextPage());
  t2 = micros();
  ui_us += (t2 - t1);
}

SerialFrame ser;

void Ser_Init(SerialFrame * ser) {
  ser->in_frame = false;
  ser->cmd = SER_UNKNOWN;
  ser->bytes = 0;
  memset(ser->buffer, 0, 21);
}

bool Ser_Feed(SerialFrame * ser, int byte) {
  if (!ser->in_frame) {
    // If not in a frame, interpret first character as command
    ser->cmd = SER_UNKNOWN;
    for (uint8_t i=1; i<11; i++) { // skip 0 (unknown command)
      if (byte == serial_cmd_types[i]) {
        ser->cmd = (SerialCmd) i;
        break;
      }
    }

    ser->in_frame = true;
    return false;
  } else {
    if (byte == ' ' && ser->bytes == 0) {
      // consume spaces between command and data
      return false;
    } else if (byte == '\n' && (ser->bytes > 0) && ser->buffer[ser->bytes-1] == '\r') {
      // If we read a \n and it's preceeded by a \r, decrease the buffer size by 1 and
      // dispatch the event
      ser->bytes--;
      return true;
    } else {
      // Just consume into the buffer, protect against overflow
      if (ser->bytes < 21) { 
        ser->bytes++;
      }
      ser->buffer[ser->bytes - 1] = byte;
      return false;
    }
  }
}

void Ser_On_Complete(SerialFrame * ser, unsigned long now) {
  bool config_found = false;
  uint16_t unpacked_short = 0;
  switch (ser->cmd) {
    case SER_HEARTBEAT:
      last_heartbeat = now;
      if (ser->bytes > 0 && ser->buffer[0] == 'Y') {
        bpq_running = true;
      } else {
        bpq_running = false;
      }
      break;
    case SER_PRINT:
      char * ui_buffer;
      if (ser->buffer[0] == '1') {
        ui_buffer = ui.message_line_1;
      } else if (ser->buffer[0] == '2') {
        ui_buffer = ui.message_line_2;
      } else {
        break;
      }
      memset(ui_buffer, 0, 21);
      memcpy(ui_buffer, &(ser->buffer[2]), ser->bytes - 2);
      ui.display_error = true;
      ui.last_error_display = now;
      break;
    case SER_NET:
      // IP address
      memcpy(rpi.ip_address, ser->buffer, 4);
      break;
    case SER_GET:
    case SER_SET:
      for (uint8_t i=0; i<(NUM_CONFIGS + NUM_TUNABLES); i++) {
        if (memcmp_P(ser->buffer, staticConfigs[i].name, 2) == 0) {
          Serial.flush();
          Serial.print(F("CONFIG ")); Serial.write(ser->buffer, 2);
          Serial.print(" "); 
          if (ser->cmd == SER_SET) {
            unpacked_short = (ser->buffer[3] & 0xff);
            unpacked_short <<= 8;
            unpacked_short |= (ser->buffer[4] & 0xff);
            float new_val = unpacked_short / 1000.0;
            *(dynamicConfigs[i].value) = new_val;
            if (i == 2) {
              // B3 Battery Float Voltage
              charger.state = CHARGE_OFF;
              charger.state_ms = now;
            }
            display_float(new_val, -6, 3, &Serial);
          } else {
            display_float(*(dynamicConfigs[i].value), -6, 3, &Serial);
          }
          Serial.println();
          config_found = true;
          break;
        }
      }
      if (!config_found) {
        Serial.flush();
        Serial.print(F("CONFIG "));
        Serial.write(ser->buffer, 2);
        Serial.println(F(" ?"));
      }
      break;
    case SER_TEST:
      if (ser->bytes == 0) {
        self_test_ms = 30000;
      } else {
        unpacked_short = (ser->buffer[0] & 0xff);
        unpacked_short <<= 8;
        unpacked_short |= (ser->buffer[1] & 0xff);
        self_test_ms = ((uint32_t) unpacked_short & 0xffff) * 1000;
      }
      Serial.flush();
      Serial.print(F("LOG Self-test for "));
      Serial.print(self_test_ms);
      Serial.println(F("ms"));
      break;
    case SER_SAVE_EEPROM:
      Serial.println(F("LOG Saving EEPROM"));
      save_config();
      break;
    case SER_LOAD_EEPROM:
      Serial.println(F("LOG Loading EEPROM"));
      load_config();
      break;
    case SER_RESET_EEPROM:
      Serial.println(F("LOG Resetting EEPROM"));
      reset_config();
      break;
    case SER_VERSION:
      Serial.print(F("LOG Firmware version "));
      Serial.println(F(BUILD_VERSION));
      break;
    default:
      break;
  }
}

unsigned long last_serial_print = 0;

void read_serial(unsigned long now) {
  while (Serial.available() > 0 ) {
    int b = Serial.read();
    if (b > -1) {
      if (Ser_Feed(&ser, b)) {
        Ser_On_Complete(&ser, now);
        Ser_Init(&ser);
      }
    } else {
      break;
    }
  }
}

void send_state_change_event(unsigned long now) {
  Serial.print(F("EVENT STATE_CHANGE State="));
  strcpy_P(label_buffer, (char*)pgm_read_word(&(states[state])));
  Serial.println(label_buffer);
}

void send_serial_data(MetricsReport * report, unsigned long now) {
  Serial.print(F("DATA State="));
  strcpy_P(label_buffer, (char*)pgm_read_word(&(states[state])));
  Serial.print(label_buffer);
  
  Serial.print(F(" Millis="));
  Serial.print(report->period_ms);

  //Serial.print(F(" Samples="));
  //Serial.print(report->adc_samples);

  Serial.print(F(" Battery="));
  display_float(Sensor_smoothedValueFast(&batteryVoltage), -7, 2, &Serial);
  
  Serial.print(F(" Supply="));
  display_float(Sensor_smoothedValueFast(&supplyVoltage), -7, 2, &Serial);

  //Serial.printf(" StateTime=%lu UpTime=%lu DT=%lu", (now - last_state_change), (now - start_time), (now - last_serial_print));
  Serial.print(F(" StateTime=")); Serial.print(now - last_state_change);
  Serial.print(F(" UpTime=")); Serial.print(now - start_time);

  //Serial.print(" Git="); Serial.print(BUILD_VERSION);
  if (state == Initializing) {
    // Avoid printing out weird current and temperature when first starting
    Serial.println();
    return;
  }  

  if (is_battery_connected(state) || is_supply_connected(state)) {
    Serial.print(F(" Temperature="));
    display_float(temperature, -6, 2, &Serial);
    Serial.print(F(" TV="));
    Serial.print(Sensor_instantValue(&tempSense));
  
    Serial.print(F(" AH="));
    display_float(report->load_amp_sec / 3600.0, -8, 2, &Serial);

    // Return the amount of Amp-seconds since the last report. Summing
    // these values over a period of time and converting to Amp-hours
    // will give an accurate view of the charge consumption.
    Serial.print(F(" AmpSecDelta="));
    display_float(report->load_amp_sec_delta, -8, 2, &Serial);

    // Take the accumulated "current_amp_sec" since the last report
    // and divide by the time since the last report. This gives us a
    // good average of the load charge.
    Serial.print(F(" AmpAvg="));  
    display_float(report->load_amp_avg, -8, 2, &Serial);

    // Min/max current seen during this reporting period.
    Serial.print(F(" AmpMax="));
    display_float(report->load_amp_max, -8, 2, &Serial);
    Serial.print(F(" AmpMin="));
    display_float(report->load_amp_min, -8, 2, &Serial);

    // Return the number of Watt-seconds since last report. Summing these
    // values over time gives a good total power consumption.
    Serial.print(F(" WattSecDelta="));
    display_float(report->watt_sec_delta, -8, 2, &Serial);

    // Similarly, divide the accumulated Watt-seconds by the duration of this
    // report. This gives a properly averaged power.
    Serial.print(F(" WattAvg="));
    display_float(report->watt_avg, -8, 2, &Serial);

    //Serial.print(" E="); Serial.print(pid.pid_error); 
    //Serial.print(" I="); Serial.print(pid.pid_integral);
    Serial.print(F(" BattPid=")); Serial.print((uint8_t) pid->pid_output);
    //Serial.print(F(" CC=")); Serial.print((uint8_t) pid_cc.pid_output);
    //Serial.print(F(" CV=")); Serial.print((uint8_t) pid_cv.pid_output);
    Serial.print(F(" ChargeState=")); Serial.print((uint8_t) charger.state);
    
    //Serial.print(F(" BattAH="));
    //display_float(report->battery_amp_sec / 3600.0, -8, 2, &Serial);

    Serial.print(F(" BattAmpSecDelta="));
    display_float(report->battery_amp_sec_delta, -6, 4, &Serial);

    Serial.print(F(" BattAmpAvg="));
    display_float(report->battery_amp_avg, -4, 2, &Serial);

    Serial.print(F(" BattWattSecDelta="));
    display_float(report->battery_watt_sec_delta, -5, 2, &Serial);

    //Serial.print(" U=");
    //Serial.print(ui_us);
    /*
    Serial.print(F(" AgentSeen=")); Serial.print(now - last_heartbeat);

    Serial.print(F(" IP=")); 
    Serial.print(rpi.ip_address[0], 10); 
    Serial.print(".");
    Serial.print(rpi.ip_address[1], 10); 
    Serial.print(".");
    Serial.print(rpi.ip_address[2], 10); 
    Serial.print(".");
    Serial.print(rpi.ip_address[3], 10); 
    */
  }
  Serial.println();
  last_serial_print = now;
}

/******************
 * Setup and loop *
 ******************/

void setup1() {
  pinMode(BATTERY_PIN, OUTPUT);  
  pinMode(SUPPLY_PIN, OUTPUT);  
  pinMode(RPI_PWR_PIN, OUTPUT);

  digitalWrite(SUPPLY_PIN, HIGH);
  digitalWrite(BATTERY_PIN, LOW);
  digitalWrite(RPI_PWR_PIN, LOW);

  pinMode(RPI_POWEROFF_PIN, INPUT_PULLUP);
  Serial.begin(SERIAL_BAUD_RATE);
}

uint8_t val = 0;
void loop1() { 
  digitalWrite(SUPPLY_PIN, LOW);
  digitalWrite(RPI_PWR_PIN, LOW);
 
  Serial.print(F("DATA State=INIT"));
  Serial.print(F(" Samples="));
  Serial.print(metrics.adc_samples);

  Serial.print(F(" Battery="));
  display_float(Sensor_smoothedValueFast(&batteryVoltage), -7, 2, &Serial);
  
  Serial.print(F(" Supply="));
  display_float(Sensor_smoothedValueFast(&supplyVoltage), -7, 2, &Serial);

  Serial.print(F(" Current="));
  display_float(Sensor_smoothedValueFast(&currentSense), -7, 2, &Serial);

  Serial.print(F(" Battery="));
  display_float(Sensor_smoothedValueFast(&chargeSense), -7, 2, &Serial);

  Serial.println();
  delay(100);
}

// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile unsigned long last_now;

uint8_t adc_channel = 0;

SensorRead* sensors[5];

/**
 * Interrupt routine for a level change on MCP100 voltage supervisor.
 * This allows very fast detection of a supply voltage drop below a threshold.
 * 
 * This method is somewhat optimized, but could be optimized further to not
 * use the Arduino digitalRead/digitalWrite. On the scope, from the time the
 * MCP100 drops to 0v and the battery pin goes low is 22µs.
 * 
 * The time from the battery pin going low to the mosfets switching is around 18µs.
*/
ISR (PCINT0_vect) {
  if (state == Standby && digitalRead(12) == 0) {
    if (ui.headless_debug_mode_enabled) {
      return;
    }
    digitalWrite(BATTERY_PIN, LOW);
    ui.error_code = LowSupplyVoltageMessage2;
    state = Backup;
    last_state_change = last_now;
    apply_state(last_now, Backup);
  }
}

/**
 * Pin change interrupt for the encoder button
*/
ISR (PCINT2_vect) {
  bool pin_state = digitalRead(BUTTON_PIN);
  // If the button goes high, the button rose
  if (pin_state) {
    UI_encoder_button_press(&ui, millis());
  }
}


bool setup_done = false;

ISR(ADC_vect) {
  if (!setup_done) {
    return;
  }

  uint16_t v = ADC; 
  Sensor_Update(sensors[adc_channel], v);
  metrics.adc_samples++;

  if (++adc_channel == 5) {
    adc_channel = 0;
  }
  ADMUX = (ADMUX & ~0x07) | adc_channel;
}

EMPTY_INTERRUPT (TIMER1_COMPB_vect);

#define TIMER_500_HZ

void setup() {
  cli();
  // Enable ADC and interrupt vector
  sbi(ADCSRA, ADEN);
  sbi(ADCSRA, ADIE);
  sbi(ADCSRA, ADIF);
  sbi(ADCSRA, ADATE);

  // Set ADC pre-scaler to 128 for 126kHz ADC clock
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

    // Timer/Counter1 Compare Match B
  sbi(ADCSRB, ADTS0);
  sbi(ADCSRB, ADTS2);

  // External AREF
  cbi(ADMUX, REFS0);
  cbi(ADMUX, REFS1);

  sbi(PCICR, PCIE0);        // Enable pin change interrupts on port b
  sbi(PCMSK0, PCINT4);      // Set pin change mask for PB4 (D12, pin 18, MISO)

  sbi(PCICR, PCIE2);        // Enable pin change interrupts on port d
  sbi(PCMSK2, PCINT22);     // Set pin change mask for PD6

  // Timer1 for ADC sampling
  // http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  sbi(TCCR1B, WGM12);       // turn on CTC mode
  sbi(TIMSK1, OCIE1B);      // enable timer compare interrupt


#if defined(TIMER_500_HZ)
  sbi(TCCR1B, CS10);        // no prescaler
  OCR1A = 15999;            // 500Hz
  OCR1B = 15999;
  double w = 0.9747;
#elif defined(TIMER_100_HZ)
  sbi(TCCR1B, CS11);        // prescaler 8   
  OCR1A = 9999;             // 100Hz
  OCR1B = 9999;
  double w = 0.8797;
#else
  sbi(TCCR1B, CS11);        // prescaler 8   
  OCR1A = 19999;            // 50Hz
  OCR1B = 19999;
  double w = 0.7738;
#endif

  sei();

  pinMode(RPI_POWEROFF_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BATTERY_PIN, OUTPUT);  
  pinMode(SUPPLY_PIN, OUTPUT);  
  pinMode(RPI_PWR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  digitalWrite(SUPPLY_PIN, HIGH); // Start with the power supply disconnected
  digitalWrite(BATTERY_PIN, LOW); // If the uC has power, there is a battery! Start off with battery connected
  digitalWrite(RPI_PWR_PIN, LOW); // Start with RPi 12V connected

  //analogReference(EXTERNAL);
  /*pinMode(A0, INPUT); // Battery voltage
  pinMode(A1, INPUT); // Load Current
  pinMode(A2, INPUT); // Temperature voltage
  pinMode(A3, INPUT); // Supply voltage
  pinMode(A4, INPUT); // Charge Current
*/
  Serial.begin(SERIAL_BAUD_RATE);
  
  EEPROM.begin();
  // Initialize EEPROM
#ifdef EEPROM_RESET
  reset_config();
#endif
  uint8_t eeprom_ver = EEPROM.read(0);
  if (eeprom_ver == (uint8_t)0xff) {
      reset_config();
  }
  load_config();

  // This delay is to prevent the display from initializing while ISP is also initializing.
  delay(2000);
  u8g2.begin();
  //u8g2.setBitmapMode(1);
  u8g2.clearDisplay();

  start_time = millis();
  Serial.flush();
  Serial.println();
  Serial.print(F("EVENT STARTUP Version="));
  Serial.print(F(BUILD_VERSION));
  Serial.print(F(" Date="));
  Serial.println(F(BUILD_DATE));

  bool button_held = digitalRead(BUTTON_PIN) == LOW;

  display_info(&ui, start_time, NONE);
  delay(1000);
  
#ifdef TARPN_LOGO
  //display.clearDisplay();
  //display.drawBitmap(0, 8, tarpn_logo_bitmap, 128, 32, WHITE);
  //display.display();
  //delay(1000);
#else
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(4, 4);
  display.setTextSize(3);
  display.println("TARPN");  
  display.display();
  delay(500);
  display.invertDisplay(true);
  delay(500);
  display.invertDisplay(false);
#endif

  // Initialize
  state = Initializing;
  last_state_change = start_time;
  last_adc_read = 0L;
  last_serial_write = 0L;
  last_serial_read = 0L;
  last_heartbeat = 0L;

  //user_sw = Bounce();
  //user_sw.attach(BUTTON_PIN, INPUT);
  //user_sw.interval(BUTTON_PIN);

  PID_Init(&pid_cc, 1.0f, 0.0f);
  PID_Init(&pid_cv, 10.0f, 0.0f);

  Ser_Init(&ser);
  UI_Init(&ui, start_time);
  RPi_Init(&rpi);
  metrics.report = &report;
  charger.state = CHARGE_OFF;
  charger.state_ms = start_time;

  Sensor_Init(&batteryVoltage, &(configs.reference_voltage), &(configs.voltage_factor), &(configs.voltage_offset), w);
  Sensor_Init(&supplyVoltage, &(configs.reference_voltage),  &(configs.voltage_factor), &(configs.voltage_offset), w);
  Sensor_Init(&tempSense, &(configs.reference_voltage), &ONE, NULL, w);
  Sensor_Init(&currentSense, &(configs.reference_voltage), &(configs.current_factor), &(configs.current_offset), w);
  Sensor_Init(&chargeSense, &(configs.reference_voltage), &(configs.batt_current_factor), &(configs.batt_current_offset), w);

  sensors[0] = &batteryVoltage;
  sensors[1] = &currentSense;
  sensors[2] = &tempSense;
  sensors[3] = &supplyVoltage;
  sensors[4] = &chargeSense;

  // If button held throughput startup, enter headless mode and flash status LED 
  if (digitalRead(BUTTON_PIN) == LOW && button_held) {
    ui.headless_debug_mode_enabled = true;
    for(uint8_t i=0; i<3; i++) {
      digitalWrite(STATUS_LED, HIGH);
      delay(50);
      digitalWrite(STATUS_LED, LOW);
      delay(50);
    }
  }

  //if (!ui.headless_debug_mode_enabled) {
  //  sbi(ADCSRA, ADSC); // trigger the next ADC
  //}
  setup_done = true;
}

void Metrics_Update(Metrics * metrics, double elapsed_sec) {
      /*
      Voltage: batteryVoltage and supplyVoltage
      Current: AmpAvg (amp_sec / dt), AmpMax (peak_current)
      Power: load_watts WattAvg (watt_sec / dt). 
    */
    // integrate the current over the sample period
  
  double current = Sensor_smoothedValueFast(&currentSense);
  double amp_sec = max(0, current * elapsed_sec);
  metrics->current_amp_seconds += amp_sec;
  //metrics->peak_current = max(metrics->peak_current, Sensor_maxValue(&currentSense)); //max(metrics->peak_current, current);
  //metrics->min_current = min(metrics->)
  if (is_supply_connected(state)) {
    metrics->current_watt_seconds += (amp_sec * Sensor_smoothedValueFast(&supplyVoltage));
  } else {
    metrics->current_watt_seconds += (amp_sec * Sensor_smoothedValueFast(&batteryVoltage));
  }

  current = Sensor_smoothedValueFast(&chargeSense);
  amp_sec = current * elapsed_sec;
  metrics->battery_amp_seconds += amp_sec;
  metrics->battery_watt_seconds_acc += (amp_sec * Sensor_smoothedValueFast(&batteryVoltage));

  //metrics->adc_samples++;
}

void Metrics_Report(Metrics * metrics, unsigned long now) {
  metrics->report->period_ms = (now - metrics->last_report_ms);
  double report_period_s = metrics->report->period_ms / 1000.0;

  metrics->report->load_amp_sec_delta = (metrics->current_amp_seconds - metrics->report->load_amp_sec);
  metrics->report->load_amp_sec = metrics->current_amp_seconds;
  metrics->report->load_amp_avg = metrics->report->load_amp_sec_delta / report_period_s;
  metrics->report->load_amp_max = Sensor_offsetAndScaleValue(&currentSense, currentSense.maxValue);
  metrics->report->load_amp_min = Sensor_offsetAndScaleValue(&currentSense, currentSense.minValue);

  metrics->report->watt_sec_delta = (metrics->current_watt_seconds - metrics->report->watt_sec);
  metrics->report->watt_sec = metrics->current_watt_seconds;
  metrics->report->watt_avg = metrics->report->watt_sec_delta / report_period_s;

  metrics->report->battery_amp_sec_delta = (metrics->battery_amp_seconds - metrics->report->battery_amp_sec);
  metrics->report->battery_amp_sec = metrics->battery_amp_seconds;
  metrics->report->battery_amp_avg = metrics->report->battery_amp_sec_delta / report_period_s;
  metrics->report->battery_watt_sec_delta = (metrics->battery_watt_seconds_acc - metrics->report->battery_watt_sec);
  metrics->report->battery_watt_sec = metrics->battery_watt_seconds_acc;
  
  metrics->last_report_ms = now;
  
  metrics->report->adc_samples = metrics->adc_samples;
  metrics->adc_samples = 0;
  Sensor_resetMinMax(&currentSense);
}

void loop() {
  unsigned long now = millis();
  last_now = now;

  // Evaluate the state machine and change the output state
  boolean changed_state = next_state(now, temperature, &ui);
  apply_state(now, state);
  if (changed_state) {
    send_state_change_event(now);
    last_serial_write = now;
  }

  // Only enter low power mode if we're not headless
  if (state == ShutdownComplete && !ui.headless_debug_mode_enabled) {
    // Low power mode will mess with millis(). The ShutdownComplete state ignores time
    // and only considers the instantaneous value of the ADC for the battery voltage.
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

    // Flash the status LED so users know we're alive
    digitalWrite(STATUS_LED, HIGH); 
    delay(50);
    digitalWrite(STATUS_LED, LOW);
    
    // After one sleep, clear the display
    //display.clearDisplay();
    //display.display();
    //display.dim(true);
    u8g2.clearDisplay();
    //u8g2.setPowerSave(1);

    // Read the battery voltage to give us a chance to wakeup
    //read_voltage(A0, &batteryVoltage, time_us);
    return;
  }

  
  // Only read the ADCs every so often
  unsigned long dt = now - last_adc_read;
  if (dt > 100) {
    temperature = calculate_temp();
    double elapsed_sec = dt / 1000.;
    Metrics_Update(&metrics, elapsed_sec);
    double chargeVoltage = Sensor_smoothedValueFast(&batteryVoltage);
    double chargeCurrent = Sensor_smoothedValueFast(&chargeSense);

    switch (charger.state) {
      case CHARGE_INIT:
        if ((now - charger.state_ms) > MIN_CHARGE_STATE_MS) { // TODO increase this to some minutes
          charger.state = CHARGE_CC;
          charger.state_ms = now;
          pid = &pid_cc;
        }
        break;
      case CHARGE_CC:
        if ((now - charger.state_ms) > MIN_CHARGE_STATE_MS && (chargeVoltage - configs.batt_float_voltage) > 0.01) {
          charger.state = CHARGE_CC_HOLD;
          charger.state_ms = now;
          //pid_cv.pid_output = pid_cc.pid_output;
        }
        PID_Update(&pid_cc, dt, 1.0, -1.0 * chargeCurrent);
        break;
      case CHARGE_CC_HOLD:
        if ((now - charger.state_ms) > MIN_CHARGE_STATE_MS) {
          charger.state = CHARGE_CV;
          charger.state_ms = now;
          pid = &pid_cv;
          pid_cv.pid_output = pid_cc.pid_output;
        }
        //PID_Update(&pid_cc, now, 1.0, -1.0 * chargeCurrent);
        //PID_Update(&pid_cv, now, configs.batt_float_voltage, chargeVoltage);
        break;
      case CHARGE_CV:
        // Just trickle charge forever
        PID_Update(&pid_cv, dt, configs.batt_float_voltage, chargeVoltage);
        break;   
      case CHARGE_OFF:
      default:
        pid_cc.pid_output = 0.0;
        pid_cv.pid_output = 0.0;
        if ((now - charger.state_ms) > MIN_CHARGE_STATE_MS && state == Standby) {
          if (configs.batt_float_voltage > chargeVoltage) {
            charger.state = CHARGE_INIT;
            charger.state_ms = now;
          }
        }
        break;
    }

    RPi_Update(&rpi, now);
    last_adc_read = now;
  }

  if ((now - last_serial_read) > 200) {
    read_serial(now);
    last_serial_read = now;
  }

  // Output to Serial occasionally
  if ((now - last_serial_write) > 2000) {
    Metrics_Report(&metrics, now);
    send_serial_data(metrics.report, now);
    ui_us = 0;
    last_serial_write = now;
  }

  // Read from the encoder
  UI_encoder_position(&ui, now, myEnc.read());
  //user_sw.update();
  //if (user_sw.rose()) {
  //  UI_encoder_button_press(&ui, now);
  //}

  // Handle any user event
  UserEvent event = UI_next_event(&ui, now);

  // Choose a display mode
  if (event != NONE || (now - ui.last_display_update_ms) > 500) {
    if (ui.config_mode_enabled) {
      display_config(&ui, now, event);
    } else {
      display_main(&ui, now, event);
    }
    ui.last_display_update_ms = now;
  }
}
