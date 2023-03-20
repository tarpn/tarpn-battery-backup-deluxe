#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce2.h>
// #define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <LowPower.h>

#include "config.h"
#include "voltage_read.h"
#include "free_memory.h"

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
#define RPI_SHUTDOWN_PIN 4  // when pulled low, signals the RPi to shutdown  
#define RPI_POWEROFF_PIN 6  // pulled high, low, high when RPi is ready for power off 
#define STATUS_LED 7
#define RPI_PWR_PIN 8
#define BATTERY_PIN 9
#define SUPPLY_PIN 10
#define BUTTON_PIN 11

#define WARMUP_TIME_MS                2000    // Collect samples from the ADC for this long after startup
#define STANDBY_LOW_VOLTAGE           12.5    // Below this PSU voltage, connect the battery
#define BATTERY_HIGH_VOLTAGE_CUTOFF   15.0    // At this voltage, disconnect the battery to protect loads
#define BATTERY_HIGH_VOLTAGE_RECOVERY 14.0    // At this voltage, reconnect the battery after high cutoff
#define BATTERY_LOW_VOLTAGE_CUTOFF    11.0    // At this voltage, disconnect the battery to protect battery
#define BATTERY_LOW_VOLTAGE_RECOVERY  12.0    // At this voltage, reconnect the battery after low cutoff
//#define BATTERY_LOW_VOLTAGE_SHUTDOWN  10.0    // At this voltage, the system will shutdown the RPI (TODO sleep?)
#define BATTERY_HIGH_VOLTAGE_TRIP_MS  10
#define BATTERY_LOW_VOLTAGE_TRIP_MS   3000
#define BATTERY_RECOVERY_MS           10000
#define OVER_TEMP_CUTOFF              60      // At this temperature in celcius, disconnect battery
#define OVER_TEMP_RECOVERY            50      // 
#define OVER_TEMP_RECOVERY_MS         10000   // 

#define ABSOLUTE_MAX_VOLTAGE 18.0
#define ABSOLUTE_MAX_TEMP 100.0
//#define EEPROM_RESET 1

#define ADC_ZENER_VOLTAGE     4.84    // Measured from D1
#define VOLTAGE_FACTOR        0.167   // For 10k 49.9k resistor divider use 0.167. For 10k 100k, use 0.091
#define CURRENT_FACTOR        0.1204  // Shunt resistor value times gain factor (0.003 * 40 = 0.120)
#define CURRENT_OFFSET        2.400   // Op amp uses a 2.4V reference to allow for negative voltages

struct StatefulConfigs {
  // Tunables
  float reference_voltage;
  float voltage_factor;
  float current_factor;
  float current_offset;
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
} config_defaults = {
  // Tunables
  ADC_ZENER_VOLTAGE, 
  VOLTAGE_FACTOR, 
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
  OVER_TEMP_RECOVERY_MS
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
  RPiShutdown,      // 12
  ShutdownComplete  // 13
};

const char state_0[] PROGMEM = "Null";
const char state_1[] PROGMEM = "Initializing";
const char state_2[] PROGMEM = "Standby";
const char state_3[] PROGMEM = "Backup";
const char state_4[] PROGMEM = "Recovery";
const char state_5[] PROGMEM = "BatteryLow";
const char state_6[] PROGMEM = "BatteryLowTrip";
const char state_7[] PROGMEM = "BatteryHigh";
const char state_8[] PROGMEM = "BatteryHighTrip";
const char state_9[] PROGMEM = "OverTemp";
const char state_10[] PROGMEM = "OverTempRecovery";
const char state_11[] PROGMEM = "BeginShutdown";
const char state_12[] PROGMEM = "RPiShutdown";
const char state_13[] PROGMEM = "ShutdownComplete";

const char* const states[] PROGMEM = {
  state_0, state_1, state_2, state_3, state_4, state_5,
  state_6, state_7, state_8, state_9, state_10, state_11, 
  state_12, state_13
};

enum Messages {
  NoMessage,
  SupplyRecovered,
  LowSupplyVoltageMessage,
  LowBatteryVoltageMessage,
  HighBatteryVoltageMessage,
  OverTempMessage,
  EEPROMSavedMessage,
  EEPROMResetMessage,
  RPIShutdownMessage,
  ShutdownMessage
};

enum UserEvent {
  NONE,
  BUTTON_PRESS,
  LONG_PRESS,
  SCROLL_RIGHT,
  SCROLL_LEFT
};

const char label_0[] PROGMEM = "Battery Voltage";
const char label_1[] PROGMEM = "Supply Voltage";
const char label_2[] PROGMEM = "Load Current";
const char label_3[] PROGMEM = "Temperature";
const char* const labels[] PROGMEM = {label_0, label_1, label_2, label_3};
char label_buffer[16];


float ONE = 1.0;
struct VoltageRead * batteryVoltage = new VoltageRead(&(configs.reference_voltage), &(configs.voltage_factor));
struct VoltageRead * supplyVoltage = new VoltageRead(&(configs.reference_voltage),  &(configs.voltage_factor));
struct VoltageRead * tempSense = new VoltageRead(&(configs.reference_voltage), &ONE);
struct VoltageRead * currentSense = new VoltageRead(&(configs.reference_voltage), &(configs.current_factor), &(configs.current_offset));

/**
 * A nicely packed structure for storing the configurations in the EEPROM.
 * All values are scaled up by 1000 (except timing values) and stored as 16-bit unsigned ints.
 */
struct PackedConfig {
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
};

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
}

void fromPackedConfig(StatefulConfigs * configs, PackedConfig * packed) {
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

void save_config() {
  PackedConfig to_store;
  toPackedConfig(&configs, &to_store);
  EEPROM.put(1, to_store);
}

void reset_config() {
  for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.update(i, 0);
  }
  EEPROM.update(0x00, 1);
  PackedConfig to_store;
  toPackedConfig(&config_defaults, &to_store);
  EEPROM.put(1, to_store);
}

void load_config() {
  PackedConfig packed;
  EEPROM.get(1, packed);
  fromPackedConfig(&configs, &packed);
}

long start_time;
State state;
long last_state_change;
bool switch_closed;
long last_switch_open_time;
long last_switch_closed_time;
double temperature;
long last_adc_read;
long last_serial_write;

Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);
int last_encoder_pos = 0;
Bounce user_sw;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
long next_display_change;
bool config_mode_enabled = false;
bool long_press_captured = false;
uint8_t display_choice = 0;
Messages error_code = NoMessage;
Messages display_error_code = NoMessage;
long last_error_display = 0;

double current_amp_seconds = 0;
double current_watt_seconds = 0;

long last_blink_off = -1; // if not -1, flash off
long last_blink_on = -1;  // if not -1, flash on

bool rpi_powered_on = false;

void read_rpi_power_state() {
  rpi_powered_on = digitalRead(RPI_POWEROFF_PIN);
}

void shutdown_rpi() {
  pinMode(RPI_SHUTDOWN_PIN, OUTPUT);
  delay(1);
  digitalWrite(RPI_SHUTDOWN_PIN, LOW);
  delay(100);
  pinMode(RPI_SHUTDOWN_PIN, INPUT);
}

/**
 * Basic blink/pulse routine. On for 400ms, off for 100ms. Used to flash a single character on the OLED on/off.
 */
boolean do_blink(long now, uint16_t on_ms, uint16_t off_ms) {
  if (last_blink_off == -1 && last_blink_on == -1) {
    last_blink_on = now;
    return true;
  }

  if (last_blink_on == -1) {
    // flash is off
    if ((now - last_blink_off) > off_ms) {
      last_blink_on = now;
      last_blink_off = -1;
      return true;
    }
    return false;
  }

  if (last_blink_off == -1) {
    if ((now - last_blink_on) > on_ms) {
      last_blink_on = -1;
      last_blink_off = now;
      return false;
    } 
    return true;
  }

  return false;
}

void read_voltage(uint8_t pin, VoltageRead * read) {
  read->sensorValue = (uint16_t) analogRead(pin);
  read->filter->filter(read->sensorValue);
  read->fastFilter->filter(read->sensorValue);
}

/**
 * Calculate the temperature using Steinhart–Hart equation. 49.9k 10k resistor divider is hard coded in here.
 */
double read_temp() {
  // We still use the EMWA filter in VoltageRead, but calclulate the real value here.
  double res = (49900.0 * tempSense->adcVoltageSlow()) / (batteryVoltage->smoothedValueSlow() - tempSense->adcVoltageSlow());
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
bool next_state(long now, double temp) {
  State current_state = state;

  if (current_state != Initializing &&
      current_state != OverTemp &&
      current_state != BeginShutdown &&
      current_state != ShutdownComplete &&
      temp >= configs.overtemp_cutoff) {
    // Over temp logic applies to most states, so do it separately.
    last_state_change = now;
    error_code = OverTempMessage;
    state = OverTemp;
    return true;
  }

  double load_voltage_drop_max = currentSense->instantValue() * 0.05;
  int current_state_ms = now - last_state_change;
  State next_state = Null;

  switch (current_state) {
  case Null:  
    // Shouldn't be possible, but just transition to Initializing
    next_state = Initializing;
    break;
  case Initializing:
    if (current_state_ms >= WARMUP_TIME_MS) {
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
    if ((supplyVoltage->instantValue() + load_voltage_drop_max) < configs.standby_voltage) {
      error_code = LowSupplyVoltageMessage;
      next_state = Backup;
    }
    break;
  case Backup:
    if (batteryVoltage->smoothedValueSlow() < configs.batt_low_voltage_cutoff) {
      next_state = BatteryLow;
    } else if (batteryVoltage->smoothedValueSlow() >= configs.batt_high_voltage_cutoff) {
      next_state = BatteryHigh;
    } else if (supplyVoltage->instantValue() >= configs.standby_voltage) {
      next_state = Recovery;
    }
    break;
  case Recovery:
    if (supplyVoltage->instantValue() < configs.standby_voltage) {
      next_state = Backup;
    } else if (current_state_ms >= 3000) {
      error_code = SupplyRecovered;
      next_state = Standby;
    } 
    break;
  case BatteryLow:
    if (current_state_ms >= configs.batt_low_voltage_trip_ms) {
      error_code = LowBatteryVoltageMessage;
      next_state = BatteryLowTrip;
    } else if (batteryVoltage->smoothedValueSlow() >= configs.batt_low_voltage_recovery) {
      next_state = Backup;
    }
    break;
  case BatteryLowTrip:
    if (current_state_ms >= 10000 && batteryVoltage->smoothedValueSlow() < 10.0) {
      error_code = ShutdownMessage;
      next_state = BeginShutdown;
      shutdown_rpi();
    } else if (batteryVoltage->smoothedValueSlow() >= configs.batt_low_voltage_recovery) {
      next_state = Backup;
    }
    break;
  case BatteryHigh:
    if (current_state_ms >= configs.batt_high_voltage_trip_ms) {
      error_code = HighBatteryVoltageMessage;
      next_state = BatteryHighTrip;
    } else if (batteryVoltage->smoothedValueSlow() < configs.batt_high_voltage_recovery) {
      next_state = Backup;
    }
    break;
  case BatteryHighTrip:
    if (batteryVoltage->smoothedValueSlow() < configs.batt_high_voltage_recovery) {
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
    if (current_state_ms > 10000 || !rpi_powered_on) {
      error_code = RPIShutdownMessage;
      next_state = RPiShutdown;
    }
    break;
  case RPiShutdown:
    if (current_state_ms > 10000) {
      // Transition to low power mode here
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(2);
      display.print(F("Shutting\nDown..."));
      display.display();
      display.dim(true);
      next_state = ShutdownComplete;
    }
    break;
  case ShutdownComplete:
    if (batteryVoltage->instantValue() >= configs.batt_low_voltage_recovery) {
      display.dim(false);
      next_state = Backup;
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
      return true;
    default:
      return false;
  }
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

bool in_shutdown_sequence(State state) {
  switch (state) {
    case BeginShutdown:
    case RPiShutdown:
    case ShutdownComplete:
      return true;
    default:
      return false;
  }
}

/**
 * For the current state, determine the appropriate digital outputs. 
 */
void apply_state(long now, State state) {
  switch (state) {
    case Backup:
    case Recovery:
    case BatteryLow:
    case BatteryHigh:
      // RPi is on, battery is connected, supply is disconnected
      digitalWrite(RPI_PWR_PIN, LOW);
      digitalWrite(BATTERY_PIN, LOW);
      digitalWrite(SUPPLY_PIN, HIGH);
      digitalWrite(STATUS_LED, HIGH);
      break;
    case BatteryLowTrip:
    case OverTemp:
    case OverTempRecovery:
    case BatteryHighTrip:
      // RPi is on, battery is disconnected, supply is disconnected
      digitalWrite(RPI_PWR_PIN, LOW);
      digitalWrite(BATTERY_PIN, HIGH);
      digitalWrite(SUPPLY_PIN, HIGH);
      digitalWrite(STATUS_LED, LOW);
      break;
    case BeginShutdown:
      // RPi is on, battery is disconnected, supply is disconnected
      digitalWrite(RPI_PWR_PIN, LOW);
      digitalWrite(BATTERY_PIN, HIGH);
      digitalWrite(SUPPLY_PIN, HIGH);
      digitalWrite(STATUS_LED, LOW);
      break;
    case ShutdownComplete:
      // RPi is off, battery is disconnected, supply is disconnected
      digitalWrite(RPI_PWR_PIN, HIGH);
      digitalWrite(BATTERY_PIN, HIGH);
      digitalWrite(SUPPLY_PIN, HIGH);
      digitalWrite(STATUS_LED, LOW);
      break;
    case Initializing:
    case Standby:
    default:
      // RPi is on, battery is disconnected, supply is connected
      digitalWrite(RPI_PWR_PIN, LOW);
      digitalWrite(BATTERY_PIN, HIGH);
      digitalWrite(SUPPLY_PIN, LOW);
      digitalWrite(STATUS_LED, LOW);
      break;
  }
}

/*********************************
 *                               *
 * Configuration and UI Routines *
 *                               *
 *********************************/

uint8_t config_depth = 0;
uint8_t config_selection_0 = 0;
uint8_t config_selection_1 = 0;
uint8_t config_selection_2 = 0;

void display_info(long now, UserEvent event) {
  if (event == BUTTON_PRESS) {
    config_depth = 0;
    return;
  }

  int uptime_sec = (int) ((now - start_time) / 1000.0);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(F("TARPN BBD  "));
  display.println(uptime_sec);
  display.println(F("David Arthur, K4DBZ"));
  display.println(BUILD_DATE);
  display.print(GIT_REV);
  display.print(F(" "));
  display.print(freeMemory());
  display.display();
}

// These are allocated in SRAM since they need reference to global objects
struct DynamicConfig {
  float * value;
  VoltageRead * derived;
};

// These are stored in Flash RAM (PROGMEM)
struct StaticConfig {
  const char name[3];
  const char label_1[20];
  const char label_2[20];
  int8_t hi;  // highest power of 10 to change
  int8_t lo;  // lowest power of 10 to change
  uint8_t decimal_places;  // digits after decimal
} staticConfigSRAM {"", "", "", 0, 0, 0};

#define NUM_CONFIGS 11
#define NUM_TUNABLES 3

struct DynamicConfig dynamicConfigs[NUM_CONFIGS + NUM_TUNABLES] = {
  {&(configs.standby_voltage), NULL},
  {&(configs.batt_recovery_ms), NULL},
  {&(configs.batt_high_voltage_cutoff), NULL},
  {&(configs.batt_high_voltage_recovery), NULL},
  {&(configs.batt_high_voltage_trip_ms), NULL},
  {&(configs.batt_low_voltage_cutoff), NULL},
  {&(configs.batt_low_voltage_recovery), NULL},
  {&(configs.batt_low_voltage_trip_ms), NULL},
  {&(configs.overtemp_cutoff), NULL},
  {&(configs.overtemp_recovery), NULL},
  {&(configs.overtemp_recovery_ms), NULL},
  {&(configs.current_factor), currentSense},
  {&(configs.current_offset), currentSense},
  {&(configs.reference_voltage), currentSense}
};

const struct StaticConfig staticConfigs[NUM_CONFIGS + NUM_TUNABLES] PROGMEM = {
  {"B1", "Battery", "Standby Voltage", 0, -1, 1},
  {"B2", "Battery", "Recovery ms", 3, 0, 0},
  {"H1", "Battery High", "Voltage Cutoff", 0, -1, 1},
  {"H2", "Battery High", "Voltage Recovery", 0, -1, 1},
  {"H2", "Battery High", "Trip Time ms", 3, 0, 0},
  {"L1", "Battery Low", "Voltage Cutoff", 0, -1, 1},
  {"L2", "Battery Low", "Voltage Recovery", 0, -1, 1},
  {"L3", "Battery Low", "Trip Time ms", 2, 0, 0},
  {"T1", "Temperature", "Cutoff Temp", 1, -1, 1},
  {"T2", "Temperature", "Recovery Temp", 1, -1, 1},
  {"T3", "Temperature", "Recovery Time ms", 3, 0, 0},
  {"X1", "Current Sense", "", -1, -3, 3},
  {"X2", "Current Offset", "", 0, -3, 3},
  {"X3", "ADC Voltage", "", 0, -3, 3}
};

char float_buffer[10];
void display_float(float value, uint8_t width, uint8_t precision, Print * printer) {
  dtostrf(value, width, precision, float_buffer);
  (*printer).print(float_buffer);
}

DynamicConfig dynamicConfig;
void adjust_set_levels(long now, UserEvent event, bool levelsOrTune, uint8_t choices_len) {  
  // For each config, need the display string, valid units to cycle through, and location
  // of digit to flash.

  // Select the correct DynamicConfig and load the StaticConfig from PROGMEM
  if (levelsOrTune) {
    // Set Levels
    dynamicConfig = dynamicConfigs[config_selection_1];
    memcpy_P(&staticConfigSRAM, &staticConfigs[config_selection_1], sizeof(StaticConfig));
  } else {
    // Tune
    dynamicConfig = dynamicConfigs[NUM_CONFIGS + config_selection_1];
    memcpy_P(&staticConfigSRAM, &staticConfigs[NUM_CONFIGS + config_selection_1], sizeof(StaticConfig));
  }

  uint8_t digit_choices = 1 + staticConfigSRAM.hi - staticConfigSRAM.lo;
  uint8_t digit_idx = config_selection_2 % digit_choices;
  int8_t exponent = (staticConfigSRAM.hi - digit_idx);
  double adj = pow(10.0, exponent);
  double val = *(dynamicConfig.value);

  if (event == SCROLL_LEFT) {
    val -= adj;
  }
  if (event == SCROLL_RIGHT) {
    val += adj;
  }
  *(dynamicConfig.value) = val;

  if (event == BUTTON_PRESS) {
    if (config_selection_2 == (digit_choices - 1)) {
      config_selection_2 = 0;
      config_depth = 1;
    } else {
      config_selection_2++;
      return;
    }
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(staticConfigSRAM.name);
  display.print(F(" ")); 
  display_float(val, 5, staticConfigSRAM.decimal_places, &display);
  uint8_t blink_pos;
  if (!do_blink(now, 400, 100)) {
    uint8_t last_place = 84; // width is 5 characters (60px), right aligned
    blink_pos = last_place - (exponent * 12);
    if (staticConfigSRAM.decimal_places > 0) {
      blink_pos -= (12 + 12 * staticConfigSRAM.decimal_places);
    }
    if (exponent < 0) {
      blink_pos += 12;
    }
    display.fillRect(blink_pos, 0, 12, 16, SSD1306_BLACK);
  }
  display.setCursor(108, 0);
  display.print((char) 0x1B); // <- arrow
  display.setCursor(0, 16);
  display.setTextSize(1);
  display.println(staticConfigSRAM.label_1);
  display.println(staticConfigSRAM.label_2);
  if (dynamicConfig.derived != NULL) {
    double derived = dynamicConfig.derived->smoothedValueFast();
    if (derived < 0.0) {
      display.setCursor(74, 24);
    } else {
      display.setCursor(80, 24);
    }
    display_float(derived, 4, 2, &display);
  }
  display.display();
}

/**
 * Render the "Set Levels" sub-menu. The configurable values are defined in "dynamicConfigs" and "staticConfigs".
*/
void display_set_levels(long now, UserEvent event, bool levelsOrTune, uint8_t choices_len) {
  if (event == SCROLL_RIGHT) {
    cycle_selection(&config_selection_1, choices_len + 1, true);
  }
  if (event == SCROLL_LEFT) {
    cycle_selection(&config_selection_1, choices_len + 1, false);
  }
  if (event == BUTTON_PRESS) {
    if (config_selection_1 == choices_len) {
      config_selection_1 = 0;
      config_depth = 0;
      return;
    }

    config_depth = 2;
    return;
  }

  if (config_selection_1 == choices_len) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(F("Return"));
    display.drawBitmap(80, 0, return_glyph, 12, 18, WHITE);
    display.display();
    return;
  }

  // Select the correct DynamicConfig and load the StaticConfig from PROGMEM
  DynamicConfig dynamicConfig;
  if (levelsOrTune) {
    // Set Levels
    dynamicConfig = dynamicConfigs[config_selection_1];
    memcpy_P(&staticConfigSRAM, &staticConfigs[config_selection_1], sizeof(StaticConfig));
  } else {
    // Tune
    dynamicConfig = dynamicConfigs[NUM_CONFIGS + config_selection_1];
    memcpy_P(&staticConfigSRAM, &staticConfigs[NUM_CONFIGS + config_selection_1], sizeof(StaticConfig));
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(staticConfigSRAM.name);
  display.print(" ");
  display_float(*(dynamicConfig.value), 5, staticConfigSRAM.decimal_places, &display);
  display.setCursor(108, 0);
  display.print((char) 0x12); // up/down arrow
  display.setCursor(0, 16);
  display.setTextSize(1);
  display.println(staticConfigSRAM.label_1);
  display.println(staticConfigSRAM.label_2);
  if (dynamicConfig.derived != NULL) {
    double derived = dynamicConfig.derived->smoothedValueFast();
    if (derived < 0.0) {
      display.setCursor(74, 24);
    } else {
      display.setCursor(80, 24);
    }
    display_float(derived, 4, 2, &display);
  }
  display.display();
}


void display_config(long now, UserEvent event) {
  if (event == LONG_PRESS) {
    config_mode_enabled = false;
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);

  uint8_t choices = 7; 
  if (config_depth == 0) {
    if (event == SCROLL_RIGHT) {
      cycle_selection(&config_selection_0, choices, true);
    }
    if (event == SCROLL_LEFT) {
      cycle_selection(&config_selection_0, choices, false);
    }
    if (event == BUTTON_PRESS) {
      switch (config_selection_0) {
        case 3:
          save_config();
          error_code = EEPROMSavedMessage;
          break;
        case 4:
          reset_config();
          error_code = EEPROMResetMessage;
          break;
        case 5:
          shutdown_rpi();
          error_code = RPIShutdownMessage;
          break;
        case 6:
          config_selection_1 = 0;
          config_mode_enabled = false;
          break;
        default:
          config_depth = 1;
          config_selection_1 = 0;
          break;
      }
      return;
    }
    
    display.setCursor(4, 8);
    display.setTextSize(2);
    switch (config_selection_0) {
      case 0:
        display.println(F("Set Limits"));
        break;
      case 1:
        display.println(F("Tune"));
        break;
      case 2:
        display.println(F("Info"));
        break;
      case 3:
        display.setCursor(4, 0);
        display.print(F("Save"));
        display.setCursor(4, 16);
        display.print(F("EEPROM"));
        break;
      case 4:
        display.setCursor(4, 0);
        display.print(F("Reset"));
        display.setCursor(4, 16);
        display.print(F("EEPROM"));
        break;
      case 5:
        display.setCursor(4, 0);
        display.print(F("Shutdown"));
        display.setCursor(4, 16);
        display.print(F("RPi"));
        break;
      case 6:
        display.print(F("Return"));
        display.drawBitmap(84, 8, return_glyph, 12, 18, WHITE);
        break;
      
    } 
    display.display();
    return;
  }

  if (config_depth == 1) {
    switch (config_selection_0) {
      case 0:
        display_set_levels(now, event, true, NUM_CONFIGS);
        break;
      case 1:
        display_set_levels(now, event, false, NUM_TUNABLES);
        break;
      case 2:
        display_info(now, event);
        break;
    } 
    return;
  }

  if (config_depth == 2) {
    if (config_selection_0 == 0) {
        adjust_set_levels(now, event, true, NUM_CONFIGS);
    } else if (config_selection_0 == 1) {
        adjust_set_levels(now, event, false, NUM_TUNABLES);
    }
  }
}

void display_icons(long now) {
  display.drawBitmap(96, 0, battery_glyph, 12, 18, WHITE);
  display.drawBitmap(116, 0, ac_glyph, 12, 18, WHITE);
  if (rpi_powered_on) {
    display.drawBitmap(110, 12, pi_glyph, 4, 4, WHITE);
  }

  bool batt = is_battery_connected(state);
  bool supp = is_supply_connected(state); 
  bool draw_radio = batt | supp;
  if (do_blink(now, 1000, 200)) {
    if (batt) {
      display.drawLine(98, 19, 98, 27, WHITE); // |
      display.drawLine(99, 28, 103, 28, WHITE); // --
    } else if (supp) {
      display.drawLine(126, 19, 126, 27, WHITE); // |
      display.drawLine(125, 28, 121, 28, WHITE); // --
    } else {
      // neither connected
      draw_radio = true;
    }
  }

  if (draw_radio) {
    display.drawBitmap(104, 20, radio_glyph, 16, 12, WHITE);
  }
}

void display_message(long now, UserEvent event) {
  display.clearDisplay();  
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  
  display.setTextSize(3);
  display.print("E");
  display.print(display_error_code);

  display.setTextSize(1);
  display.setCursor(0, 24);

  switch (display_error_code) {
    case NoMessage:
      break;
    case SupplyRecovered:
      display.println(F("Supply Restored"));
      break;
    case LowSupplyVoltageMessage:
      display.print(F("No Supply"));
      break;
    case LowBatteryVoltageMessage:
      display.print(F("Low Battery"));
      break;
    case HighBatteryVoltageMessage:
      display.print(F("High Battery"));
      break;
    case OverTempMessage:
      display.print(F("Over Temp"));
      break;
    case EEPROMSavedMessage:
      display.print(F("EEPROM saved"));
      break;
    case EEPROMResetMessage:
      display.print(F("EEPROM reset"));
      break;
    case RPIShutdownMessage:
      display.print(F("RPi Shutdown"));
      break;
    case ShutdownMessage:
      display.print(F("Shutdown RPi"));
      break;
    default:
      display.print(F("Unknown"));
      break;
  }

  display_icons(now);
  display.display();
}

void display_main(long now, UserEvent event) {
  if (event == SCROLL_RIGHT) {
    next_display_change = now;
  } 
  if (event == SCROLL_LEFT) {
    next_display_change = now;
    display_choice -= 2;
  }
  if (event == BUTTON_PRESS) {
    config_mode_enabled = true;
  }

  if (now >= next_display_change) {
    // Automatically cycle through
    display_choice++;
    next_display_change = now + 5000;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  VoltageRead * display_voltage;
  uint8_t label_index = display_choice % 4;
  
  switch (display_choice % 4) {
    case 0: // Battery
      display_voltage = batteryVoltage;
      break;
    case 1: // Supply
      display_voltage = supplyVoltage;
      break;
    case 2: // Current
      display_voltage = currentSense;
      break;
    case 3: // Temp
      display_voltage = tempSense;
    default:
      break;
  }

  // Large output
  display.setCursor(0, 0);
  display.setTextSize(3);
  if (display_choice % 4 == 3) {
    display.print(temperature, 2);
  } else {
    display.print(display_voltage->smoothedValueSlow(), 2);
  }

  display.setTextSize(1);
  display.setCursor(0, 24);
  strcpy_P(label_buffer, (char*)pgm_read_word(&(labels[label_index])));
  display.print(label_buffer);
  display_icons(now);
  
  /*
  // Debug output (on the right)
  display.setTextSize(1);
  display.setCursor(96, 0);
  display.print(F("|E"));
  display.print(state);
  display.setCursor(96, 8);
  display.print(F("|"));
  display.print(rpi_powered_on);
  //display.print(display_voltage->sensorValue); 
  display.setCursor(96, 16);
  display.print(F("|"));
  display_float(display_voltage->adcVoltageFast(), 4, 2, &display);
  display.setCursor(96, 24);
  display.print(F("|"));
  display_float(display_voltage->adcVoltageSlow(), 4, 2, &display);
  */  
  display.display();
}

/**
 * Write out the current state, voltages, current, and time to the serial line.
 * Since the serial output is only sent every few seconds, the slow moving EWMA
 * is used in order to capture intermittent values (e.g., current spikes). 
 */
double last_amp_seconds = 0.0;
double last_watt_seconds = 0.0;
long last_serial_print = 0;


void serial_print(long now) {
  strcpy_P(label_buffer, (char*)pgm_read_word(&(states[state])));
  Serial.printf(F("%-16s"), label_buffer);
  
  Serial.print(F(" Battery="));
  display_float(batteryVoltage->smoothedValueFast(), -7, 3, &Serial);
  
  Serial.print(F(" Supply="));
  display_float(supplyVoltage->smoothedValueFast(), -7, 3, &Serial);
  
  Serial.print(F(" RPiOn="));
  Serial.print(rpi_powered_on);

  Serial.printf(F(" StateTime=%lu UpTime=%lu"), (now - last_state_change), now);

  if (state == Initializing) {
    // Avoid printing out weird current and temperature when first starting
    Serial.println();
    return;
  }  

  if (is_battery_connected(state) || is_supply_connected(state)) {
    Serial.print(F(" Temperature="));
    display_float(temperature, -6, 3, &Serial);
  
    Serial.print(F(" Current="));
    display_float(currentSense->smoothedValueFast(), -6, 2, &Serial);
    
    Serial.print(F(" AH="));
    double ah = current_amp_seconds / 3600.;
    display_float(ah, -8, 2, &Serial);

    // Return the amount of Amp-seconds since the last report. Summing
    // these values over a period of time and converting to Amp-hours
    // will give an accurate view of the charge consumption.
    Serial.print(F(" AmpSecDelta="));
    double amp_sec = (current_amp_seconds - last_amp_seconds);
    display_float(amp_sec, -8, 4, &Serial);
    last_amp_seconds = current_amp_seconds;

    // Take the accumulated "current_amp_sec" since the last report
    // and devide by the time since the last report. This gives us a
    // much better view of amps than just the current value or even the EWMA.
    Serial.print(F(" AmpAvg="));  
    double dt = (now - last_serial_print) / 1000.0;
    double amps = amp_sec / dt;
    display_float(amps, -8, 4, &Serial);

    // Similarly, divide the accumulated Watt-seconds by the duration of this
    // report. This gives a propertly averaged power.
    Serial.print(F(" PowerAvg="));
    double watt_sec = (current_watt_seconds - last_watt_seconds);
    double watts = watt_sec / dt;
    display_float(watts, -8, 4, &Serial);
  }
  Serial.println();
  last_serial_print = now;
}

/******************
 * Setup and loop *
 ******************/

void setup() {
  Serial.begin(9600);
  
  EEPROM.begin();
  // Initialize EEPROM
#ifdef EEPROM_RESET
  reset_config();
#endif
  byte eeprom_ver = EEPROM.read(0x00);
  if (eeprom_ver == 255) {
      reset_config();
  }
  load_config();

  analogReference(EXTERNAL);
  pinMode(A0, INPUT); // Battery voltage
  pinMode(A1, INPUT); // Load Current
  pinMode(A2, INPUT); // Temperature voltage
  pinMode(A3, INPUT); // Supply voltage

  pinMode(STATUS_LED, OUTPUT);
  pinMode(BATTERY_PIN, OUTPUT);  
  pinMode(SUPPLY_PIN, OUTPUT);  
  pinMode(RPI_PWR_PIN, OUTPUT);
  pinMode(RPI_SHUTDOWN_PIN, INPUT); // Leave this floating when we're not pulling it low
  pinMode(RPI_POWEROFF_PIN, INPUT);

  digitalWrite(SUPPLY_PIN, HIGH);
  digitalWrite(BATTERY_PIN, LOW); // If the uC has power, there is a battery! Start off with battery connected
  digitalWrite(RPI_PWR_PIN, LOW);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  start_time = millis();
  //Serial.println("0\tV\t0.0");
  Serial.println();
  Serial.flush();
  Serial.println("Start");

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextWrap(true);
  display.print(EEPROM.length());
  for (uint16_t i=0; i<EEPROM.length(); i++) {
    display.print(EEPROM.read(i));
  }
  display.display();
  delay(1000);

  // https://en.wikipedia.org/wiki/Code_page_437
  display.cp437(true);
  display_info(start_time, NONE);
  delay(1000);
  
#ifdef TARPN_LOGO
  display.clearDisplay();
  display.drawBitmap(0, 8, tarpn_logo_bitmap, 128, 32, WHITE);
  display.display();
  delay(1000);
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
  switch_closed = false;
  last_switch_open_time = start_time;
  last_switch_closed_time = 0L;
  last_adc_read = 0L;
  last_serial_write = 0L;
  next_display_change = 0L;

  user_sw = Bounce();
  user_sw.attach(5, INPUT);
  user_sw.interval(5);
}


void loop() {
  long now = millis();

  // Always read the supply voltage so we can quickly detect loss of power
  read_voltage(A3, supplyVoltage);

  // Evaluate the state machine and change the output state
  boolean changed_state = next_state(now, temperature);
  apply_state(now, state);
  if (changed_state) {
    serial_print(now);
    last_serial_write = now;
  }

  if (state == ShutdownComplete) {

    // Low power mode will mess with millis(). The ShutdownComplete state ignores time
    // and only considers the instantaneous value of the ADC for the battery voltage.
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

    // Flash the status LED so users know we're alive
    digitalWrite(STATUS_LED, HIGH); 
    delay(50);
    digitalWrite(STATUS_LED, LOW);
    
    // After one sleep, clear the display
    display.clearDisplay();
    display.display();
    display.dim(true);

    // Read the battery voltage to give us a chance to wakeup
    read_voltage(A0, batteryVoltage);
    return;
  }

  // Only read the ADCs every so often
  if ((now - last_adc_read) < 20) {
    return;
  } else {
    read_voltage(A0, batteryVoltage);
    read_voltage(A1, currentSense);
    read_voltage(A2, tempSense);
    temperature = read_temp();
    double current_sec = (now - last_adc_read) / 1000.;
    current_amp_seconds += (currentSense->smoothedValueFast() * current_sec);
    if (is_battery_connected(state)) {
      current_watt_seconds += (currentSense->smoothedValueFast() * batteryVoltage->smoothedValueFast() * current_sec);
    } else if (is_supply_connected(state)) {
      current_watt_seconds += (currentSense->smoothedValueFast() * supplyVoltage->smoothedValueFast() * current_sec);
    }

    last_adc_read = now;
    read_rpi_power_state();
  }

  // Output to Serial occasionally
  if ((now - last_serial_write) > 2000) {
    // TODO figure out how to do non-blocking serial. If our voltage drops during this print, there will be
    // a delay in the switching
    serial_print(now);
    last_serial_write = now;
  } 

  // Check for user input
  user_sw.update();
  if (user_sw.read() == LOW && user_sw.currentDuration() >= 1000 && !long_press_captured) {
    // Captured a long button press
    long_press_captured = true;
  }

  UserEvent event = NONE;
  if (user_sw.rose()) {
    if (long_press_captured) {
      event = LONG_PRESS;
      long_press_captured = false;
    } else {
      event = BUTTON_PRESS;
    }
    long_press_captured = false;
  }

  // Read encoder
  int new_encoder_pos = myEnc.read();
  if (new_encoder_pos > last_encoder_pos + 2) {
    event = SCROLL_RIGHT;
    last_encoder_pos = new_encoder_pos;
  } else if (new_encoder_pos < last_encoder_pos - 2) {
    event = SCROLL_LEFT;
    last_encoder_pos = new_encoder_pos;
  }

  // Choose a display mode
  if (error_code != NoMessage) {
    // If there's a new error code, display it
    display_error_code = error_code;
    error_code = NoMessage;
    last_error_display = now;
  } else { 
    if (display_error_code != NoMessage && event == NONE && (now - last_error_display) < 2000) {
      // Display error messsage for 2 seconds unless we get user input      
      display_message(now, event);
      return;
    } else {
      display_error_code = NoMessage;
    }

    if (in_shutdown_sequence(state)) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.print("/o_o\\");
      display_icons(now);
      display.display();
    } else if (config_mode_enabled) {
      display_config(now, event);
    } else {
      display_main(now, event);
    }
  }
}
