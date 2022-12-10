#include <EEPROM.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce2.h>
// #define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include "config.h"
#include "voltage_read.h"

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define STATUS_LED 7
#define BUTTON_PIN 11
#define ENCODER_PIN_A 3
#define ENCODER_PIN_B 2
#define RPI_PIN 0
#define MOSTFET_SW 9

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

//#define EEPROM_RESET 1

#define ADC_ZENER_VOLTAGE     4.65    // Measured from D1
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
  ADC_ZENER_VOLTAGE, 
  VOLTAGE_FACTOR, 
  CURRENT_FACTOR, 
  CURRENT_OFFSET,
  
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
  BatteryLow,       // 4
  BatteryLowTrip,   // 5
  BatteryHigh,      // 6
  BatteryHighTrip,  // 7
  OverTemp,         // 8
  OverTempRecovery  // 9
};

enum Messages {
  NoMessage,
  ReverseCurrentMessage,
  LowSupplyVoltageMessage,
  LowBatteryVoltageMessage,
  HighBatteryVoltageMessage,
  OverTempMessage,
  EEPROMSavedMessage,
  EEPROMResetMessage
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
const char label_2[] PROGMEM = "Battery Current";
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
  configs->batt_high_voltage_cutoff = packed->batt_high_voltage_cutoff / 1000.0;
  configs->batt_high_voltage_recovery = packed->batt_high_voltage_recovery / 1000.0;
  configs->batt_high_voltage_trip_ms = packed->batt_high_voltage_trip_ms;
  configs->batt_low_voltage_cutoff = packed->batt_low_voltage_cutoff / 1000.0;
  configs->batt_low_voltage_recovery = packed->batt_low_voltage_recovery / 1000.0;
  configs->batt_low_voltage_trip_ms = packed->batt_low_voltage_trip_ms;
  configs->batt_recovery_ms = packed->batt_recovery_ms;
  configs->overtemp_cutoff = packed->overtemp_cutoff;
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

Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);
int last_encoder_pos = 0;
Bounce user_sw;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
long next_display_change;
bool config_mode_enabled = false;
bool long_press_captured = false;
uint8_t display_choice = 0;
Messages error_code = NoMessage;
long last_error_display = 0;

long last_blink_off = -1; // if not -1, flash off
long last_blink_on = -1;  // if not -1, flash on

boolean do_blink(long now) {
  if (last_blink_off == -1 && last_blink_on == -1) {
    last_blink_on = now;
    return true;
  }

  if (last_blink_on == -1) {
    // flash is off
    if ((now - last_blink_off) > 100) {
      last_blink_on = now;
      last_blink_off = -1;
      return true;
    }
    return false;
  }

  if (last_blink_off == -1) {
    if ((now - last_blink_on) > 400) {
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

double read_temp() {
  // Temperature is a bit difference since it has a complicated formula and not just a scaling factor.
  // We still use the EMWA filter in VoltageRead, but calclulate the real value here.
  double res = (49900.0 * tempSense->adcVoltageSlow()) / (batteryVoltage->smoothedValueSlow() - tempSense->adcVoltageSlow());
  double steinhart = logf(res / 10000.0);
  steinhart /= 3933;
  steinhart += (1.0 / 298.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  return steinhart;
}

void open_switch(long now) {
  if (switch_closed)
    last_switch_open_time = now;
  switch_closed = false;
  digitalWrite(MOSTFET_SW, LOW);
  digitalWrite(STATUS_LED, LOW);
}

void close_switch(long now) {
  if (!switch_closed)
    last_switch_closed_time = now;
  switch_closed = true;
  digitalWrite(MOSTFET_SW, HIGH);
  digitalWrite(STATUS_LED, HIGH);
}

bool is_mosfet_on() {
  return switch_closed;
}

State next_state(long now, double temp, State state) {
  if (state != Initializing &&
      state != OverTemp &&
      temp >= OVER_TEMP_CUTOFF) {
    // Over temp logic applies to most states, so do it separately.
    last_state_change = now;
    error_code = OverTempMessage;
    return OverTemp;
  }

  int current_state_ms = now - last_state_change;
  State next_state = Null;

  switch (state) {
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
    if (supplyVoltage->instantValue() < STANDBY_LOW_VOLTAGE) {
      error_code = LowSupplyVoltageMessage;
      next_state = Backup;
    }
    break;
  case Backup:
    if (batteryVoltage->smoothedValueSlow() < BATTERY_LOW_VOLTAGE_CUTOFF) {
      error_code = LowBatteryVoltageMessage;
      next_state = BatteryLow;
    } else if (batteryVoltage->smoothedValueSlow() >= BATTERY_HIGH_VOLTAGE_CUTOFF) {
      error_code = HighBatteryVoltageMessage;
      next_state = BatteryHigh;
    } else if (currentSense->smoothedValueFast() < -0.5) {
      // More then 0.5 amps of current flowing from supply to battery
      error_code = ReverseCurrentMessage;
      next_state = Standby;
    }
    break;
  case BatteryLow:
    if (current_state_ms >= BATTERY_LOW_VOLTAGE_TRIP_MS) {
      next_state = BatteryLowTrip;
    } else if (batteryVoltage->smoothedValueSlow() >= BATTERY_LOW_VOLTAGE_RECOVERY) {
      next_state = Backup;
    }
    break;
  case BatteryLowTrip:
    if (batteryVoltage->smoothedValueSlow() >= BATTERY_LOW_VOLTAGE_RECOVERY) {
      next_state = Backup;
    }
    break;
  case BatteryHigh:
    if (current_state_ms >= BATTERY_HIGH_VOLTAGE_TRIP_MS) {
      next_state = BatteryHighTrip;
    } else if (batteryVoltage->smoothedValueSlow() < BATTERY_HIGH_VOLTAGE_RECOVERY) {
      next_state = Backup;
    }
    break;
  case BatteryHighTrip:
    if (batteryVoltage->smoothedValueSlow() < BATTERY_HIGH_VOLTAGE_RECOVERY) {
      next_state = Backup;
    }
    break;
  case OverTemp:
    if (temp < OVER_TEMP_RECOVERY) {
      next_state = OverTempRecovery;
    }
    break;
  case OverTempRecovery:
    // Don't need to check for over temp here since we check it at the top of this routine.
    // Just check that we're still below the recovery temperature for long enough.
    if (current_state_ms >= OVER_TEMP_RECOVERY_MS && temp < OVER_TEMP_RECOVERY) {
      next_state = Standby;
    }
    break;
  }


  if (next_state == Null) {
    return state;
  } else {
    last_state_change = now;
    return next_state;
  }
}

void apply_state(long now, State state) {
  switch (state) {
    case Backup:
    case BatteryLow:
    case BatteryHigh:
      close_switch(now);
      break;
    case Initializing:
    case Standby:
    case BatteryLowTrip:
    case BatteryHighTrip:
    case OverTemp:
    case OverTempRecovery:
    default:
      open_switch(now);
      break;
  }
}

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
#define NUM_TUNABLES 2

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
  {&(configs.current_offset), currentSense}
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
  {"X2", "Current Offset", "", 0, -2, 2}
};

char float_buffer[10];
void display_float(float value, uint8_t width, uint8_t precision) {
  dtostrf(value, width, precision, float_buffer);
  display.print(float_buffer);
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
  display_float(val, 5, staticConfigSRAM.decimal_places);
  uint8_t blink_pos;
  if (!do_blink(now)) {
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
    display_float(derived, 4, 2);
  }
  display.display();
}

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
  display_float(*(dynamicConfig.value), 5, staticConfigSRAM.decimal_places);
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
    display_float(derived, 4, 2);
  }
  display.display();
}

/**
 * Config mode, single click to enter, select "Return" or long click to exit.
 * 
 * Choices: 
 *  - Set Limits
 *  - Tune
 *  - Info
 *  - Save EEPROM
 *  - Return
 * 
 * Set Limits will show the voltage, temperature, and current limits for the board.
 * Tune will allow tweaking the voltage divider scaling factors.
 * 
 * In "Set Limits", each value is displayed with a name like V1, T1, etc. 
 * 
 * V1 14.50 ⏎
 * High voltage cutoff (small font)
 * 
 * Click to enter Set mode. Starts with the largest settable unit (e.g., 1 or 0.1) 
 * and that digit begins flashing on/off. A turn on the encoder changes the value, and
 * a click selects the next digit. A special return glyph is also selectable which will
 * exit Set mode. A long press also exits Set mode (and Config mode).
 * 
 * The "Tune" option is mostly the same as "Set Limits", but we also display a related
 * real-time value being read by the board. This is used for dialing in things like the
 * ADC voltage and the current sensor.
 * 
*/
void display_config(long now, UserEvent event) {
  if (event == LONG_PRESS) {
    config_mode_enabled = false;
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);

  uint8_t choices = 6; 
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

void display_message(long now, UserEvent event) {
  display.clearDisplay();  
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  switch (error_code) {
    case ReverseCurrentMessage:
      display.println(F("Battery\nDetected"));
      break;
    case LowSupplyVoltageMessage:
      display.println(F("Low Supply\nVoltage"));
      break;
    case LowBatteryVoltageMessage:
      display.println(F("Battery\nUndervolt"));
      break;
    case HighBatteryVoltageMessage:
      display.println(F("Battery\nOvervolt"));
      break;
    case OverTempMessage:
      display.println(F("Over Temp"));
      break;
    case EEPROMSavedMessage:
      display.println(F("EEPROM\nsaved!"));
      break;
    case EEPROMResetMessage:
      display.println(F("EEPROM\nreset!"));
      break;
    default:
      display.print(F("Message "));
      display.println(error_code);
      break;
  }
  display.display();
  error_code = NoMessage;
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

  // Big output
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

  // Small output
  display.setTextSize(1);
  display.setCursor(96, 0);
  display.print(F("|"));
  display.print(state);
  display.setCursor(96, 8);
  display.print(F("|"));
  display.print(display_voltage->sensorValue); 
  display.setCursor(96, 16);
  display.print(F("|"));
  display_float(display_voltage->adcVoltageFast(), 4, 2);
  display.setCursor(96, 24);
  display.print(F("|"));
  display_float(display_voltage->adcVoltageSlow(), 4, 2);
  display.display();
}

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

  pinMode(MOSTFET_SW, OUTPUT);  
  digitalWrite(MOSTFET_SW, LOW);

  pinMode(RPI_PIN, OUTPUT);
  pinMode(A0, INPUT); // Battery voltage
  pinMode(A1, INPUT); // Supply voltage
  pinMode(STATUS_LED, OUTPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  start_time = millis();

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
  
  /*
  // Flash the status led
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(STATUS_LED, LOW);
  delay(500);
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(STATUS_LED, LOW);
  delay(500);
  */

  /*
  // Print out EEPROM
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("ver: %d\n", eeprom_ver);
  display.print("ref: ");
  display.println(configs.reference_voltage, 3);
  display.print("vol: ");
  display.println(configs.voltage_factor, 3);
  display.print("cur: ");
  display.println(configs.current_offset, 3);
  display.display();
  delay(1000);
  */

  // Initialize
  state = Initializing;
  last_state_change = start_time;
  switch_closed = false;
  last_switch_open_time = start_time;
  last_switch_closed_time = 0L;
  last_adc_read = 0L;
  next_display_change = 0L;

  user_sw = Bounce();
  user_sw.attach(5, INPUT);
  user_sw.interval(5);

  digitalWrite(RPI_PIN, HIGH);
}

void loop() {
  long now = millis();

  // TODO Optimize SupplyOff handling? (would need a hardware interrupt)
  read_voltage(A3, supplyVoltage);

  // Determine desired switch state
  state = next_state(now, temperature, state);
  apply_state(now, state);

  // Read ADCs ever so often
  if ((now - last_adc_read) < 20) {
    return;
  } else {
    last_adc_read = now;
    read_voltage(A0, batteryVoltage);
    read_voltage(A1, currentSense);
    read_voltage(A2, tempSense);
    temperature = read_temp();
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
    last_error_display = now;
    display_message(now, event);
  } else { 
    if (event == NONE && (now - last_error_display) < 2000) {
      // Display error messsage for 2 seconds unless we get user input
      return;
    }

    if (config_mode_enabled) {
      display_config(now, event);
    } else {
      display_main(now, event);
    }
  }  
}
