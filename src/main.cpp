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
#define BATTERY_RECOVERY_TRIP_MS      10000
#define OVER_TEMP_CUTOFF              60      // At this temperature in celcius, disconnect battery
#define OVER_TEMP_RECOVERY            50      // 
#define OVER_TEMP_RECOVERY_MS         10000   // 

//#define EEPROM_RESET 1

#define ADC_ZENER_VOLTAGE     4.65    // Measured from D1
#define VOLTAGE_FACTOR        0.167   // For 10k 49.9k resistor divider use 0.167. For 10k 100k, use 0.091
#define CURRENT_FACTOR        0.1204  // Shunt resistor value times gain factor (0.003 * 40 = 0.120)
#define CURRENT_OFFSET        2.400   // Op amp uses a 2.4V reference to allow for negative voltages

// TODO add limits to StatefulConfigs

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
  //float overtemp_cutoff;
  //float overtemp_recovery;
  //float overtime_recovery_ms;

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
  BATTERY_RECOVERY_TRIP_MS
  //OVER_TEMP_CUTOFF,
  //OVER_TEMP_RECOVERY,
  //OVER_TEMP_RECOVERY_MS
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

enum UserEvent {
  NONE,
  BUTTON_PRESS,
  LONG_PRESS,
  SCROLL_RIGHT,
  SCROLL_LEFT
};

char * labels[] = {
  "Battery Voltage",
  "Supply Voltage",
  "Battery Current",
  "Temperature" 
};

struct VoltageRead * batteryVoltage = new VoltageRead(ADC_ZENER_VOLTAGE, VOLTAGE_FACTOR);
struct VoltageRead * supplyVoltage = new VoltageRead(ADC_ZENER_VOLTAGE, VOLTAGE_FACTOR);
struct VoltageRead * tempSense = new VoltageRead(ADC_ZENER_VOLTAGE, 1.0);
struct VoltageRead * currentSense = new VoltageRead(ADC_ZENER_VOLTAGE, CURRENT_FACTOR, CURRENT_OFFSET);

void save_config() {
  EEPROM.put(1, configs);
  batteryVoltage->factor = configs.voltage_factor;
  supplyVoltage->factor = configs.voltage_factor;
  currentSense->factor = configs.current_factor;
  currentSense->offset = configs.current_offset;
}

void reset_config() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }

  EEPROM.put(1, config_defaults);
}

void load_config() {
  EEPROM.get(1, configs);
  batteryVoltage->factor = configs.voltage_factor;
  supplyVoltage->factor = configs.voltage_factor;
  currentSense->factor = configs.current_factor;
  currentSense->offset = configs.current_offset;
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
int error_code = -1;
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
      next_state = Backup;
    }
    break;
  case Backup:
    if (batteryVoltage->smoothedValueSlow() < BATTERY_LOW_VOLTAGE_CUTOFF) {
      next_state = BatteryLow;
    } else if (batteryVoltage->smoothedValueSlow() >= BATTERY_HIGH_VOLTAGE_CUTOFF) {
      next_state = BatteryHigh;
    } else if (currentSense->smoothedValueFast() < -0.5) {
      // More then 0.5 amps of current flowing from supply to battery
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
  display.print("TARPN BBD  ");
  display.println(uptime_sec);
  display.println("David Arthur, K4DBZ");
  display.println(BUILD_DATE);
  display.print(GIT_REV);
  display.display();
}

struct ConfigDisplay {
  char * name;
  char * label_1;
  char * label_2;
  float * value;
  int8_t hi;  // highest power of 10 to change
  int8_t lo;  // lowest power of 10 to change
  uint8_t decimal_places;  // digits after decimal
  VoltageRead * derived;
};

#define NUM_CONFIGS 6
#define NUM_TUNABLES 2

float foo = 12.5;

struct ConfigDisplay configurables[NUM_CONFIGS] = {
  {"H1", "Battery High", "Voltage Cutoff", &(configs.batt_high_voltage_cutoff), 0, -1, 1, NULL},
  {"H2", "Battery High", "Voltage Recovery", &(configs.batt_high_voltage_recovery), 0, -1, 1, NULL},
  {"H3", "Battery High", "Trip Time ms", &(configs.batt_high_voltage_trip_ms), 3, 0, 0, NULL},
  {"L1", "Battery Low", "Voltage Cutoff", &(configs.batt_low_voltage_cutoff), 0, -2, 2, NULL},
  {"L2", "Battery Low", "Voltage Recovery", &(configs.batt_low_voltage_recovery), 0, -2, 2, NULL},
  {"L3", "Battery Low", "Trip Time ms", &(configs.batt_low_voltage_trip_ms), 3, 0, 0, NULL}
  //{"R1", "Battery", "Recovery ms", &(configs.batt_recovery_ms), 3, 0, 0, NULL},
  //{"T1", "Temperature", "Cutoff Temp", &(configs.overtemp_cutoff), 2, 1, 1, NULL},
  //{"T2", "Temperature", "Recovery Temp", &(configs.overtemp_recovery), 2, 1, 1, NULL},
  //{"T3", "Temperature", "Recovery Time ms", &(configs.overtime_recovery_ms), 3, 0, 0, NULL}
};

struct ConfigDisplay tunables[2] = {
  {"X1", "Current Sense", "", &(configs.current_factor), -1, -3, 3, currentSense},
  {"X2", "Current Offset", "", &(configs.current_offset), 0, -2, 2, currentSense},
};

void adjust_set_levels(long now, UserEvent event, ConfigDisplay * choices, uint8_t choices_len) {  
  // For each config, need the display string, valid units to cycle through, and location
  // of digit to flash.

  ConfigDisplay configDisplay = choices[config_selection_1];
  uint8_t digit_idx = config_selection_2 % (1 + configDisplay.hi - configDisplay.lo);
  // exponent is 2 => 100, 1 => 10, 0 => 1, -1 => 0.1, -2 => 0.01
  int8_t exponent = (configDisplay.hi - digit_idx);
  double adj = pow(10.0, exponent);
  double val = *(configDisplay.value);

  if (event == SCROLL_LEFT) {
    val -= adj;
  }
  if (event == SCROLL_RIGHT) {
    val += adj;
  }
  *(configDisplay.value) = val;

  if (event == BUTTON_PRESS) {
    if (config_selection_2 == 2) {
      config_selection_2 = 0;
      config_depth = 1;
    } else {
      config_selection_2++;
      return;
    }
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(configDisplay.name);
  display.print(" "); 
  display.print(val, configDisplay.decimal_places);
  if (!do_blink(now)) {
    uint8_t base = 36;
    if (val < 0) {
      base += 12;
    }
    // TODO deal with negative sign
    if (val >= 10.0) {
      // two digits before decimal
      if (exponent == 1) {
        display.fillRect(base, 0, 12, 16, SSD1306_BLACK);
      } else if (exponent == 0) {
        display.fillRect(base + 12, 0, 12, 16, SSD1306_BLACK);
      } else if (exponent == -1) {
        display.fillRect(base + 36, 0, 12, 16, SSD1306_BLACK);
      } else if (exponent == -2) {
        display.fillRect(base + 48, 0, 12, 16, SSD1306_BLACK);
      }
    } else {
      // one digit (or zero) before decimal
      if (exponent == 0) {
        display.fillRect(base, 0, 12, 16, SSD1306_BLACK);
      } else if (exponent == -1) {
        display.fillRect(base + 24, 0, 12, 16, SSD1306_BLACK);
      } else if (exponent == -2) {
        display.fillRect(base + 36, 0, 12, 16, SSD1306_BLACK);
      } else if (exponent == -3) {
        display.fillRect(base + 48, 0, 12, 16, SSD1306_BLACK);
      }
    }
  }
  display.setCursor(108, 0);
  display.print((char) 0x1B); // <- arrow
  display.setCursor(0, 16);
  display.setTextSize(1);
  display.println(configDisplay.label_1);
  display.println(configDisplay.label_2);
  if (configDisplay.derived != NULL) {
    double derived = configDisplay.derived->smoothedValueFast();
    if (derived < 0.0) {
      display.setCursor(74, 24);
    } else {
      display.setCursor(80, 24);
    }
    display.print(derived);
  }
  display.display();
}

void display_set_levels(long now, UserEvent event, ConfigDisplay * choices, uint8_t choices_len) {
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
    display.print("Return");
    display.drawBitmap(80, 0, return_glyph, 12, 18, WHITE);
    display.display();
    return;
  }

  ConfigDisplay configDisplay = choices[config_selection_1];

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(configDisplay.name);
  display.print(" "); 
  display.print(*(configDisplay.value), configDisplay.decimal_places);
  display.setCursor(108, 0);
  display.print((char) 0x12); // up/down arrow
  display.setCursor(0, 16);
  display.setTextSize(1);
  display.println(configDisplay.label_1);
  display.println(configDisplay.label_2);
  if (configDisplay.derived != NULL) {
    double derived = configDisplay.derived->smoothedValueFast();
    if (derived < 0.0) {
      display.setCursor(74, 24);
    } else {
      display.setCursor(80, 24);
    }
    display.println(derived);
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

  uint8_t choices = 5;  // TODO add Save EEPROM
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
          reset_config();
          break;
        case 4:
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
        display.println("Set Limits");
        break;
      case 1:
        display.println("Tune");
        break;
      case 2:
        display.println("Info");
        break;
      case 3:
        display.setCursor(4, 0);
        display.print("Reset");
        display.setCursor(4, 16);
        display.print("EEPROM");
        break;
      case 4:
        display.print("Return");
        display.drawBitmap(84, 8, return_glyph, 12, 18, WHITE);
        break;
      
    } 
    display.display();
    return;
  }

  if (config_depth == 1) {
    switch (config_selection_0) {
      case 0:
        display_set_levels(now, event, configurables, NUM_CONFIGS);
        break;
      case 1:
        display_set_levels(now, event, tunables, NUM_TUNABLES);
        break;
      case 2:
        display_info(now, event);
        break;
    } 
    return;
  }

  if (config_depth == 2) {
    if (config_selection_0 == 0) {
        adjust_set_levels(now, event, configurables, NUM_CONFIGS);
    } else if (config_selection_0 == 1) {
        adjust_set_levels(now, event, tunables, NUM_TUNABLES);
    }
  }

}

void display_message(long now, UserEvent event) {
  // TODO, don't clear display here, just overlay the message in a box
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  switch (error_code) {
    case 0:
      display.println("Reverse current");
      break;
    case 1:
      display.println("Low PSU voltage");
      break;
    case 2:
      display.println("Low battery voltage");
      break;
    case 3:
      display.println("Over temp");
      break;
  }
  display.display();
  error_code = -1;
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
  display.print(labels[label_index]);

  // Small output
  display.setTextSize(1);
  display.setCursor(96, 0);
  display.print("|");
  display.print(state);
  display.setCursor(96, 8);
  display.print("|");
  display.print(display_voltage->sensorValue); 
  display.setCursor(96, 16);
  display.print("|");
  display.print(display_voltage->adcVoltageFast(), 2);
  display.setCursor(96, 24);
  display.print("|");
  display.print(display_voltage->adcVoltageSlow(), 2);
  display.display();
}

void setup() {
  Serial.begin(9600);
  EEPROM.begin();

  // Initialize EEPROM
#ifdef EEPROM_RESET
  EEPROM.write(0x00, 255);
#endif
  byte eeprom_ver = EEPROM.read(0x00);
  if (eeprom_ver == 255) {
    EEPROM.write(0x00, 1);
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

  // https://en.wikipedia.org/wiki/Code_page_437
  display.cp437(true);
  display_info(start_time, NONE);
  delay(2000);
  


#ifdef TARPN_LOGO
  display.clearDisplay();
  display.drawBitmap(0, 8, tarpn_logo_bitmap, 128, 32, WHITE);
  display.display();
  delay(2000);
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
  
  // Flash the status led
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(STATUS_LED, LOW);
  delay(500);
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(STATUS_LED, LOW);
  delay(500);

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
  if (error_code != -1) {
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
