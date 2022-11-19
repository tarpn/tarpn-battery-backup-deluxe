#ifndef UI_h
#define UI_h

enum Messages {
  NoMessage,
  SupplyRecovered,
  LowSupplyVoltageMessage,
  LowSupplyVoltageMessage2,
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


const char error_0[] PROGMEM = "";
const char error_1[] PROGMEM = "Supply Recovered";
const char error_2[] PROGMEM = "Low Supply Detected";
const char error_3[] PROGMEM = "Supply Interrupted";
const char error_4[] PROGMEM = "Low Battery";
const char error_5[] PROGMEM = "High Battery";
const char error_6[] PROGMEM = "Over Temp";
const char error_7[] PROGMEM = "EEPROM Saved";
const char error_8[] PROGMEM = "EEPROM Reset";
const char error_9[] PROGMEM = "RPi is Shutdown";
const char error_10[] PROGMEM = "Shutting Down RPi";

const char * const error_messages[] PROGMEM = {
  error_0, error_1, error_2, error_3, error_4, error_5,
  error_6, error_7, error_8, error_9, error_10
};

const char menu_0_0[] PROGMEM = "Set Limits";
const char menu_0_1[] PROGMEM = "Tune";
const char menu_0_2[] PROGMEM = "Save EEPROM";
const char menu_0_3[] PROGMEM = "Reset EEPROM";
const char menu_0_4[] PROGMEM = "Shutdown RPi";
const char menu_0_5[] PROGMEM = "Info";
const char menu_0_6[] PROGMEM = "Return";

const char * const menu_0[] PROGMEM = {
  menu_0_0, menu_0_1, menu_0_2, menu_0_3,
  menu_0_4, menu_0_5, menu_0_6
};

const uint8_t menu_0_len = 7;

typedef struct {
  // Encoder
  int last_encoder_pos;

  // Display
  unsigned long last_display_update_ms;
  uint8_t display_choice;
  Messages error_code;
  bool display_error;
  unsigned long last_error_display;
  char message_line_1[21];
  char message_line_2[21];

  // Menus
  bool config_mode_enabled;
  uint8_t config_depth = 0;

  uint8_t config_selection_0 = 0;
  uint8_t config_offset_0 = 0;

  uint8_t config_selection_1 = 0;
  uint8_t config_offset_1 = 0;

  uint8_t config_selection_2 = 0;
  uint8_t config_offset_2 = 0;

  char buffer_1[17];
  char buffer_2[17];

  // Events
  UserEvent event;
  unsigned long last_event;

  // LED
  bool status_led;

  // Debug
  bool headless_debug_mode_enabled;
} UIState;

void UI_Init(UIState * state, unsigned long millis);

void UI_encoder_button_press(UIState * state, unsigned long millis);

void UI_encoder_position(UIState * state, unsigned long millis, int pos);

UserEvent UI_next_event(UIState * state, unsigned long millis);

#endif