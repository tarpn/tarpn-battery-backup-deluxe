Serial data specification for BBD firmware


v1 Sep 14, 2023
===============

On firmware startup, the string "START" is sent followed by a newline.

Subsequently, every 2 seconds or when a event occurs, the firmware will send data in the 
following BNF form:

```
<line>		::= <state> <ws> "Battery=" <float> <ws> "Supply=" <float> <ws> "RPiOn=" <bool> <ws> "StateTime=" <long> <ws> "UpTime=" <long> <ws> "DT=" <long> <ws> "Git=" <str> [<cont>] <nl>
<ws>		::= " " [<ws>]	; any amount of whitespace characters
<nl>		::= "\n"	; newline character
<float>		::= <digits> "." <digits>
<long>		::= <digits>
<bool>		::= "1" | "0"
<str>		::= <letter> [<str>]
<digits>	::= <digit> [<digits>]
<state>		::= "Null" | "Initializing" | "Standby" | "Backup" | "Recovery" | "BatteryLow" | "BatteryLowTrip" | "BatteryHigh" | "BatteryHighTrip" | "OverTemp" | "OverTempRecovery" | "BeginShutdown" | "RPiShutdown" | "ShutdownComplete"
<cont>		::= <ws> "Temperature=" <float> <ws> "AH=" <float> <ws> "AmpSecDelta=" <float> <ws> "BatteryAmpSec=" <float> <ws> "AmpAvg=" <float> <ws> "AmpMax=" <float> <ws> "WattSecDelta=" <float> "<ws>WattAvg=" <float>
```

v2 Dec 26, 2023
===============

Serial data is ascii text separated by newlines. The first word of a line indicates its type.

Previously, each line was interpreted as metrics data with the exception of a special "START" line. Now, each line 
will begin with its message type allowing for more flexibility in the protocol.

Three types will be defined initially

* `DATA`: a line of key/value data pairs. See format below
* `EVENT <type> [<args>]`: a discrete event that occurred, such as a switch changing states
* `LOG`: informational text sent by the firmware

Also, this version of the protocol introduces bidirectional communication. 

* `P <0|1> <len> <text>`: informational text sent by the PC for display by the OLED. One of two lines is specified followed 
                           by the desired text. Any characters exceeding the width of the display will be truncated
* `H`: a heartbeat, sent periodically to inform the firmware that the PC is alive




Change state strings to be all caps with underscores. This is to align the state with the new UI
for the OLED display.

```
<line>		::= <state> <ws> "Battery=" <float> <ws> "Supply=" <float> <ws> "RPiOn=" <bool> <ws> "StateTime=" <long> <ws> "UpTime=" <long> <ws> "DT=" <long> <ws> "Git=" <str> [<cont>] <nl>
<ws>		::= " " [<ws>]	; any amount of whitespace characters
<nl>		::= "\n"	; newline character
<float>		::= <digits> "." <digits>
<long>		::= <digits>
<bool>		::= "1" | "0"
<str>		::= <letter> [<str>]
<digits>	::= <digit> [<digits>]
<state>		::= "NULL" | "INITIALIZING" | "STANDBY" | "BACKUP" | "RECOVERY" | "BATT_LOW" | "BATT_LOW_TRIP" | "BATT_HIGH" | "BATT_HIGH_TRIP" | "OVER_TEMP" | "OVER_TEMP_RECOVER" | "BEGIN_SHUTDOWN" | "RPI_SHUTDOWN" | "SHUTDOWN_COMPLETE"
<cont>		::= <ws> "Temperature=" <float> <ws> "AH=" <float> <ws> "AmpSecDelta=" <float> <ws> "BatteryAmpSec=" <float> <ws> "AmpAvg=" <float> <ws> "AmpMax=" <float> <ws> "WattSecDelta=" <float> "<ws>WattAvg=" <float>
```

##  Metrics Flow

### Current

Shows "instantaneous" current. 

Prometheus metrics: load_amps, peak_amps

load_amps: Gauge, set by AmpAvg from serial
peak_amps: Gauge, set by AmpMax from serial

AmpAvg is the number of accumulated amp-seconds divided by the reporting period. Amp-seconds are 
continuously integrated whenever the ADC is sampled.

### Power

Shows "instantaneous" power (watts).

Prometheus metric: load_watt, load_watt_sec

load_watt: Gauge, set by WattAvg from serial
load_watt_sec: Counter, incremented by WattSecDelta from serial

WattAvg is accumulated watt-seconds divided by the reporting period. The watt-seconds are
continuous integrated as (current * voltage) whenever the ADC is sampled. This gives more accurate
reading since the voltage can vary based on the current.

WattSecDelta is the number of watt-seconds accumulated during the reporting period.

```
metrics->report->load_amp_sec_delta = (metrics->current_amp_seconds - metrics->report->load_amp_sec);
metrics->report->load_amp_sec = metrics->current_amp_seconds;
metrics->report->load_amp_avg = metrics->report->load_amp_sec_delta / report_period_s;
```

TODO

* Instantaneous Battery Voltage, Current. 
* Instantaneous Load Power (computed based on state)
* Delta of power. Watt*sec since last report. `increase(load_watt_sec_total[1m]) / 60`
* Cumulative power? Watt*sec since startup. Is this useful? Why not just sum the delta