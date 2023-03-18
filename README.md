# TARPN Battery Backup

This project is a DC power monitoring and backup system intended for use with TARPN nodes. It consists of two PCBs,
firmware for an ATMega328p, a Python script, and off-the-shelf open source monitoring software.

The control PCB monitors battery voltage, power supply voltage, temperature, and load current. It manages two high
powered switches on the power PCB which control whether the loads are connected to the power supply, battery, or neither.

A separate low powered switch is used to control the 12V supply for the RaspberryPi. 

# PCB Overview

![IMG_0660](https://user-images.githubusercontent.com/55116/211970858-cadd3850-cdff-4307-b690-b1d392b2d327.jpg)

Power board layout

```
 -[B]--[L]--[S]--[N]--[N]--[N]-
|  |    |    |                 |
|  |~S2~ ~S1~               [C]|
|  |                           |
|   ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ S3     |
|                        |     |
 -----------------------[P][N]-
```

Labels:

* B: Battery Positive
* S: Supply Positive
* L: Load Positive
* N: Common Grounds
* P: Critical Load Positive
* S1: Supply Switch
* S2: Battery Switch
* S3: Critical Load Switch
* C: 12-pin connector to control board


Three p-channel mosfet switches are utilized to connect various loads to one of two DC power sources. 
Loads connected to the "L" terminal are either connected to the Power Supply "S" via S1, the Battery 
"B" via S2, or they are disconnected. A separate lower-current switch S3 is used for critical loads, 
such as a RaspberryPi or other low power control circuit.

S1 and S2 are high powered DPAK mosfets with a very low on-resistance.

All connected supplies and loads share a common ground. This ground is also shared with the control board. 
If a RaspberryPi (or other device) is monitoring the serial output of the control board, it must also share
this common ground.


Control board layout

```
 ------------------------------
|         ___________      ___  |
|[C]     |   OLED    |    | R | |
|         -----------      ---  |
|    [RPi]               [ICSP] |
 ------------------------------
```

Labels:

* C: 12-pin connector to power board
* RPi: 10-pin connector to Raspberry Pi
* ICSP: Programming header for ATMega
* OLED: 128x32 OLED display
* R: Rotary encoder with switch





# Modes of operation

The digital electronics on the power board, the 5V regulator for the control board, and the critical load
output are powered directly by the battery. 

## Standby

When a power supply is attached and supplying a sufficient voltage, the system will be in Standby mode. 
Loads are powered by the power supply through S1.

* S1: closed
* S2: open
* S3: closed

## Backup

If a low voltage is detected on the power supply, the loads are transferred to the battery. This switching
time is typically in the microsecond range. The ability of the loads to maintain sufficient voltage during
the switching time is dependent on the load current and the available standby capacitance. 
C1 on the power board can be populated with a reasonably large (1000µF range) bulk capacitor. 

* S1: open 
* S2: closed
* S3: closed

## Recovery

Once in the Backup state, the system will enter the Recovery state if a sufficient voltage is seen at the
power supply input. To avoid oscillations, the system will remain in the Recovery state for a configured
number of milliseconds before transitioning to Standby.

* S1: open 
* S2: closed
* S3: closed

## Faults

The system monitors battery voltage to protect the loads from over-voltage and to protect the battery from
under-voltage. In these two cases, the battery is disconnected from the loads.

A temperature sensor is used to monitor the temperature of the PCB near the mosfet switches. If a high
temperature is detected, the switches are both opened to prevent the mosfets from over-heating and failing.

# Configuration

TODO

# Monitoring

TODO

# RaspberryPi Clean Shutdown

TODO

* S1: open 
* S2: open
* S3: closed
