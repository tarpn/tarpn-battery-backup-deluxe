# TARPN Battery Backup

Power board layout

```
 -[B]--[L]--[S]--[N]--[N]--[N]-
|  |    |    |                 |
|  |~S2~ ~S1~                  |
|  |                           |
|   ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ S3     |
|                        |     |
 -----------------------[P][N]-
```

Terminals:

* B: Battery Positive
* S: Supply Positive
* L: Load Positive
* N: Common Grounds
* P: Critical Load Positive
* S1: Supply Switch
* S2: Battery Switch
* S3: Critical Load Switch

Three p-channel mosfet switches are utilized to connect various loads to one of two DC power sources. 
Loads connected to the "L" terminal are either connected to the Power Supply "S" via S1, the Battery 
"B" via S2, or they are disconnected. A separate lower-current switch S3 is used for critical loads, 
such as a RaspberryPi or other low power control circuit.

All connected supplies and loads share a common ground. This ground is also shared with the control board. 
If a RaspberryPi (or other device) is monitoring the serial output of the control board, it must also share
this common ground.


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
power supply input. To avoid oscillations, the system will remain in the Recovery state for a certain
number of milliseconds before transitioning to Standby.

* S1: open 
* S2: closed
* S3: closed
