# TARPN Battery Backup Deluxe

The Battery Backup Deluxe (BBD) is a system that monitors and switches DC power between a 
power supply and a battery. It is a bespoke power management solution for multi-port packet
radio nodes in the [TARPN project](http://tarpn.net).

Features:
* Automatic switchover to battery
* Battery protection (low/high voltage cutoff)
* Over-temperature cutoff
* Separate switching of low powered micro electronics and high powered loads
* Safe shutdown procedure of RaspberryPi

Non-Features:
* Battery charging
* Solar panel integration
* Load fusing

Two custom PCBs are employed for voltage sensing, switching, and control. The two boards are
referred to as the Power Board and the Control Board. The Power Board connects to a power supply,
a battery, the loads, a regulator for a RaspberryPi, and the Control Board. The Control Board
connects to a RaspberryPi. 

# Images

Power Board, bare PCB
![Power Board PCB](https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/images/PowerBoardLowRes.jpg)

Power Board, assembled and installed

![IMG_0660](https://user-images.githubusercontent.com/55116/211970858-cadd3850-cdff-4307-b690-b1d392b2d327.jpg)


# PCB Overview

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
"B" via S2, or they are disconnected. A separate low power switch S3 is used for critical loads, 
such as a RaspberryPi or other low power control circuits.

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


# Control Board Modes

The digital electronics on the power board, the 5V regulator for the Control Board, and the critical load
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

* S1: open 
* S2: open
* S3: closed


# Configuration

TODO

# Monitoring

TODO

# RaspberryPi Interface

J8 on the control board connects to pins 1 through 10 on the RaspberryPi. 

| Pin | RPi signal | Control board signal |
| --- | ---------- | -------------------- |
| 1   | +3V3       | used for pullup of pin 5 |
| 2   | +5V        | unused |
| 3   | GPIO 2     | RPI_PWR_INDICATOR (active low) |
| 4   | +5V        | unused |
| 5   | GPIO 3     | RPI_SIG (active low) |
| 6   | Ground     | Ground |
| 7   | GPIO 4     | unused† |
| 8   | GPIO 14    | RXD (serial) |
| 9   | Ground     | Ground |
| 10  | GPIO 15    | TXD (serial) |

† This pin is sometimes cut on older TARPN installations


The RPI_SIG signal is used to trigger a shutdown of the RPi. When this signal is momentarily pulled low the RPi 
will perform a safe shutdown. A pull-up resistor between RPi pin 5 and RPi pin 1 is used to prevent this
signal from floating.

The RPI_PWR_INDICATOR is used to inform the control board whether or not the RPi has power. When this signal is
low, the RPi is powered on. This signal allows the control board to determine when the RPi has powered off 
following a safe shutdown.

RXD and TXD are used for serial communication from the control board to the RPi. This is used for exporting
telemetry from the control board.


To enable the functions of RPI_SIG and RPI_PWR_INDICATOR, two special configurations are needed on the RPi.
In the /boot/config.txt file, add the following lines below the comment "# Additional overlays and parameters are documented /boot/overlays/README":

```
dtoverlay=gpio-poweroff,gpiopin=2,active_low=1
dtoverlay=gpio-shutdown,gpio_pin=3,active_low=1,gpio_pull=up
```

The hardward UART must also be enabled for communication with the control board. At the bottom of `/boot/config.txt`, add

```
[all]
enable_uart=1
```

This can be done when preparing the SD card or on a running RPi. A reboot is necessary for the changes to take affect.

## Monitoring

Assuming a starting point of Raspberry Pi OS (Debian Bullseye), we need to install a few dependencies.

```
sudo apt-get -y install python3-pip supervisor
```

Configure supervisor to be run as the "pi" user. Modify the file `/etc/supervisor/supervisord.conf` to add a "chown"
to the "unix_http_server" section:


```
[unix_http_server]
file=/var/run/supervisor.sock   ; (the path to the socket file)
chmod=0700                      ; sockef file mode (default 0700)
chown=pi:pi                     
```

Restart supervisor

```
sudo service supervisor restart
```

Create an installation directory under `/opt` and install a Python environment into it.

_These instructions assume an installation directory of /opt/tarpn. However, the software can be installed at any location_

```
sudo mkdir /opt/tarpn
sudo chown pi:pi /opt/tarpn
python3 -m virtualenv /opt/tarpn
```

Install the `tarpn-bbd` program into the newly created Python environment

```
/opt/tarpn/bin/pip install --upgrade --index-url https://pypi.mumrah.synology.me/simple tarpn-bbd
```

Copy the default config file

```
cp /opt/tarpn/config/bbd.ini.sample /opt/tarpn/config/bbd.ini
vi /opt/tarpn/config/bbd.ini
```

```
[default]
log.dir = /opt/tarpn/logs
log.config = config/logging.ini
serial.port = /dev/ttyUSB0
serial.speed = 9600
```

