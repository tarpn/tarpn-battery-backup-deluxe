# TARPN Battery Backup Deluxe

An open source DC-DC Uninterrupted Power Supply for TARPN. 

📖 Click here for [Operation Manual](https://github.com/tarpn/tarpn-battery-backup-deluxe/wiki/Operation-Manual) 📖

Battery Backup Deluxe (BBD) is a system that monitors and switches DC power between a 
power supply and a battery. It is a bespoke power management solution for multi-port packet
radio nodes in the [TARPN project](http://tarpn.net). It is also generally useful for any 12V
systems including non-TARPN Amateur Radio activities.

Supported Features:
* High current capability (30A continuous)
* Automatic switchover to battery
* Battery protection (low/high voltage cutoff)
* Over-temperature cutoff
* Separate switching of critical loads power for priority devices (3A max)
* Safe shutdown procedure of RaspberryPi

Experimental Features:
* Battery charging

Non-Features:
* Load distribution or fusing
* Integration with battery chargers (AC/DC chargers, or solar charge controllers)


Two custom PCBs are employed for voltage sensing, switching, and control. The two boards are
referred to as the Power Board and the Control Board. The Power Board connects to a regulated
DC power supply, a 12V battery, a 12V load, a DC buck regulator for a RaspberryPi, and the Control Board. 
The Control Board connects to a RaspberryPi using a communication cable. 


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


# Tuning

To account for variability of components used by the sensor circuits, the firmware includes several "tunables".

* X1: Current Sense. A scaling factor for the current shunt resistor R3. It should be set to 0.04 (20 * R3)
* X2: Current Offset. A voltage offset provided by the reference voltage U1. It should be set to 2.486
* X3: ADC Reference. A voltage reference provided by U5. Measured from AREF (typically 4.860)
* X4: Voltage Factor. A scaling factor for the two resistor divider networks R1+R4 and R8+R11. It should be set to R4 / (R1 + R4).
* X5: Voltage Offset. A voltage offset for the resistor divider networks. Set this to 0.0
* X6: Battery Factor. The same as X1, but for R36 (this may be deprecated in the future)
* X7: Battery Offset. The same as X2, but for the battery current shunt (this may be deprecated in the future)



# Configuration

TODO

# RaspberryPi Interface

J8 on the control board connects to pins 1 through 10 on the RaspberryPi. 

| Pin | RPi signal | Control board signal |
| --- | ---------- | -------------------- |
| 1   | +3V3       | used for pullup of pin 5 |
| 2   | +5V        | unused |
| 3   | GPIO 2     | RPI_PWR (active low) |
| 4   | +5V        | unused |
| 5   | GPIO 3     | RPI_SHUTDOWN (active low) |
| 6   | Ground     | Ground |
| 7   | GPIO 4     | Reset† |
| 8   | GPIO 14    | RXD (serial) |
| 9   | Ground     | Ground |
| 10  | GPIO 15    | TXD (serial) |

† This pin is sometimes cut on older TARPN installations. See below for workaround instructions.


The RPI_SHUTDOWN signal is used to trigger a shutdown of the RPi. When this signal is momentarily pulled low the RPi 
will perform a safe shutdown. A pull-up resistor between RPi pin 5 and RPi pin 1 is used to prevent this
signal from floating.

The RPI_PWR is used to inform the control board whether or not the RPi has power. When this signal is
low, the RPi is powered on. This signal allows the control board to determine when the RPi has powered off 
following a safe shutdown.

RXD and TXD are used for serial communication from the control board to the RPi. This is used for exporting
telemetry from the control board.

If pin 7 has been cut, a separate jumper between an unused GPIO on the RPi and the RESET line on the control 
board will be needed. Pin 5 of the ISCP header can be used to solder one end of a jumper. When doing firmware
uploads from the RPi, a different reset pin can be selected with the `--gpio` argument.

## RPi Hardware Modification

To accommodate the 10 pin ribbon connector on the RPi, pins 11 and 12 must be snipped or bent out of the way.

![IMG_1835](https://github.com/tarpn/tarpn-battery-backup-deluxe/assets/55116/62586ad9-2c98-45f4-abae-aaa8377520b3)


## RPi Hardware Configuration

To enable the functions of RPI_SHUTDOWN and RPI_PWR, two special configurations are needed on the RPi.
In the `/boot/config.txt` file, add the following lines below the comment 
`# Additional overlays and parameters are documented /boot/overlays/README`:

```
dtoverlay=gpio-poweroff,gpiopin=2,active_low=1
dtoverlay=gpio-shutdown,gpio_pin=3,active_low=1,gpio_pull=up
```

The hardward UART must also be enabled for communication with the control board. At the bottom of `/boot/config.txt`, add

```
[all]
enable_uart=1
```

This can be done when preparing the SD card or on a running RPi. If done on a running RPi, a reboot is necessary for the 
changes to take affect.

## RPi Software Setup

Assuming a starting point of Raspberry Pi OS (Debian Bullseye or Bookworm), we need to install a few dependencies.

```
sudo apt-get -y install python3-pip python3-venv supervisor prometheus prometheus-node-exporter lsof avrdude
```

Configure supervisor to be run as the "pi" user. Modify the file `/etc/supervisor/supervisord.conf` to add a "chown"
to the "unix_http_server" section:

```
[unix_http_server]
file=/var/run/supervisor.sock   ; (the path to the socket file)
chmod=0700                      ; sockef file mode (default 0700)
chown=pi:pi                     
```

Restart supervisor service
```
sudo service supervisor restart
```

Confirm supervisor is running and accessible

```
supervisorctl pid
```

Should return the PID (Process ID) of the supervisord program.

### Install tarpn-bbd Python package

Create an installation directory under `/opt` and install a Python environment into it.

_These instructions assume an installation directory of /opt/tarpn. However, the software can be installed at any location_

```
sudo mkdir /opt/tarpn
sudo chown pi:pi /opt/tarpn
python3 -m venv /opt/tarpn
```

Install the `tarpn-bbd-agent` program into the newly created Python environment

```
/opt/tarpn/bin/pip install --upgrade --index-url https://pypi.mumrah.synology.me/simple tarpn-bbd-agent
```

This will install the program and create several directories under `/opt/tarpn`. 

### Configure Supervisor

Once the tarpn-bbd Python package is installed, we can copy a supervisor config into the system's
configuration directory. This will allow for the Python program to be automatically run on RPi startup
and automatically restarted if it encounters a fatal error.

```
sudo cp /opt/tarpn/extra/tarpn-bbd-agent-supervisor.conf /etc/supervisor/conf.d/tarpn-bbd-agent.conf
```

This file tells supervisor how to run the tarpn-bbd-agent program. Issue a "reload" command to tell
supervisor to read this new config file

```
supervisorctl reload
```

Now the agent program should be running. Run "supervisorctl status" to confirm. The output should look like:

```
> supervisorctl status
tarpn-bbd-agent                  RUNNING   pid 15487, uptime 0:00:04
```

Check that the metrics are being exposed over HTTP. Visit http://localhost:9000 on the RPi, or http://<your-rpi-ip-address>:9000
from another computer on your network. You should see a plain text output that looks like:

```
# HELP python_gc_objects_collected_total Objects collected during gc
# TYPE python_gc_objects_collected_total counter
python_gc_objects_collected_total{generation="0"} 327.0
python_gc_objects_collected_total{generation="1"} 81.0
python_gc_objects_collected_total{generation="2"} 0.0
```

Next, check that the node metrics are being exposed by the prometheus-node-exporter. Visit http://localhost:9100
or http://<your-rpi-ip-address>:9100. This will have a lot more output than the other exporter. Here is a snippet

```
# HELP node_cpu_seconds_total Seconds the CPUs spent in each mode.
# TYPE node_cpu_seconds_total counter
node_cpu_seconds_total{cpu="0",mode="idle"} 2883.87
node_cpu_seconds_total{cpu="0",mode="iowait"} 5.04
node_cpu_seconds_total{cpu="0",mode="irq"} 0
node_cpu_seconds_total{cpu="0",mode="nice"} 0
node_cpu_seconds_total{cpu="0",mode="softirq"} 0.95
node_cpu_seconds_total{cpu="0",mode="steal"} 0t
node_cpu_seconds_total{cpu="0",mode="system"} 11.35
node_cpu_seconds_total{cpu="0",mode="user"} 22.08
```

### Python Configuration

The default configuration should work on most installations, but if a different serial port or HTTP port is needed, 
it can be changed in `/opt/tarpn/config/bbd.ini`

```
[default]
log.dir = /opt/tarpn/logs
log.config = config/logging.ini
serial.port = /dev/ttyS0
serial.speed = 38400
prometheus.port = 9000
```

After re-configuring, restart the service with supervisor

```
supervisorctl restart tarpn-bbd-agent
```

# Monitoring

If desired, a metrics collection database (Prometheus) and dashboard system (Grafana) can be used to visualize
the sensor data exported by the control board.

Prometheus needs network access to the RPi running `tarpn-bbd`, and Grafana needs network access to Prometheus.
On a home network, this is usually achieved without any extra effort. It is possible to run the TARPN stack 
(including `tarpn-bbd`) along with Prometheus and Grafana, but it is also possible to run Prometheus and Grafana
on a separate server if resources of the RPi become constained.

## Prometheus Configuration

Prometheus is an efficient time-series database that is optimized for metrics collection. By default, Prometheus 
is installed on the RPi that is also running TARPN and the Python agent. If desired, Prometheus can also be 
installed on another server (RPi or otherwise). Follow the 
[general installation instructions](https://prometheus.io/docs/prometheus/latest/installation/) for this.

Put the prometheus config in place

```
sudo cp /opt/tarpn/extra/prometheus.yml /etc/prometheus/prometheus.yml
```

And restart prometheus

```
sudo service prometheus restart
```

Launch the Prometheus UI and confirm the three targets are "UP". Load the UI by accessing http://localhost:9090 
on a web browser on the RPi, or http://<your-rpi-ip-address>:9090 from a web browser on another computer in your
network.

Click on Status, then Targets

<img width="960" alt="image" src="https://github.com/tarpn/tarpn-battery-backup-deluxe/assets/55116/9cc0acac-e0d7-4f8c-9001-9213153b9122">

And the three Targets should be in the "UP" State.

<img width="960" alt="image" src="https://github.com/tarpn/tarpn-battery-backup-deluxe/assets/55116/1a08ab31-84a5-4ec7-ade9-8fe8cefcb366">



## Grafana Install and Configuration

Follow the instructions to [install Grafana OSS](https://grafana.com/tutorials/install-grafana-on-raspberry-pi/#install-grafana).
Grafana can run on the TARPN RPi or on a separate server (RPi or otherwise).

For other installation types, follow the [general installation instructions](https://grafana.com/docs/grafana/latest/setup-grafana/installation/).

Copy some configuration files from the `tarpn-bbd` installation over to Grafana

```
sudo cp /opt/tarpn/extra/grafana.ini /etc/grafana/grafana.ini
sudo cp /opt/tarpn/extra/prometheus-datasource.yml /etc/grafana/provisioning/datasources/prometheus.yml
```

Enable Grafana service (as indicated the Grafana installation output). This will enable Grafana to start
when the RPi starts.

```
sudo /bin/systemctl daemon-reload
sudo /bin/systemctl enable grafana-server
```

Start Grafana (restart if it's already running)
```
sudo service grafana restart
```

Access the Grafana UI at http://localhost:3000 or `http://<your-rpi-ip-address>:3000`. 

<img width="1256" alt="image" src="https://github.com/tarpn/tarpn-battery-backup-deluxe/assets/55116/ad20f8eb-2b7a-4dbf-ae33-4c774eda3bcd">

Confirm the "prometheus" datasource was created


https://github.com/tarpn/tarpn-battery-backup-deluxe/assets/55116/4cb37a2d-a11b-4c16-b97d-481c0bf3d614

Next, import the dashboard

* Open https://github.com/tarpn/tarpn-battery-backup-deluxe/blob/main/py/extra/grafana-dashboard.json
* Click "Download Raw File"
* Open Grafana UI
* Navigate to Dashboards
* Click New -> Import
* Select the dashboard JSON file
* Select the "Prometheus" datasource we looked at earlier

https://github.com/tarpn/tarpn-battery-backup-deluxe/assets/55116/16305afa-259e-4ca9-8b7f-3a017b2bc0ff



