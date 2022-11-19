# TARPN BBD Firmware

To build the firmware use the included Makefile. `make init` must be run once 
prior to building

```
make init
```

Then, to build the firmware use the "clean" and "build" targets

```
make clean build
```

To produce a Python package with the firmware and bootloader bundled, use the "dist" target:

```
make dist
```

## tarpn-bbd-firmware Python package

This directory contains a "virtual" Python package that is used for distributing
the firmware. This is a Python package that does not contain any Python programs, 
but rather contains the binary files for the BBD firmware and bootloader. A 
"virtual" Python package was created to make it easy to manage different releases 
of the firmware using pip.

To install the latest firmware, use "pip install --upgrade"

```
pip install --upgrade --index-url https://pypi.mumrah.synology.me/simple tarpn-bbd-firmware
```

The version scheme used for this package (and the firmware) follows the PEP440
convention: <MAJOR>.<MINOR>[.<SEGMENT>]

* MAJOR version indicates the hardware compatibility version. This will rarely change
* MINOR version indicates a normal revision of the firmware. This will change
  often as bugs are fixed and features are releases.
* SEGMENT is used for post-release and pre-release indicators.

