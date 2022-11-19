import argparse
from importlib.metadata import version, PackageNotFoundError
from functools import partial
import os
import sys
from time import sleep
import shlex
import subprocess
import re


def check(fun, name, skip=False, allow_fail=False):
    try:
        s = f"Ensuring {name}..."
        print(f"{s: <60}", end="")
        if not skip:
            fun()
            print("\t\t✅ ok")
        else:
            print("\t\t🚫 skipped")
    except AssertionError as msg:
        if allow_fail:
            print(f"\t\t⚠️ failed: {msg}")
        else:
            print(f"\t\t❌ failed: {msg}")
            exit(1)
    except Exception as exp:
        print(f"\t\t❌ error: {exp}")
        exit(1)


def firmware_file():
    return os.path.join(sys.prefix, "extra/firmware.hex")


def ensure_firmware_file():
    try:
        ver = version("tarpn-bbd-firmware")
    except PackageNotFoundError:
        assert False, "Could not find 'tarpn-bbd-firmware' package"
    path = firmware_file()
    assert os.path.exists(path), f"Could not find firmware file at {path}"


def ensure_supervisor():
    p = subprocess.run(shlex.split("supervisorctl version"), stdout=subprocess.PIPE)
    assert p.returncode == 0


def check_agent_process() -> bool:
    out = subprocess.run(shlex.split("supervisorctl status tarpn-bbd-agent"), stdout=subprocess.PIPE).stdout
    return b"RUNNING" in out


def stop_agent_process():
    p = subprocess.run(shlex.split("supervisorctl stop tarpn-bbd-agent"), stdout=subprocess.PIPE)
    assert p.returncode == 0, "Could not stop agent program"
    assert not check_agent_process(), "Stopped agent program, but still shows RUNNING"


def start_agent_process():
    p = subprocess.run(shlex.split("supervisorctl start tarpn-bbd-agent"), stdout=subprocess.PIPE)
    assert p.returncode == 0, "Could not start agent program"
    assert check_agent_process(), "Started agent program, but not showing RUNNING"


def ensure_serial_port_free():
    p = subprocess.run(shlex.split("lsof /dev/ttyS0"), stdout=subprocess.PIPE)
    if p.returncode == 1:
        return
    else:
        lines = p.stdout.decode("ascii").split("\n")
        cols = re.split("\s+", lines[1])
        assert False, f"Process {cols[0]} (pid {cols[1]}) is using the serial port"


def reset_atmega(gpio: int) -> True:
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        assert False, "Missing RPi.GPIO dependency"

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(gpio, GPIO.OUT)

    GPIO.output(gpio, GPIO.LOW)
    sleep(0.500)
    GPIO.output(gpio, GPIO.HIGH)


def upload_firmware():
    path = firmware_file()
    p = subprocess.run(shlex.split(f"avrdude -v -p atmega328p -c arduino -b 38400 -D -P /dev/ttyS0 -U flash:w:{path}:i"))
    assert p.returncode == 0, "Firmware upload failed!"


def main():
    parser = argparse.ArgumentParser(description="Firmware updater for TARPN BBD")
    parser.add_argument("--gpio", type=int, default=7, help="Override the default GPIO used for Atmega reset (7).")
    parser.add_argument("--dry-run", action="store_true", help="Perform a dry-run of the firmware update")
    parser.add_argument("--reset-only", action="store_true", help="Only reset the atemga, do not update the firmware")

    args = parser.parse_args()
    if args.reset_only:
        check(partial(reset_atmega, args.gpio), "atmega is reset", skip=args.dry_run)
        exit(0)

    check(ensure_firmware_file, "firmware is present")
    check(ensure_supervisor, "supervisor is installed") 
    check(stop_agent_process, "agent is not running", allow_fail=True)
    check(ensure_serial_port_free, "serial port is unused") 
    check(partial(reset_atmega, args.gpio), "atmega is reset", skip=args.dry_run)
    check(upload_firmware, "firmware is uploaded", skip=args.dry_run)
    check(start_agent_process, "agent is running")

     
if __name__ == "__main__":
    main()
