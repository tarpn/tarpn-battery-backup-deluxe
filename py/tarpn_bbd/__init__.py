import argparse
import configparser
from importlib_metadata import version, PackageNotFoundError
import logging
import logging.config
import os
import re
import shutil

import serial
from prometheus_client import Counter, Gauge, Histogram, Enum, start_http_server


try:
    __version__ = version("tarpn-bbd")
except PackageNotFoundError:
    # package is not installed
    pass


logger = logging.getLogger('root')

state_enum = Enum("board_state", "The internal state of the board", states=["Null","Initializing","Standby","Backup","Recovery","BatteryLow","BatteryLowTrip","BatteryHigh","BatteryHighTrip","OverTemp","OverTempRecovery", "BeginShutdown", "RPiShutdown", "ShutdownComplete"])

battery_volts = Gauge("battery_volts", "Battery voltage")
supply_volts = Gauge("supply_volts", "Power Supply voltage")
load_amps = Gauge("load_amps", "Load current")
load_amp_hour = Gauge("load_ah", "Total load amp-hours")
load_amp_sec = Gauge("load_amp_sec", "Delta amp-seconds")
temp = Gauge("temperature_celcius", "Board temperature")
rpi_on = Gauge("rpi_on", "RaspberryPi On")
serial_lines = Counter("serial_lines", "Number of serial lines read since start")
serial_errors = Counter("serial_errors", "Number of serial errors since start")

def parse_line(line):
    logger.debug(line)
    serial_lines.inc()
    toks = re.split("\s+", line)
    state = toks[0]
    data = {}
    for tok in toks[1:]:
        kv = tok.split("=", 1)
        data[kv[0]] = float(kv[1])
    state_enum.state(state)
    battery_volts.set(data.get("Battery"))
    supply_volts.set(data.get("Supply"))
    load_amps.set(data.get("AmpAvg", 0.0))
    load_amp_hour.set(data.get("AH", 0.0))
    temp.set(data.get("Temperature"))
    rpi_on.set(data.get("RPiOn", 0))
    load_amp_sec.set(data.get("AmpSecDelta", 0))
  

def main():
    parser = argparse.ArgumentParser(description="Run the TARPN BDD metrics scraper")
    parser.add_argument("config", nargs="?", default="config/bbd.ini", help="Config file")

    parser.add_argument("--version", action="store_true", help="Print the version and exit")
    args = parser.parse_args()

    if args.version:
        print(__version__)
        exit(0)

    # Bootstrap config file
    if not os.path.exists(args.config) and os.path.basename(args.config) == "bbd.ini":
        shutil.copyfile("config/bbd.ini.sample", args.config)

    # Load config
    config_defaults = {
      "log.dir": "/opt/tarpn/logs",
      "log.config": "config/logging.ini"
    } 

    print(f"Loading config from {args.config}") 
    config_parser = configparser.ConfigParser(defaults=config_defaults,
                                              interpolation=configparser.ExtendedInterpolation(),
                                              inline_comment_prefixes=";",
                                              default_section="default")
    config_parser.read(args.config)
    config = config_parser["default"]
    
    logging_config_file = config.get("log.config", "not_set")
    if logging_config_file != "not_set":
        log_dir = config.get("log.dir")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        print(f"Loading log configuration from {logging_config_file}")
        logging.config.fileConfig(
            logging_config_file, defaults={"log.dir": log_dir}, disable_existing_loggers=False)
    
    logger.info(f"Starting up TARPN BDD scraper {__version__}")
    http_port = config.getint("prometheus.port", 8000)
    logger.info(f"Starting Prometheus server on port {http_port}")
    start_http_server(http_port)

    port = config.get("serial.port")
    speed = config.getint("serial.speed")
    logger.info(f"Opening serial port {port} at {speed}")
    ser = serial.Serial(None, speed)
    ser.port = port
    try:
        ser.open()
    except Exception as e:
        logger.error(e)
        logger.info(f"Could not open serial port {port}, exiting")
        exit(1)
   
    logger.info("Starting read loop")
    while True:
        line = ser.readline()
        try:    
            parse_line(line.decode("utf-8").strip())
        except Exception as e:
            logger.error(e)
            serial_errors.inc()
            continue

    logger.info("Exiting")

if __name__ == "__main__":
    main()
