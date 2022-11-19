import argparse
import configparser
import dataclasses
import logging
import logging.config
import os
import queue
import shutil
import signal
import struct
import subprocess
import time
from datetime import datetime
from io import BytesIO
from typing import Dict, Callable

from prometheus_client import Counter, Gauge, Enum, Info, start_http_server

from tarpn.bbd.unix import AgentSocketProtocol, UnixServerThread
from tarpn.io import IOProtocol
from tarpn.io.serial import SerialDevice
from tarpn.logging import LoggingMixin, TRACE
from tarpn.thread.ring import RingBuffer
from tarpn.thread.scheduler import CloseableThreadLoop, Scheduler

from tarpn.bbd import __version__
from tarpn.bbd.spec import Parser, ParsedBBDLine, BBDState, BBDChargeState, BBDParseException

logger = logging.getLogger('root')

state_enum = Enum("board_state", "The internal state of the board", states=[
    "Null","Initializing","Standby","Backup","Recovery","BatteryLow",
    "BatteryLowTrip","BatteryHigh","BatteryHighTrip","OverTemp",
    "OverTempRecovery", "BeginShutdown", "RPiShutdown", "ShutdownComplete",
    "SelfTest"
])

charge_state_enum = Enum("batt_charge_state", "The status of the battery charger", states=[
    "Off", "Init", "CC", "Hold", "CV"
])


max_amps = None
min_amps = None

last_update_time = time.time()


@dataclasses.dataclass
class AgentEvent:
    pass


@dataclasses.dataclass
class AgentStartupEvent:
    pass


@dataclasses.dataclass
class AgentHeartbeat(AgentEvent):
    pass


@dataclasses.dataclass
class AgentSendIP(AgentEvent):
    pass


@dataclasses.dataclass
class BBDEvent(AgentEvent):
    event_type: str
    event_data: Dict[str, str]


@dataclasses.dataclass
class BBDData(AgentEvent):
    data: ParsedBBDLine


@dataclasses.dataclass
class BBDLog(AgentEvent):
    msg: str


@dataclasses.dataclass
class BBDConfig(AgentEvent):
    key: str
    value: str


@dataclasses.dataclass
class UnixSocketEvent(AgentEvent):
    msg: str


class BBDSerialProtocol(IOProtocol, Parser):
    def __init__(self, hardware_version: int, callback: Callable[[AgentEvent], None]):
        self.out_queue: RingBuffer[bytes] = RingBuffer(4)
        self.buffer = bytearray()
        self.callback: Callable[[AgentEvent], None] = callback
        self.hardware_version = hardware_version

    def send_to_bbd(self, data: bytes):
        self.out_queue.offer(data)

    # IOProtocol
    def handle_bytes(self, data: bytes) -> None:
        for b in data:
            self.buffer.append(b)
            if self.buffer.endswith(b"\r\n"):
                try:
                    self.parse(self.buffer, self.hardware_version)
                except BBDParseException:
                    logger.error(f"Could not parse buffer: {self.buffer}", )
                except Exception:
                    logger.error(f"Had an error while parsing buffer: {self.buffer}", )
                self.buffer.clear()

    def next_bytes_to_write(self, max_bytes: int) -> bytes:
        item, dropped = self.out_queue.take(timeout=0.100)  # Don't wait too long here, this blocks
        if item is not None:
            return item + b'\r\n'

    # Parser
    def handle_line(self, line: ParsedBBDLine):
        self.callback(BBDData(line))

    def handle_log(self, log: str):
        self.callback(BBDLog(log))

    def handle_event(self, event_type: str, data: Dict[str, str]):
        self.callback(BBDEvent(event_type, data))

    def handle_config(self, key: str, value: str):
        self.callback(BBDConfig(key, value))


class BBDAgentThread(CloseableThreadLoop, LoggingMixin):
    def __init__(self,
                 event_queue: queue.Queue,
                 unix_client: AgentSocketProtocol,
                 bbd_writer: Callable[[bytes], None],
                 enable_shutdown: bool,
                 hardware_version: int):
        CloseableThreadLoop.__init__(self, "agent")
        LoggingMixin.__init__(self, logging.getLogger("agent"))
        self.unix_client = unix_client
        self.bbd_writer = bbd_writer
        self.event_queue = event_queue
        self.enable_shutdown = enable_shutdown
        self.hardware_version = hardware_version
        self.event_queue.put(AgentStartupEvent())
        self.last_heartbeat = datetime.utcnow()
        self.first_bbd_line = True

    def enqueue_event(self, event: AgentEvent):
        self.event_queue.put(event)

    def deadline_sec(self, now: datetime) -> float:
        dt = (now - self.last_heartbeat).total_seconds()
        if dt > 3.0:
            self.event_queue.put(AgentHeartbeat())
            return 0.0
        else:
            return dt

    def iter_loop(self):
        now = datetime.utcnow()
        deadline = self.deadline_sec(now)
        event = None
        try:
            event = self.event_queue.get(timeout=deadline)
        except queue.Empty:
            pass

        if event is not None:
            self.debug(f"Processing event {event}")
            if isinstance(event, AgentStartupEvent):
                self.bbd_writer(b"P 1 Agent Start")
                self.event_queue.put(AgentSendIP())
            elif isinstance(event, AgentHeartbeat):
                self.bbd_writer(b"H")
                self.last_heartbeat = now
            elif isinstance(event, AgentSendIP):
                self.bbd_writer(b"P 2 Update IP")
                ip = get_local_ip()
                self.bbd_writer(build_net_message(ip))
            elif isinstance(event, BBDEvent):
                self.handle_bbd_event(event.event_type, event.event_data)
            elif isinstance(event, BBDLog):
                self.info(f"Log from firmware: {event.msg}")
                self.unix_client.send_if_connected(event.msg)
            elif isinstance(event, BBDConfig):
                self.info(f"Config change from firmware: {event.key} = {event.value}")
                self.unix_client.send_if_connected(f"CONFIG {event.key} {event.value}")
            elif isinstance(event, BBDData):
                self.handle_bbd_data(event.data)
            elif isinstance(event, UnixSocketEvent):
                self.handle_unix_socket_data(event.msg)
            else:
                self.warning(f"Ignoring unknown event with unknown type {event}")

    def handle_bbd_event(self, event_type: str, event_data: Dict[str, str]):
        self.info(f"Event from firmware: {event_type} {event_data}")
        self.unix_client.send_if_connected(f"EVENT {event_type} {event_data}")

        if event_type == "STARTUP":
            board_build_info.info(event_data)
            self.event_queue.put(AgentSendIP())

        elif event_type == "SHUTDOWN_RPI":
            if self.enable_shutdown:
                logger.info("RPi will shut down in 3 sec")
                self.bbd_writer(b"P 1 RPi is shutting")
                self.bbd_writer(b"P 2 down in 3 sec")
                time.sleep(3)
                logger.info("RPi will shut down now")
                subprocess.call("sudo shutdown -h now", shell=True)
            else:
                logger.info("RPi would shut down if shutdown.enabled was set.")
                self.bbd_writer(b"P 1 RPi not shutting")
                self.bbd_writer(b"P 2 down. Check config.")

    def handle_bbd_data(self, parsed_line: ParsedBBDLine):
        if self.first_bbd_line:
            self.unix_client.send_if_connected(f"EVENT STATE_CHANGE {parsed_line.state.value}".encode("ascii"))
            self.first_bbd_line = False
        global max_amps, min_amps, last_update_time
        serial_lines.inc()
        agent_build_info.info({"version": __version__})

        if parsed_line.state != BBDState.Unknown:
            state_enum.state(parsed_line.state.name)

        if parsed_line.charging_state != BBDChargeState.Unknown:
            charge_state_enum.state(parsed_line.charging_state.name)

        battery_volts.set(parsed_line.battery_volts)
        supply_volts.set(parsed_line.supply_volts)
        batt_amps.set(parsed_line.batt_amp_average)

        # Instead of directly updating a Prometheus gauge with these values
        # use global accumulators. This lets us report the accumulated value
        # when Prometheus scrapes (when the metric functions run).
        if max_amps is None:
            max_amps = parsed_line.amp_max
        else:
            max_amps = max(max_amps, parsed_line.amp_max)
        if min_amps is None:
            min_amps = parsed_line.amp_min
        else:
            min_amps = min(min_amps, parsed_line.amp_min)

        load_amps_gauge.labels(operation="avg").set(parsed_line.amp_average)

        load_amp_sec.inc(parsed_line.amp_second_delta)
        load_power.set(parsed_line.watt_avg)
        load_power_sec.inc(parsed_line.watt_second_delta)

        # Assume that the battery is connected to an external charger when the PSU is available
        # This means, we do not count discharging power in STANDBY states even if we see a voltage
        # on the battery current monitor. The assumption is this current is supplied by the trickle charge.
        if parsed_line.state not in (BBDState.Backup, BBDState.SelfTest):
            # Standby states (PSU is powering system)
            if parsed_line.charging_pwm == 0:
                # Not charging. Any discharge current is the power being drawn by the RPi
                psu_power_sec.inc(parsed_line.watt_second_delta)
                rpi_power_sec.inc(parsed_line.batt_watt_second_delta)
            else:
                # Charging the battery.
                # If we're charging, batt_watt_second_delta is negative. Subtract
                # this from the load power to get total PSU power. If batt_watt_second_delta
                # isn't negative, then don't account for it. This happens when the battery
                # is still supplying some current to the RPi even when the PWM is >0
                psu_power_sec.inc(parsed_line.watt_second_delta - min(0.0, parsed_line.batt_watt_second_delta))
                if parsed_line.batt_watt_second_delta < 0:
                    # If battery power is negative, it's charging. Report to the charging watt-seconds counter
                    batt_charge_batt_watt_sec_counter.inc(abs(parsed_line.batt_watt_second_delta))
                    batt_charge_amp_sec_counter.inc(abs(parsed_line.batt_amp_second_delta))
                else:
                    logger.warning("Unexpectedly saw battery discharging while charger was active.")
        else:
            # BACKUP or SELF_TEST mode
            non_load_power = parsed_line.batt_watt_second_delta - parsed_line.watt_second_delta
            if non_load_power > 0:
                rpi_power_sec.inc(non_load_power)
            else:
                logger.warning(f"Calculated non-positive power {non_load_power} in state {parsed_line.state}.")

            if parsed_line.batt_watt_second_delta < 0:
                # Don't report any apparent charging current when in backup mode.
                logger.warning(f"Unexpectedly saw battery charging current in {parsed_line.state}. Not reporting this")
            else:
                batt_discharge_watt_sec_counter.inc(parsed_line.batt_watt_second_delta)
                batt_discharge_amp_sec_counter.inc(parsed_line.batt_amp_second_delta)

        batt_charging_pwm.set(parsed_line.charging_pwm)
        temp.set(parsed_line.temperature_celsius)
        rpi_on.set(parsed_line.rpi_on)
        uptime._value.set(parsed_line.uptime_ms)
        agent_time_since_update_sec.set(time.time() - last_update_time)
        last_update_time = time.time()

    def handle_unix_socket_data(self, cmd: str):
        toks = cmd.split(" ", 2)
        cmd = toks[0]
        if cmd == "PRINT":
            num = int(toks[1])
            text = toks[2]
            if num not in (1, 2):
                logger.error(f"Can only send text to line 1 or 2. We got {num}.")
                self.unix_client.send_if_connected(f"ERROR Can only send text to line 1 or 2. We got {num}.")
            elif len(text) > 19:
                logger.error(f"Can only send up to 19 characters. We got {len(text)}.")
                self.unix_client.send_if_connected(f"ERROR Can only send up to 19 characters. We got {len(text)}.")
            else:
                logger.info(f"Sending text '{text}' to BBD")
                self.bbd_writer(f"P {num} {text}".encode("ascii"))
        elif cmd == "TEST":
            if len(toks) > 1:
                seconds = int(toks[1])
                if seconds > 65535:
                    self.unix_client.send_if_connected("ERROR Cannot specify seconds greater than 65535")
                    return
                logger.info(f"Starting self-test for {seconds} seconds.")
                packed = struct.pack(">H", seconds)
                buf = BytesIO()
                buf.write(b"T ")
                buf.write(packed)
                buf.seek(0)
                self.bbd_writer(buf.read())
            else:
                logger.info(f"Starting self-test.")
                self.bbd_writer(b"T")
        elif cmd == "GET":
            key = toks[1]
            logger.info(f"Getting config {key} from BBD")
            self.bbd_writer(f"G {key}".encode("ascii"))
        elif cmd == "SET":
            key = toks[1]
            val = toks[2]
            logger.info(f"Setting config {key} to {val}")
            int16 = int(float(val) * 1000.0)
            packed = struct.pack(">H", int16)
            buf = BytesIO()
            buf.write(f"S {key} ".encode("ascii"))
            buf.write(packed)
            buf.seek(0)
            self.bbd_writer(buf.read())
        elif cmd == "SAVE":
            logger.info(f"Sending EEPROM save command")
            self.bbd_writer(b"E")
        elif cmd == "LOAD":
            logger.info(f"Sending EEPROM load command")
            self.bbd_writer(b"L")
        elif cmd == "RESET":
            logger.info(f"Sending EEPROM reset command")
            self.bbd_writer(b"R")
        elif cmd == "VERSION":
            logger.info(f"Getting version from bbd")
            self.bbd_writer(b"V")
        else:
            logger.warning(f"Ignoring unknown command: {cmd}")


def get_min_amps() -> float:
    global min_amps
    _min_amps = min_amps
    min_amps = None
    return _min_amps or 0


def get_max_amps() -> float:
    global max_amps
    _max_amps = max_amps
    max_amps = None
    return _max_amps or 0


agent_build_info = Info("agent_build_info", "Agent Build Info")
board_build_info = Info("board_build_info", "Board Build Info")
battery_volts = Gauge("battery_volts", "Battery voltage")
supply_volts = Gauge("supply_volts", "Power Supply voltage")

load_amps_gauge = Gauge("load_amps", "Load Amps", ["operation"])
load_amps_gauge.labels(operation="min").set_function(get_min_amps)
load_amps_gauge.labels(operation="max").set_function(get_max_amps)

load_amp_sec = Counter("load_amp_sec", "Total amp-seconds")
load_power = Gauge("load_watt", "Average load power")
load_power_sec = Counter("load_watt_sec", "Total watt-seconds")

psu_power_sec = Gauge("psu_watt_sec", "PSU watt-seconds")
rpi_power_sec = Gauge("rpi_watt_sec", "RPi watt-seconds")

batt_amps = Gauge("batt_amps", "Average battery current")
batt_charging_pwm = Gauge("batt_charging_pwm", "PSU to Battery switch pwm")
batt_discharge_amp_sec_counter = Counter("batt_discharge_amp_sec", "Total battery amp-seconds discharged")
batt_discharge_watt_sec_counter = Counter("batt_discharge_watt_sec", "Total battery watt-seconds discharged")
batt_charge_amp_sec_counter = Counter("batt_charge_amp_sec", "Total battery charging amp-seconds")
batt_charge_batt_watt_sec_counter = Counter("batt_charge_watt_sec", "Total battery charging watt-seconds")

temp = Gauge("temperature_celsius", "Board temperature")
rpi_on = Gauge("rpi_on", "RaspberryPi On")
serial_lines = Counter("serial_lines", "Number of serial lines read since start")
serial_errors = Counter("serial_errors", "Number of serial errors since start")
uptime = Counter("board_uptime_msec", "Number of milliseconds of uptime")
agent_time_since_update_sec = Gauge("agent_time_since_update_sec", "Number of seconds since the agent was updated")


def get_local_ip() -> bytes:
    p = subprocess.run(r"""
                       ifconfig |
                       grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' |
                       grep -Eo '([0-9]*\.){3}[0-9]*' |
                       grep -v '127.0.0.1'""",
                       shell=True, stdout=subprocess.PIPE)
    return p.stdout.strip()


def build_net_message(ip: bytes) -> bytes:
    octets = ip.split(b".")
    buf = BytesIO()
    buf.write(b"N ")
    for octet in octets:
        buf.write(struct.pack("B", int(octet)))
    buf.write(b"")
    buf.seek(0)
    return buf.read()


def main():
    parser = argparse.ArgumentParser(description="Run the TARPN BDD agent")
    parser.add_argument("config", nargs="?", default="config/bbd.ini", help="Config file")
    parser.add_argument("-v", "--verbose", action="count", help="Enable debug logging on stdout", default=0)
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

    if args.verbose > 0:
        level = logging.DEBUG
        if args.verbose > 1:
            level = TRACE
        logger.setLevel(level)
        for handler in logger.handlers:
            handler.setLevel(level)

    logger.info(f"Starting up TARPN BDD scraper {__version__}")
    http_port = config.getint("prometheus.port", 8000)
    logger.info(f"Starting Prometheus server on port {http_port}")
    start_http_server(http_port)

    port = config.get("serial.port")
    speed = config.getint("serial.speed")

    event_queue = queue.Queue()

    def handle_unix_data(msg: str):
        event_queue.put(UnixSocketEvent(msg))

    hw_version = config.getint("hardware.version", 7)
    unix_protocol = AgentSocketProtocol(handle_unix_data)
    unix_thread = UnixServerThread(unix_protocol)
    serial_protocol = BBDSerialProtocol(hw_version, event_queue.put)
    agent = BBDAgentThread(
        event_queue=event_queue,
        unix_client=unix_protocol,
        bbd_writer=serial_protocol.send_to_bbd,
        enable_shutdown=config.getboolean("shutdown.enabled", False),
        hardware_version=hw_version
    )
    ser = SerialDevice(
        protocol=serial_protocol,
        port_id=0,
        device_name=port,
        speed=speed,
        read_timeout_s=0.100,
        write_timeout_s=0.100,
        read_limit=1024,
        bitrate=9600,   # rate limit the serial write
        max_buffer=1024
    )

    def signal_handler(signum, _):
        logger.info(f"Shutting down due to {signal.Signals(signum).name}")
        scheduler.shutdown()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    scheduler = Scheduler()
    ser.start_threads(scheduler)
    scheduler.submit(unix_thread)
    scheduler.submit(agent)
    logger.info("Finished BBD Agent Startup")
    scheduler.join()
    logger.info("Finished BBD Agent Shutdown")


if __name__ == "__main__":
    main()
