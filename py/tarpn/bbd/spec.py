import dataclasses
import enum
import re
from typing import Dict, Optional


class BBDState(enum.Enum):
    Unknown = ""
    Null = "NULL"
    Initializing = "INIT"
    Standby = "STANDBY"
    Backup = "BACKUP"
    Recovery = "RECOVERY"
    BatteryLow = "BATT_LOW"
    BatteryLowTrip = "BATT_LOW_TRIP"
    BatteryHigh = "BATT_HIGH"
    BatteryHighTrip = "BATT_HIGH_TRIP"
    OverTemp = "OVER_TEMP"
    OverTempRecovery = "OVER_TEMP_RECOVER"
    BeginShutdown = "BEGIN_SHUTDOWN"
    RPiShutdown = "RPI_SHUTDOWN"
    ShutdownComplete = "SHUTDOWN_COMPLETE"
    SelfTest = "SELF_TEST"

    @classmethod
    def _missing_(cls, value):
        return cls.Unknown


class BBDChargeState(enum.Enum):
    Unknown = -1
    Off = 0
    Init = 1
    CC = 2
    Hold = 3
    CV = 4

    @classmethod
    def _missing_(cls, value):
        return cls.Unknown

    @classmethod
    def from_str(cls, value: str):
        int_val = int(value)
        return cls(int_val)


@dataclasses.dataclass
class ParsedBBDLine:
    state: BBDState
    battery_volts: float
    supply_volts: float
    rpi_on: bool
    state_time_ms: int
    uptime_ms: int
    dt_ms: int
    temperature_celsius: float
    amp_hour: float
    amp_second_delta: float
    batt_amp_average: float
    batt_amp_second_delta: float
    batt_watt_second_delta: float
    amp_average: float
    amp_max: float
    amp_min: float
    watt_second_delta: float
    watt_avg: float
    build_version: str
    charging_pwm: int
    charging_state: BBDChargeState
    raw: str
    unparsed: Dict[str, str]


class BBDParseException(BaseException):
    pass


class Parser:
    def handle_line(self, line: ParsedBBDLine):
        return

    def handle_log(self, log: str):
        return

    def handle_event(self, event_type: str, data: Dict[str, str]):
        return

    def handle_config(self, key: str, value: str):
        return

    def parse(self, buffer: bytes, hw_version: int):
        try:
            line = buffer.decode("ascii").strip()
        except UnicodeDecodeError:
            raise BBDParseException("Could not read buffer as ASCII")

        if len(line) == 0:
            return

        line_type, rem = line.split(" ", 1)
        if line_type == "EVENT":
            tokens = re.split(r"\s+", rem)
            event_type = tokens[0]
            token_dict = {}
            for tok in tokens[1:]:
                kv = tok.split("=", 1)
                token_dict[kv[0]] = kv[1]
            self.handle_event(event_type, token_dict)
        elif line_type == "LOG":
            self.handle_log(rem)
        elif line_type == "CONFIG":
            k, v = rem.split(" ", 1)
            self.handle_config(k, v)
        elif line_type == "DATA":
            tokens = re.split(r"\s+", rem)
            token_dict = {}
            for tok in tokens:
                kv = tok.split("=", 1)
                token_dict[kv[0]] = kv[1]

            parsed = ParsedBBDLine(
                state=remove_as_type(token_dict, "State", BBDState, BBDState.Unknown),
                battery_volts=remove_as_type(token_dict, "Battery", float, 0),
                supply_volts=remove_as_type(token_dict, "Supply", float, 0),
                rpi_on=remove_as_type(token_dict, "RPiOn", bool, False),
                state_time_ms=remove_as_type(token_dict, "StateTime", int, 0),
                uptime_ms=remove_as_type(token_dict, "UpTime", int, 0),
                dt_ms=remove_as_type(token_dict, "DT", int, 0),
                temperature_celsius=remove_as_type(token_dict, "Temperature", float, 0),
                amp_hour=remove_as_type(token_dict, "AH", float, 0),
                amp_second_delta=remove_as_type(token_dict, "AmpSecDelta", float, 0),
                batt_amp_second_delta=remove_as_type(token_dict, "BattAmpSecDelta", float, 0),
                batt_amp_average=remove_as_type(token_dict, "BattAmpAvg", float, 0),
                batt_watt_second_delta=remove_as_type(token_dict, "BattWattSecDelta", float, 0),
                amp_average=remove_as_type(token_dict, "AmpAvg", float, 0),
                amp_max=remove_as_type(token_dict, "AmpMax", float, 0),
                amp_min=remove_as_type(token_dict, "AmpMin", float, 0),
                watt_second_delta=remove_as_type(token_dict, "WattSecDelta", float, 0),
                watt_avg=remove_as_type(token_dict, "WattAvg", float, 0),
                build_version=remove_as_type(token_dict, "Git", str, ""),
                charging_pwm=remove_as_type(token_dict, "BattPid", int, 0),
                charging_state=remove_as_type(token_dict, "ChargeState", BBDChargeState.from_str, BBDChargeState.Unknown),
                raw=line,
                unparsed=token_dict
            )

            if hw_version == 7:
                # HW 7 had reversed polarity on the current sensor which inverts the current measurements
                parsed.batt_amp_average *= -1.0
                parsed.batt_amp_second_delta *= -1.0
                parsed.batt_watt_second_delta *= -1.0

            self.handle_line(parsed)


def remove_as_type(data, key, fun, default):
    if key not in data:
        return default 
    value = data.pop(key)
    try:
        return fun(value)
    except ValueError:
        return default 



