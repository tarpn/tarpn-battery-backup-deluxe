import unittest

from tarpn.bbd.agent import build_net_message
from tarpn.bbd.spec import Parser, BBDState, ParsedBBDLine, BBDChargeState

LINES = [
b'DATA State=BACKUP           Battery=13.373  Supply=0.192   RPiOn=1 StateTime=275251 UpTime=278685 DT=2009 Git=1.1.post1 Temperature=27.445 AH=0.00     AmpSecDelta=0.0007   BatteryAmpSec=0.0000   AmpAvg=0.0004   AmpMax=0.0890   WattSecDelta=0.0099   WattAvg=0.0049  \r\n'
]


class CapturingParser(Parser):
    def __init__(self):
        self.captured = []

    def handle_line(self, line: ParsedBBDLine):
        self.captured.append(line)


class SpecTest(unittest.TestCase):
    def test_parse_line(self):
        parser = CapturingParser()
        parser.parse(b"DATA State=BACKUP           Battery=12.990  Supply=0.114   RPiOn=1 StateTime=477973 UpTime=481407 DT=2012 Temperature=31.502 AH=0.00     AmpSecDelta=0.0692   BatteryAmpSec=0.0000   AmpAvg=0.0344   AmpMax=0.5952   WattSecDelta=0.9042   WattAvg=0.4494")
        line = parser.captured.pop()
        self.assertEqual(line.state, BBDState.Backup)
        self.assertEqual(line.battery_volts, 12.990)
        self.assertEqual(line.supply_volts, 0.114)
        self.assertEqual(line.rpi_on, True)
        self.assertEqual(line.uptime_ms, 481407)

    def test_parse_examples(self):
        parser = CapturingParser()
        for line in LINES:
            parser.parse(line)

    def test_ip_address(self):
        ip = b"192.168.0.1"
        msg = build_net_message(ip)
        assert msg == b'N \xc0\xa8\x00\x01\n'

    def test_charge_states(self):
        assert BBDChargeState(0) == BBDChargeState.Off
        assert BBDChargeState(1) == BBDChargeState.Init
        assert BBDChargeState(2) == BBDChargeState.CC
        assert BBDChargeState(3) == BBDChargeState.CV


if __name__ == '__main__':
    unittest.main()
