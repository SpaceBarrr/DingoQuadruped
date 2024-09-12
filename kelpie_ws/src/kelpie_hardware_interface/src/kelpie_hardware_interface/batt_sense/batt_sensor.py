import board
import busio
from adafruit_tla202x import TLA2024

class BattVSensor:
    def __init__(self, addr: int = 0x48):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tla = TLA2024(i2c, address=addr)

    @property
    def v0(self):
        self.tla.input_channel = 0
        return self.tla.voltage

    @property
    def v1(self):
        self.tla.input_channel = 1
        return self.tla.voltage

    @property
    def v2(self):
        self.tla.input_channel = 2
        return self.tla.voltage

    @property
    def v3(self):
        self.tla.input_channel = 3
        return self.tla.voltage

    @property
    def v0_batt(self):
        """
        Assumes direct battery cell voltage
        """
        return self.v0

    @property
    def v1_batt(self):
        """
        Assumes resistor divider for half cell voltage
        """
        V0 = self.v0
        v1 = self.v1
        return 2 * v1 - V0

    @property
    def v2_batt(self):
        """
        Assumes resister divider for 1/3 cell voltage
        """
        V0 = self.v0
        V1 = self.v1_batt
        v2 = self.v2
        return 3 * v2 - V1 - V0

    @property
    def v3_batt(self):
        """
        Assumes resistor divider for 1/4 cell voltage
        """
        V0 = self.v0
        V1 = self.v1_batt
        V2 = self.v2_batt
        v3 = self.v3
        return 4 * v3 - V2 - V1 - V0


if __name__ == '__main__':
    bs = BattVSensor()
    print(f"V0_batt: {bs.v0_batt}, V1_batt: {bs.v1_batt}\nV0_raw: {bs.v0}, V1_raw: {bs.v1}\nV2_raw: {bs.v2*2}")
