import board
import busio
from adafruit_tla202x import TLA2024

i2c = busio.I2C(board.SCL, board.SDA)
tla = TLA2024(i2c)

# for channel in range(3):
#     tla.input_channel = channel
#     print("Channel %d: %2f V"%(channel, tla.voltage))

tla.input_channel = 0
V1 = tla.voltage
tla.input_channel = 1
v2 = tla.voltage

V2 = 2*v2 - V1
print(f"V1: {V1}, V2: {V2}, {v2}")