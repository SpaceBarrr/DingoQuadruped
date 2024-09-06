import sys
import os
import time
import board

sys.path.insert(0, 
                os.path.dirname(os.path.realpath(__file__)).replace('scripts','src') + \
                "/kelpie_hardware_interface/imu/Adafruit_CircuitPython_LSM6DS/adafruit_lsm6ds")
from lsm6dsv16x import LSM6DSV16X

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = LSM6DSV16X(i2c)

while True:
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyro))
    print(f"Temp: {sensor.temperature}")
    print(f"Quaternion: {sensor.quaternion}")
    print("")
    time.sleep(0.5)