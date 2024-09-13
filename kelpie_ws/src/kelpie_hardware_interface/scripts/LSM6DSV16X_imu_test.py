import sys
import os
import time
import board
from transforms3d.euler import quat2euler

sys.path.insert(0,
                os.path.dirname(os.path.realpath(__file__)).replace('scripts', 'src') + \
                "/kelpie_hardware_interface/imu/CircuitPython_LSM6DSV16X/lsm6dsv16x")
from lsm6dsv16x import LSM6DSV16X, FIFOMode

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor: LSM6DSV16X = LSM6DSV16X(i2c, sensor_fusion=True)
sensor.fifo_mode = FIFOMode.CONTINUOUS_WTM_TO_FULL_MODE
sensor.fifo_watermark = 1
sensor.sflp_game_vec_batch = True
# sensor.sflp_gravity_vec_batch = False
# sensor.sflp_g_bias_batch = False
sensor.sflp_init = True
while True:
    # print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor.acc))
    # print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyration))
    # print(f"Temp: {sensor.temperature}")
    # print(f"Quaternion: {sensor.quaternion}")
    status = sensor.fifo_status
    val = sensor.quaternion
    data = sensor.fifo_batch_data 
    if val is None:
        continue
    print(f"quat data {data['SFLP_game_rotation_vector']}")
    # print(f"samples: {status.samples:3.0f}, x:{x:5.5f}, y:{y:5.5f}, z:{z:5.5f}", end="\r")
    time.sleep(0.1)
