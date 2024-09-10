from kelpie.msg import imu, att, xyz_float32
from CircuitPython_LSM6DSV16X.lsm6dsv16x.lsm6dsv16x import LSM6DSV16X, FIFOMode
import board

i2c = board.I2C()

IMU = LSM6DSV16X(i2c, sensor_fusion=True)
IMU.fifo_mode = FIFOMode.CONTINUOUS_WTM_TO_FULL_MODE
IMU.fifo_watermark = 6
IMU.sflp_game_vec_batch = True
IMU.sflp_gravity_vec_batch = True
IMU.sflp_g_bias_batch = True
IMU.sflp_init = True

IMU_MSG = imu()
IMU_MSG.att = att()
IMU_MSG.acc = xyz_float32()
IMU_MSG.gyro = xyz_float32()
IMU_MSG.grav = xyz_float32()

data = IMU.fifo_batch_data


def publish(publisher):
    print(data)
    # IMU_MSG.att.roll, IMU_MSG.att.pitch, IMU_MSG.att.yaw = att[0], att[1], att[2]
    # IMU_MSG.att.roll, IMU_MSG.att.pitch, IMU_MSG.att.yaw = att[2], att[0] - pi / 2, att[1]
    # IMU_MSG.acc.x, IMU_MSG.acc.y, IMU_MSG.acc.z = acc[0], acc[1], acc[2]
    # IMU_MSG.gyro.x, IMU_MSG.gyro.y, IMU_MSG.gyro.z = gyro[0], gyro[1], gyro[2]
    # IMU_MSG.grav.x, IMU_MSG.grav.y, IMU_MSG.grav.z = acc[0], acc[1], acc[2]
    # publisher.publish(IMU_MSG)


