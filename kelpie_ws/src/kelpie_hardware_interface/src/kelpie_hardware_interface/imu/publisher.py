import board
from kelpie.msg import imu, att, vec_3d_float32
from kelpie_hardware_interface.imu.CircuitPython_LSM6DSV16X.lsm6dsv16x.lsm6dsv16x import LSM6DSV16X, FIFOMode
from transforms3d.euler import quat2euler
i2c = board.I2C()

IMU = LSM6DSV16X(i2c, sensor_fusion=True)
IMU.fifo_mode = FIFOMode.CONTINUOUS_WTM_TO_FULL_MODE
IMU.fifo_watermark = 6
IMU.sflp_game_vec_batch = True
# IMU.sflp_gravity_vec_batch = False
# IMU.sflp_g_bias_batch = False
IMU.sflp_init = True

IMU_MSG = imu()
IMU_MSG.att = att()
IMU_MSG.acc = vec_3d_float32()
IMU_MSG.gyro = vec_3d_float32()
IMU_MSG.gbias = vec_3d_float32()
IMU_MSG.grav = vec_3d_float32()




def publish(publisher):
    data = IMU.fifo_batch_data      # Fetch batch data as it is more efficient.
    acc = IMU.accelerometer
    gyro = IMU.gyroscope

    # TODO: Interpolate quat to euler.
    quat = data["SFLP_game_rotation_vector"]
    if quat is None:
        euler = [0, 0, 0]
    else:
        euler = quat2euler(quat)       # Yes this is stupid, but it is too ingrained in V1 code to switch to quaternions fully.
    # gbias = data["SFLP_gyroscope_bias"]
    # grav = data["SFLP_gravity_vector"]
    pitch, roll, yaw = euler
    IMU_MSG.att.roll, IMU_MSG.att.pitch, IMU_MSG.att.yaw = -roll, pitch, -yaw
    IMU_MSG.acc.x, IMU_MSG.acc.y, IMU_MSG.acc.z = acc[0], acc[1], acc[2]
    IMU_MSG.gyro.x, IMU_MSG.gyro.y, IMU_MSG.gyro.z = gyro[0], gyro[1], gyro[2]
    # IMU_MSG.gbias.x, IMU_MSG.gbias.y, IMU_MSG.gbias.z = gbias[0], gbias[1], gbias[2]
    # IMU_MSG.grav.x, IMU_MSG.grav.y, IMU_MSG.grav.z = grav[0], grav[1], grav[2]
    publisher.publish(IMU_MSG)
