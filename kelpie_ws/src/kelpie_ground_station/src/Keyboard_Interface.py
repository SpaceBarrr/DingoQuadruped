# 8 axes: used all except axes[2] and axes[5]
# 11 buttons: only use buttons[0], buttons[4] and buttons[5]

# joystick mapping
gait_toggle = msg.buttons[5] # R1
hop_toggle = msg.buttons[0] # x
joystick_toggle = msg.buttons[4] # L1
x_vel = (msg.axes[1]) * self.config.max_x_velocity  # ly
y_vel = msg.axes[0] * self.config.max_y_velocity # lx
self.developing_command.yaw_rate = np.round(msg.axes[3], self.rounding_dp) * self.config.max_yaw_rate  # rx
self.developing_command.pitch = np.round(msg.axes[4], self.rounding_dp) * self.config.max_pitch  # ry
self.developing_command.height_movement = np.round(msg.axes[7], self.rounding_dp)  # dpady
self.developing_command.roll_movement = -np.round(msg.axes[6], self.rounding_dp)  # dpadx

# keyboard mapping