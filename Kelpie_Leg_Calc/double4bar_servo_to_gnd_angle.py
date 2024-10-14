from math import pi, atan2, acos, cos, sin, sqrt
from numpy import deg2rad, rad2deg
# Taken from: https://ocw.metu.edu.tr/pluginfile.php/3961/mod_resource/content/12/ch3/3-6.htm
# Should be able to follow the same principle to reverse the calculation.
# Constant lengths of first and second linkage. Using dictionaries to improve code readability as it is mathematical``
# equations.
link1 = {"a1": 35, "a2": 35, "a3": 37.6307, "a4": 40}
link2 = {"a1": 130, "a2": 40, "a3": 130, "a4": 50}

servo_lower = 20  # Servo control for lower leg
servo_upper = 0    # Servo control for upper leg

# Convert physical 0 deg origin to mathematical 0 deg origin by doing 180 - (54.7681 + servo_lower, 54.7681 is taken
# from the CAD.
s1 = (link1["a2"] ** 2 + link1["a1"] ** 2 - 2 * link1["a2"] * link1["a1"] * cos(deg2rad(180 - (54.7681 + servo_lower)))) ** (1 / 2)
phi1 = pi - acos((link1["a1"] ** 2 + s1 ** 2 - link1["a2"] ** 2) / (2 * link1["a1"] * s1))
psi1 = acos((link1["a4"] ** 2 + s1 ** 2 - link1["a3"] ** 2) / (2 * link1["a4"] * s1))

# Theta4 is the angle between the fixed linkage and the rocker
theta4 = rad2deg(phi1 - psi1)
# Need to convert theta4 into theta2; which is the angle between the second crank and the second fixed linkage (For
# the second four bar linkage situated on the upper->lower leg assembly).
# 102.541 and 54.776 are fixed constants taken from the mechanical design (CAD)
theta2 = theta4 - 102.541 + 54.776 + servo_upper

s1 = (link2["a2"] ** 2 + link2["a1"] ** 2 - 2 * link2["a2"] * link2["a1"] * cos(deg2rad(theta2))) ** (1 / 2)
phi1 = pi - acos((link2["a1"] ** 2 + s1 ** 2 - link2["a2"] ** 2) / (2 * link2["a1"] * s1))
psi1 = acos((link2["a4"] ** 2 + s1 ** 2 - link2["a3"] ** 2) / (2 * link2["a4"] * s1))
theta_lower_leg_to_upper = rad2deg(phi1 - psi1)

print(theta4)
print(theta2)

# lower leg to upper is the angle between the lower leg and the upper leg (tibia and femur)
print(theta_lower_leg_to_upper)

# below is the angle between the lower leg and the gnd plane.
print(theta_lower_leg_to_upper - servo_upper)
