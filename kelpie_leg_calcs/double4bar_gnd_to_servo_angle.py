from math import pi, atan2, acos, cos, sin, sqrt
from numpy import deg2rad, rad2deg
# Taken from: https://ocw.metu.edu.tr/pluginfile.php/3961/mod_resource/content/12/ch3/3-6.htm
# Should be able to follow the same principle to reverse the calculation.
# Constant lengths of first and second linkage. Using dictionaries to improve code readability as it is mathematical
# equations.
link1 = {"a1": 35, "a2": 35, "a3": 37.6, "a4": 43}
# Swapped a2 and a4 around to achieve the reverse. The input theta is now the angle between the two legs.
link2 = {"a1": 130, "a2": 37, "a3": 130, "a4": 43}

theta_lower_leg_to_upper = 64.6    # Servo control for lower leg
servo_upper = 10    # Servo control for upper leg

# Convert physical 0 deg origin to mathematical 0 deg origin by doing 180 - (54.7681 + servo_lower, 54.7681 is taken
# from the CAD.
s1 = (link2["a2"] ** 2 + link2["a1"] ** 2 - 2 * link2["a2"] * link2["a1"] * cos(deg2rad(theta_lower_leg_to_upper))) ** (1 / 2)
phi1 = pi - acos((link2["a1"] ** 2 + s1 ** 2 - link2["a2"] ** 2) / (2 * link2["a1"] * s1))
psi1 = acos((link2["a4"] ** 2 + s1 ** 2 - link2["a3"] ** 2) / (2 * link2["a4"] * s1))

# TODO: do the rest below here.
# Theta4 is the angle between the fixed linkage and the rocker
theta4 = rad2deg(phi1 - psi1)
print(theta4)

