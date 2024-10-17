#!/usr/bin/env python3
import numpy as np
import math as m
from kelpie_hardware_interface.servo.Interface import calculate_4_bar
from kelpie_common.Config import Leg_linkage, Configuration
import csv


GDE_lower = np.deg2rad(33)
GDE_upper = np.deg2rad(150)
link = Leg_linkage(Configuration())
n = 1000
sweep = np.linspace(0, 140, n)
data_points = np.zeros((n, 3))

# THETA0_upper = 83
# if not (np.isnan(THETA0_upper) or THETA0_upper < 82):
#     print("YES")
# exit()

for i in range(n):
    THETA2 = np.deg2rad(sweep[i])
    CDA = m.pi + THETA2 - GDE_lower - link.EDC + link.gamma
    DAB, _, _ = calculate_4_bar(CDA, link.d, link.a, link.b, link.c)
    THETA0 = DAB + link.gamma
    THETA0_upper = 180 - np.rad2deg(THETA0)

    CDA = m.pi + THETA2 - GDE_upper - link.EDC + link.gamma
    DAB, _, _ = calculate_4_bar(CDA, link.d, link.a, link.b, link.c)
    THETA0 = DAB + link.gamma
    THETA0_lower = 180 - np.rad2deg(THETA0)

    data_points[i, :] = (np.rad2deg(THETA2),
                         THETA0_upper if not (np.isnan(THETA0_upper) or THETA0_upper > 82) else 82,
                         THETA0_lower if not (np.isnan(THETA0_lower) or THETA0_lower < -50) else -50,
                         )

FILE_NAME = __file__.replace("/workspace_boundary_gen.py", "/workspace_boundary_gen.csv")

with open(FILE_NAME, mode="w+", newline='') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',', quotechar='|')
    spamwriter.writerows(data_points)

# Triangle section
