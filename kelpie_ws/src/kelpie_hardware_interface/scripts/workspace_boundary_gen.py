#!/usr/bin/env python3
import numpy as np
import math as m
from kelpie_common.Config import Leg_linkage, Configuration
import csv


def calculate_4_bar(th2, a, b, c, d):
    """Using 'Freudensteins method', it finds all the angles within a 4 bar linkage with vertices ABCD and known link lengths a,b,c,d
    defined clockwise from point A, and known angle, th2.

    Parameters
    ----------
    th2 : float
        the input angle at the actuating joint of the 4 bar linkage, aka angle DAB
    a,b,c,d: floats
        link lengths, defined in a clockwise manner from point A.

    Returns
    -------
    ABC,BCD,CDA: floats
        The remaining angles in the 4 bar linkage

    """
    # print('th2: ',m.degrees(th2),'a: ',a,'b: ',b,'c: ',c,'d: ',d)
    x_b = a * np.cos(th2)
    y_b = a * np.sin(th2)

    # define diagnonal f
    f = np.sqrt((d - x_b) ** 2 + y_b ** 2)
    beta = np.arccos((f ** 2 + c ** 2 - b ** 2) / (2 * f * c))
    gamma = np.arctan2(y_b, d - x_b)

    th4 = np.pi - gamma - beta

    x_c = c * np.cos(th4) + d
    y_c = c * np.sin(th4)

    th3 = np.arctan2((y_c - y_b), (x_c - x_b))

    ## Calculate remaining internal angles of linkage
    ABC = np.pi - th2 + th3
    BCD = th4 - th3
    CDA = np.pi * 2 - th2 - ABC - BCD

    return ABC, BCD, CDA


GDE_lower = np.deg2rad(33)
GDE_upper = np.deg2rad(150)
link = Leg_linkage(Configuration())
n = 1000
start = 0
stop = 140
sweep = np.linspace(start, stop, n)
data_points = np.zeros((n, 3))


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
    csv_writer = csv.writer(csvfile, delimiter=',', quotechar='|')
    csv_writer.writerows(data_points)


