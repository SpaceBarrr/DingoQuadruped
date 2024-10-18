#!/usr/bin/env python3
import math as m
import yaml
import os
import csv
import numpy as np
import rospy
from adafruit_servokit import ServoKit
from kelpie_common.Utilities import format_angles

class ServoInterface:
    def __init__(self, link):
        self.pwm_max = 2400
        self.pwm_min = 370
        self.link = link
        self.servo_angles = np.zeros((3, 4))
        self.kit = ServoKit(channels=16, address=0x60)  # Defininng a new set of servos uising the Adafruit ServoKit LIbrary
        self._workspace_boundary_LUT = {}
        self._build_workspace_boundary_LUT()
        """ SERVO INDICES, CALIBRATION MULTIPLIERS AND OFFSETS
            #   ROW:    which joint of leg to control 0:hip, 1: upper leg, 2: lower leg
            #   COLUMN: which leg to control. 0: front-right, 1: front-left, 2: back-right, 3: back-left.

                #               0                  1                2               3
                #  0 [[front_right_hip  , front_left_hip  , back_right_hip  , back_left_hip  ]
                #  1  [front_right_upper, front_left_upper, back_right_upper, back_left_upper]
                #  2  [front_right_lower, front_left_lower, back_right_lower, back_left_lower]] 

           'pins' define the physical pin of the servos on the servoboard """
        self.pins = np.array([[3, 9, 0, 6],
                              [4, 10, 1, 7],
                              [5, 11, 2, 8]])

        """ 'servo_multipliers' and 'complementary_angle' both work to flip some angles, x, to (180-x) so that movement on each leg is consistent despite
            physical motor oritentation changes """
        self.servo_multipliers = np.array(
            [[-1, 1, 1, -1],
             [1, -1, 1, -1],
             [1, -1, 1, -1]])
        
        self.complementary_angle = np.array(
            [[180, 0, 0, 180],
             [0, 180, 0, 180],
             [0, 180, 0, 180]])

        """ 'physical_calibration_offsets' are the angle required for the servo to be at their 'zero'locations. These zero locations
            are NOT the angles deifned in the IK, but rather locations that allow practical usage of the servo's 180 degree range of motion. 

            - Offsets for HIP servos allign the hip so that the leg is perfectly vertical at an input of zero degrees, direct from the IK.
            - Offsets for UPPER leg servos map allign the servo so that it is horizontal toward the back of the robot at an input of zero degrees, direct from the IK. 
            - Offsets for LOWER leg servos map allign the servo so that it is vertically down at zero degrees. Note that IK requires a transformation of
                angle_sent_to_servo = (180-angle_from_IK) + 90 degrees to map to this physcial servo location.  """
        
        DIR_PATH = os.path.dirname(os.path.realpath(__file__)).replace("/src/kelpie_hardware_interface/servo","/scripts")
        if os.path.isfile(f"{DIR_PATH}/calibrate_servo_angles.yaml"):
            with open(f"{DIR_PATH}/calibrate_servo_angles.yaml") as stream:
                raw_angles = yaml.load(stream, Loader=yaml.SafeLoader)
                offsets = {
                "fr r": raw_angles["fr"]["roll"], "fr u": raw_angles["fr"]["upper"], "fr l": raw_angles["fr"]["lower"],
                "fl r": raw_angles["fl"]["roll"], "fl u": raw_angles["fl"]["upper"], "fl l": raw_angles["fl"]["lower"],
                "rr r": raw_angles["rr"]["roll"], "rr u": raw_angles["rr"]["upper"], "rr l": raw_angles["rr"]["lower"],
                "rl r": raw_angles["rl"]["roll"], "rl u": raw_angles["rl"]["upper"], "rl l": raw_angles["rl"]["lower"]
            }
            self.physical_calibration_offsets = format_angles(offsets)
        else:
            rospy.logwarn("Could not find calibrate_servo_angles.yaml - Using defaults...")
            self.physical_calibration_offsets = np.zeros((3,4))
        # applying calibration values to all servos
        self.create()

    def create(self):
        for i in range(16):
            self.kit.servo[i].actuation_range = 180
            self.kit.servo[i].set_pulse_width_range(self.pwm_min, self.pwm_max)
    
    def _build_workspace_boundary_LUT(self):
        fname = __file__.replace("/src/kelpie_hardware_interface/servo/Interface.py", "/scripts/workspace_boundaries.csv")
        with open(fname, mode='r') as workspace_boundaries:
            reader = csv.reader(workspace_boundaries)
            self._workspace_boundary_LUT = {float(rows[0]):[float(rows[1]), float(rows[2])] for rows in reader}

    def set_actuator_postions(self, joint_angles):
        """Converts all angles found via inverse kinematics to the angles needed at the servo by applying multipliers
        and offsets for complimentary angles.
        It then outputs the correct angle to the servo via the adafruit servokit library. 

        Parameters
        ----------
        joint_angles : 3x4 numpy array of float angles (radians)
        """
        # Limit angles ot physical possiblity
        possible_joint_angles = self.impose_physical_limits(joint_angles)

        # Convert to servo angles
        angles = self.joint_angles_to_servo_angles(possible_joint_angles)

        # print('Final angles for actuation: ',self.servo_angles)    
        self.set_servo_angles(angles)

    def set_servo_angles(self, angles):
        # Adding final physical offset angles from servo calibration and clipping to 180 degree max
        angles = np.clip(angles + self.physical_calibration_offsets, 0, 180)
        # print('Unflipped servo_angles: ',self.servo_angles)

        # Accounting for difference in configuration of servos (some are mounted backwards)
        angles = np.round(np.multiply(angles, self.servo_multipliers) + self.complementary_angle, 1)
        for leg_index in range(4):
            for axis_index in range(3):
                try:
                    self.kit.servo[self.pins[axis_index, leg_index]].angle = angles[axis_index, leg_index]
                except Exception as error:
                    print(self.pins[axis_index, leg_index], angles[axis_index, leg_index])
                    print(error, end="\n\n")
                    rospy.logwarn("Warning - I2C IO error")

    ##  This method is used only in the calibrate servos file will make something similar to command individual actuators. 
    # def set_actuator_position(self, joint_angle, axis, leg):
    #     send_servo_command(self.pi, self.pwm_params, self.servo_params, joint_angle, axis, leg)
    def relax_motors(self, servo_list=np.ones((3, 4))):
        """Relaxes desired servos so that they appear to be turned off. 

        Parameters
        ----------
        servo_list : 3x4 numpy array of 1's and zeros. Row = Actuator; Column = leg.
                    If a Given actuator is 0 is 1 it should be deactivated, if it is 0 is should be left on. 
        """
        for leg_index in range(4):
            for axis_index in range(3):
                if servo_list[axis_index, leg_index] == 1:
                    self.kit.servo[self.pins[axis_index, leg_index]].angle = None

    def relax_all_motors(self):
        """Relaxes desired servos so that they appear to be turned off. 

        Parameters
        ----------
        servo_list : 3x4 numpy array of 1's and zeros. Row = Actuator; Column = leg.
                    If a Given actuator is 0 is 1 it should be deactivated, if it is 0 is should be left on. 
        """
        for leg_index in range(4):
            for axis_index in range(3):
                self.kit.servo[self.pins[axis_index, leg_index]].angle = None


    def joint_angles_to_servo_angles(self, joint_angles):
        """Converts joint found via inverse kinematics to the angles needed at the servo using linkage analysis.

        Parameters
        ----------
        joint_angles : 3x4 numpy array of float angles (radians)

        Returns
        -------
        servo_angles: 3x4 numpy array of float angles (degrees)
            The angles to be commanded of perfectly calibrated servos, rounded to 1dp
        
        """

        servo_angles = np.zeros((3, 4))
        for leg in range(4):
            THETA2, THETA3 = joint_angles[1:, leg]

            THETA0 = lower_leg_angle_to_servo_angle(self.link, m.pi / 2 - THETA2,
                                                    THETA3 + np.pi / 2)  # TODO draw a diagram to describe this transformatin from IK frame to LINK analysis frame

            # Adding offset from IK angle definition to servo angle definition, and conversion to degrees
            servo_angles[0, leg] = m.degrees(joint_angles[0, leg])  # servo zero is same as IK zero
            servo_angles[1, leg] = m.degrees(THETA2)  # servo zero is same as IK zero
            servo_angles[2, leg] = m.degrees(m.pi / 2 + m.pi - THETA0)  # servo zero is different to IK zero
        # print('Uncorrected servo_angles: ',self.servo_angles)

        return servo_angles
    
    def impose_physical_limits(self, desired_joint_angles):
        ''' Takes desired upper and lower leg angles and clips them to be within the range of physical possiblity. 
        This is because some angles are not possible for the physical linkage. Processing is done in degrees.
            ----------
        desired_joint_angles : numpy array 3x4 of float angles (radians)
            Desired angles of all joints for all legs from inverse kinematics

        Returns
        -------
        possble_joint_angles: numpy array 3x4 of float angles (radians)
            The angles that will be attempted to be implemeneted, limited to a possible range
        '''
        # return desired_joint_angles
        possible_joint_angles = np.zeros((3, 4))
        
        for i in range(4):
            hip, upper, lower = np.degrees(desired_joint_angles[:, i])
            print(upper, self._workspace_boundary_LUT[round(upper)]) if i == 0 else 0
            hip = np.clip(hip, -20, 20)
            upper = np.clip(upper, 0, 120)
            max_lower, min_lower = self._workspace_boundary_LUT[round(upper)]
            lower = np.clip(lower, min_lower, max_lower)

            possible_joint_angles[:, i] = hip, upper, lower
        return np.radians(possible_joint_angles)

### FUNCTIONS ###

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


def lower_leg_angle_to_servo_angle(link, THETA2, THETA3):
    ''' Converts the direct angles of the upper and lower leg joint from the inverse kinematics to the angle
    at the servo that drives the lower leg via a linkage. 
    Parameters
        ----------
    THETA2 : float
        angle of upper leg from the IK 
    THETA3 : float
        angle of lower leg from the IK 
    link: Leg_linage
        A linkage class with all link lengths and relevant angles stored. Link values are based off
        the physcial design of the link

    Returns
    -------
    THETA0: float
        The angle of the servo that drives the outside of the linkage
    '''

    # First 4 bar linkages
    GDE, DEF, EFG = calculate_4_bar(THETA3 + link.lower_leg_bend_angle, link.i, link.h, link.f,
                                    link.g)  # + link.lower_leg_bend_angle
    # Triangle section
    CDH = 3 / 2 * m.pi - THETA2 - GDE - link.EDC
    CDA = CDH + link.gamma  # input angle
    # Second 4 bar linkage
    DAB, ABC, BCD = calculate_4_bar(CDA, link.d, link.a, link.b, link.c)
    # Calculating Theta
    THETA0 = DAB + link.gamma
    # print(GDE)
    return THETA0




