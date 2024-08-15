clear all; clc; close all;
format long g
%% Set variables
link1 = sym('link1_a', [1,4]);
link2 = sym('link2_a', [1,4]);
syms theta_1 theta_2 Fx Fy
cam_offsets = deg2rad(-102.541 + 54.776);
servo_upper = theta_1;
servo_lower = theta_2;

%% Set constants
link1_consts = [35.0143 35.0000 37.6165 42.9981];
link2_consts = [130 43.0025 130.177 37];
servo_lower_offset = pi - (deg2rad(54.7681) + servo_lower);
l1 = 130;
l2 = 125.559;

%% First 4 bar linkage
s1 = (link1(2)^2 + link1(1)^2 - 2 * link1(2) * link1(1) * cos(servo_lower_offset))^(1 / 2);
phi1 = pi - acos((link1(1) ^ 2 + s1 ^ 2 - link1(2) ^ 2) / (2 * link1(1) * s1));
psi1 = acos((link1(4) ^ 2 + s1 ^ 2 - link1(3) ^ 2) / (2 * link1(4) * s1));

theta4 = phi1-psi1;
theta2 = theta4 + cam_offsets + servo_upper;

%% Secpmd 4 bar linkage
s2 = (link2(2) ^ 2 + link2(1) ^ 2 - 2 * link2(2) * link2(1) * cos(theta2)) ^ (1 / 2);
phi2 = pi - acos((link2(1) ^ 2 + s2 ^ 2 - link2(2) ^ 2) / (2 * link2(1) * s2));
psi2 = acos((link2(4) ^ 2 + s2 ^ 2 - link2(3) ^ 2) / (2 * link2(4) * s2));

% This is (almost) theta2 in direct kinematics
lower_to_upper = phi2 - psi2;

% Theta1 and Theta2 of direct kinematics
dk_theta1 = servo_upper;
dk_theta2 = pi - lower_to_upper;

% Actual DK using calculated theta's
dk = [
    l1*cos(dk_theta1)+l2*cos(dk_theta1 + dk_theta2)
    -l1*sin(dk_theta1)-l2*sin(dk_theta1 + dk_theta2)
];

% Jacobian and torque calculation
j = jacobian(dk, [theta_1, theta_2]);
torque = j.' *[Fx; Fy];


%% Calculate mock values
clc;
% Set upper and lower angles.
servo_upper_const = deg2rad(-10);
servo_lower_const = deg2rad(20);
force_x = 0; % In kg
force_y = 1.5; % In kg
g = 9.81

value = subs(lower_to_upper, [link1 link2 servo_upper servo_lower], [link1_consts link2_consts servo_upper_const servo_lower_const]);
theta2_val = double(rad2deg(value))

x_val = double(subs(dk(1), [link1 link2 servo_upper servo_lower], [link1_consts link2_consts servo_upper_const servo_lower_const]))
y_val = double(subs(dk(2), [link1 link2 servo_upper servo_lower], [link1_consts link2_consts servo_upper_const servo_lower_const]))

% Torque outputs as N-mm, convert to kg-cm
torque1 = abs(double(subs(torque(1), [link1 link2 servo_upper servo_lower Fx Fy], [link1_consts link2_consts servo_upper_const servo_lower_const force_x*g force_y*g])) * 0.0101971621)
torque2 = abs(double(subs(torque(2), [link1 link2 servo_upper servo_lower Fx Fy], [link1_consts link2_consts servo_upper_const servo_lower_const force_x*g force_y*g])) * 0.0101971621)

% Latex print, copy and paste results into online viewers.
ltx_x = latex(dk(1));
ltx_y = latex(dk(2));


