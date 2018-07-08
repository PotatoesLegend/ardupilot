import os
import sys
import numpy as np
import utility

# Everything in this script uses SI units.

class CopterFrameType:
    QUAD_X = 0
    QUAD_PLUS = 1
    PENTA = 2

class RotorSpinDir:
    CW = -1
    CCW = 1 

def compute_quad_x_motor_matrix(torque_to_thrust_ratio, arm_length):
    motor_matrix = np.zeros((6, 4))
    motor_matrix[2, :] = -1.0
    offset = np.pi / 4.0
    for i, a in enumerate([offset, offset + np.pi, -offset, offset + np.pi / 2.0 ]):
        motor_matrix[3, i] = -arm_length * np.sin(a)
        motor_matrix[4, i] = arm_length * np.cos(a)
    for i, yaw_factor in enumerate([RotorSpinDir.CCW, RotorSpinDir.CCW, RotorSpinDir.CW, RotorSpinDir.CW]):
        motor_matrix[5, i] = yaw_factor * torque_to_thrust_ratio
    return motor_matrix

def compute_quad_plus_motor_matrix(torque_to_thrust_ratio, arm_length):
    motor_matrix = np.zeros((6, 4))
    motor_matrix[2, :] = -1.0
    angle = np.pi / 2.0
    for i, a in enumerate([angle, -angle, 0, angle * 2.0]):
        motor_matrix[3, i] = -arm_length * np.sin(a)
        motor_matrix[4, i] = arm_length * np.cos(a)
    for i, yaw_factor in enumerate([RotorSpinDir.CCW, RotorSpinDir.CCW, RotorSpinDir.CW, RotorSpinDir.CW]):
        motor_matrix[5, i] = yaw_factor * torque_to_thrust_ratio
    return motor_matrix

def compute_penta_motor_matrix(torque_to_thrust_ratio, arm_length):
    motor_matrix = np.zeros((6, 5))
    motor_matrix[2, :] = -1.0
    angle = 2.0 * np.pi / 5.0
    for i, a in enumerate([angle, angle * 2.0, 0.0, -angle * 2.0, -angle]):
        motor_matrix[3, i] = -arm_length * np.sin(a)
        motor_matrix[4, i] = arm_length * np.cos(a)
    for i, yaw_factor in enumerate([RotorSpinDir.CW, RotorSpinDir.CCW, RotorSpinDir.CW, RotorSpinDir.CW, RotorSpinDir.CCW]):
        motor_matrix[5, i] = yaw_factor * torque_to_thrust_ratio
    return motor_matrix

def compute_motor_matrix(frame_type, torque_to_thrust_ratio, arm_length):
    if frame_type == CopterFrameType.QUAD_X:
        return compute_quad_x_motor_matrix(torque_to_thrust_ratio, arm_length)
    elif frame_type == CopterFrameType.QUAD_PLUS:
        return compute_quad_plus_motor_matrix(torque_to_thrust_ratio, arm_length)
    elif frame_type == CopterFrameType.PENTA:
        return compute_penta_motor_matrix(torque_to_thrust_ratio, arm_length)
    else:
        utility.print_error('Unsupported frame type.')
        raise ValueError
    # Dummy return.
    return np.zeros((6, 0))

def compute_quad_x_moment_of_inertia(rotor_weight, arm_length):
    angle = np.pi / 4.0
    Ixx = Iyy = ((arm_length * np.sin(angle)) ** 2) * rotor_weight * 4
    Izz = rotor_weight * (arm_length ** 2) * 4
    return np.diag(np.array([Ixx, Iyy, Izz]))

def compute_quad_plus_moment_of_inertia(rotor_weight, arm_length):
    angle = np.pi / 2.0
    Ixx = Iyy = 2.0 * rotor_weight * (arm_length ** 2)
    Izz = 4.0 * rotor_weight * (arm_length ** 2)
    return np.diag(np.array([Ixx, Iyy, Izz]))

def compute_penta_moment_of_inertia(rotor_weight, arm_length):
    angle = 2.0 * np.pi / 5
    Ixx = rotor_weight * (arm_length ** 2) * (2 * (np.sin(angle) ** 2) + 2 * (np.sin(angle * 2) ** 2))
    Iyy = rotor_weight * (arm_length ** 2) * (1 + 2 * (np.cos(angle) ** 2) + 2 * (np.cos(angle * 2) ** 2))
    Izz = rotor_weight * (arm_length ** 2) * 5.0
    return np.diag(np.array([Ixx, Iyy, Izz]))

def compute_moment_of_inertia(frame_type, rotor_weight, arm_length):
    # rotor_weight is in kg.
    if frame_type == CopterFrameType.QUAD_X:
        return compute_quad_x_moment_of_inertia(rotor_weight, arm_length)
    elif frame_type == CopterFrameType.QUAD_PLUS:
        return compute_quad_plus_moment_of_inertia(rotor_weight, arm_length)
    elif frame_type == CopterFrameType.PENTA:
        return compute_penta_moment_of_inertia(rotor_weight, arm_length)
    else:
        utility.print_error('Unsupported frame type.')
        raise ValueError
    # Dummy return.
    return np.eye(3)

def compute_force_equilibrium(mass, motor_matrix):
    motor_num = motor_matrix.shape[1]
    g = 9.8
    # motor_matrix * u_eq = [0, 0, -mass * g, 0, 0, 0]
    target = np.array([0, 0, -mass * g, 0, 0, 0])
    u_eq, _, _, _ = np.linalg.lstsq(motor_matrix, target)
    u_eq = u_eq.flatten()
    if not np.allclose(np.dot(motor_matrix, u_eq), target):
        utility.print_error('Cannot find force equilibrium point')
        raise ValueError
    return u_eq

def linearize_copter(mass, moment_of_inertia, motor_matrix, x_eq, u_eq):
    # mass is a scalar in kg. moment_of_inertia is a 3 x 3 matrix in kgm^2.
    motor_num = motor_matrix.shape[1]
    # TODO: compute A and B.
    A = np.eye(12)
    B = np.eye(12, motor_num)
    return A, B

if __name__ == '__main__':
    # Add some tests here.
    pass
