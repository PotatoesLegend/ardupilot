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

def lqr(A, B, Q, R, tol=1e-9, max_iter=100):
    n = A.shape[0]
    At = np.transpose(A)
    Bt = np.transpose(B)

    Z = np.vstack((np.hstack((A, np.dot(B, np.linalg.solve(R, Bt)))), np.hstack((Q, -At))))
    iteration = 0
    p = -1.0 / Z.shape[0]
    relative_norm = tol + 1.0 # Just ensure it is greater than tol at the beginning.
    while iteration < max_iter and relative_norm > tol:
        Z_old = np.copy(Z)
        Z *= np.power(np.abs(np.linalg.det(Z)), p)
        Z = Z - 0.5 * (Z - np.linalg.inv(Z))
        relative_norm = np.linalg.norm(Z - Z_old)
        iteration += 1
    if np.isnan(relative_norm) or relative_norm > tol:
        utility.print_error('Cannot solve K.') 
        raise ValueError

    W11 = Z[:n, :n]
    W12 = Z[:n, n:]
    W21 = Z[n:, :n]
    W22 = Z[n:, n:]

    lhs = np.vstack((W12, W22 + np.eye(n)))
    rhs = np.vstack((W11 + np.eye(n), W21))
    S, _, _, _ = np.linalg.lstsq(lhs, rhs)
    K = np.linalg.solve(R, np.dot(Bt, S))
    return K

def test_lqr(A, B, Q, R, K, rtol=1e-5, atol=1e-5):
    K0 = lqr(A, B, Q, R)
    if np.allclose(K, K0, rtol=rtol, atol=atol):
        utility.print_success('LQR test succeeded.')
    else:
        utility.print_error('LQR test failed.')
        print('expected K = ')
        print(K)
        print('actual K = ')
        print(K0)
        raise ValueError

if __name__ == '__main__':
    # Test LQR.
    # Quadcopter A, B, Q, R:
    A = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 9.81, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, -1.47901e-15, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1.47901e-15, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -2.43138e-15, 0, 0, 0, 0, 0, 0, 0, 0],
    ])
    B = np.array([[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [-0.789568, -0.789568, -0.789568, -0.789568],
        [0.00156214, -10.1913, 0.00156211, 10.1881],
        [10.4052, 0.331258, -9.74271, 0.331258],
        [-0.338956, 0.363457, -0.338956, 0.314455],
    ])
    Q = np.eye(12) * 10.0
    R = np.eye(4)
    K = np.array([[-2.30408, -0.000362873, -1.48028, -0.00844603, 13.2134, -1.58114, -3.39354, -0.000876037, -1.75113, -0.00489455, 2.57541, -2.19827],
        [-0.0702132, -2.23571, -1.57958, -12.8175, 0.392351, 1.58165, -0.102781, -3.29261, -1.86958, -2.49685, 0.0730018, 2.1985],
        [2.16365, -0.000362872, -1.67888, -0.00844603, -12.4287, -1.58114, 3.18798, -0.000876036, -1.98803, -0.00489455, -2.4294, -2.19827],
        [-0.0702132, 2.23643, -1.57958, 12.8344, 0.392351, 1.58063, -0.102781, 3.29436, -1.86958, 2.50664, 0.0730018, 2.19804],
    ])
    test_lqr(A, B, Q, R, K)

    # Pentacopter A, B, Q, R:
    A = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 9.81, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 3.32792e-16, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -3.32792e-16, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -8.1916e-15, 0, 0, 0, 0, 0, 0, 0, 0],
    ])
    B = np.array([[0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [-0.492893, -0.492893, -0.492893, -0.492893, -0.492893],
        [3.09937, -2.94662, 3.0604, -2.98461, 0.0175208],
        [-2.74762, -2.70897, 1.3786, 1.41691, 3.60805],
        [-0.0812917, 0.0774378, 0.072076, -0.0703091, -0.0728897],
    ])
    Q = np.eye(12) * 10.0
    R = np.eye(5)
    K = np.array([[1.53513, 1.60902, -1.45832, 9.8616, -9.54257, -1.71071, 2.31057, 2.41348, -1.78599, 2.04915, -2.04151, -3.78176],
        [1.5364, -1.55295, -1.58658, -9.54843, -9.53499, 1.29159, 2.31258, -2.33127, -2.03023, -1.99382, -2.00052, 2.92783],
        [-0.779363, 1.60461, -1.38145, 9.8543, 4.82567, 1.96648, -1.17176, 2.40792, -1.78745, 2.06018, 1.0342, 4.43084],
        [-0.777237, -1.55698, -1.36578, -9.56557, 4.76318, -1.03891, -1.16638, -2.33664, -1.68633, -2.00102, 0.974389, -2.28305],
        [-2.01776, 0.0216461, -1.2579, 0.135356, 12.4554, -0.67742, -3.03286, 0.0327585, -1.55883, 0.0260526, 2.5998, -1.47795],
    ])
    test_lqr(A, B, Q, R, K)

    # Vtail A, B, Q, R:
    A = np.array([[0, 0, 0, 0, 0, 0, 1, 2.03605e-27, 6.61413e-25, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0.999995, -0.00307832, 0, 0, 0],
        [0, 0, 0, 0, -0, 0, -6.61417e-25, 0.00307832, 0.999995, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 9.80995, -1.99736e-26, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -0.0301983, -6.48847e-24, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -3.82091e-42, 1.87662e-15, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -1.87662e-15, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -5.77686e-18, 1.24123e-39, 0, 0, 0, 0, 0, 0, 0],
    ])
    B = np.array([[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [-0.494939, 0, 0.494939, 0],
        [-0.494939, -0.69995, -0.494939, -0.69995],
        [3.61514, -7.99335, -3.54011, 7.88725],
        [-2.83289, 4.11031, -2.46932, 4.11543],
        [1.33693, 1.02504, -1.60819, -0.639179],
    ])
    Q = np.eye(12) * 10.0
    R = np.eye(4)
    K = np.array([[1.34545, 0.432376, -1.87183, 4.99102, -8.40221, 2.1211, 2.02617, 0.826243, -2.34776, 1.09457, -1.82723, 2.47247],
        [-1.81027, -2.21511, -1.25429, -11.9879, 10.8641, 0.492935, -2.69985, -3.20592, -1.54758, -2.39211, 2.20909, 0.352425],
        [1.14474, -0.398572, -1.83978, -4.96346, -7.02411, -2.26846, 1.71648, -0.803308, -2.30486, -1.10257, -1.48379, -2.63585],
        [-1.89796, 2.17887, -1.24025, 11.9599, 11.3704, -0.334737, -2.83063, 3.15655, -1.55017, 2.40058, 2.28072, -0.18408],
    ])
    test_lqr(A, B, Q, R, K)

    # Y6 A, B, Q, R:
    A = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 9.81, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, -3.7199e-16, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 3.7199e-16, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -8.68497e-15, 0, 0, 0, 0, 0, 0, 0, 0],
    ])
    B = np.array([[0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, -5.89573e-17, 0, -5.89573e-17, 0, -5.89573e-17],
        [-0.481423, -0.481423, -0.481423, -0.481423, -0.481423, -0.481423],
        [-4.49054, -4.51805, 4.6125, 4.58499, 0.118418, 0.145925],
        [1.83291, 1.83371, 1.74796, 1.74876, -3.7065, -3.70731],
        [0.169496, 0.357108, -0.355745, -0.168132, 0.0791327, -0.10848],
    ])
    Q = np.eye(12) * 10.0
    R = np.eye(6)
    K = np.array([[-0.891793, -1.55405, -1.33479, -9.30416, 5.44626, -1.29407, -1.33606, -2.31223, -1.65511, -1.99142, 1.13482, -2.51997],
        [-0.891903, -1.55897, -1.33495, -9.10417, 5.43936, 1.28791, -1.33593, -2.31084, -1.65528, -1.71291, 1.12558, 2.49729],
        [-0.89502, 1.60763, -1.2736, 9.39382, 5.48611, -1.28805, -1.34227, 2.38333, -1.57858, 1.76984, 1.14703, -2.49763],
        [-0.89513, 1.60271, -1.27376, 9.59381, 5.47921, 1.29393, -1.34215, 2.38472, -1.57874, 2.04834, 1.13778, 2.51963],
        [1.84479, 0.0238745, -1.26335, 0.239148, -11.2668, 1.29104, 2.76412, 0.0387135, -1.56555, 0.163025, -2.34138, 2.50901],
        [1.8449, 0.028787, -1.26319, 0.0391587, -11.2599, -1.29094, 2.764, 0.0373211, -1.56538, -0.115478, -2.33214, -2.50825],
    ])
    test_lqr(A, B, Q, R, K)
