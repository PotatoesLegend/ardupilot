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
    offset = np.pi / 4.0
    angle = np.pi / 2.0
    moi = np.zeros((3, 3))
    for i in range(4):
        a = offset + i * angle
        v = np.array([np.cos(a), np.sin(a), 0]) * arm_length
        s = skew_matrix(v)
        moi -= np.dot(s, s) * rotor_weight
    return moi

def compute_quad_plus_moment_of_inertia(rotor_weight, arm_length):
    offset = 0
    angle = np.pi / 2.0
    moi = np.zeros((3, 3))
    for i in range(4):
        a = offset + i * angle
        v = np.array([np.cos(a), np.sin(a), 0]) * arm_length
        s = skew_matrix(v)
        moi -= np.dot(s, s) * rotor_weight
    return moi

def compute_penta_moment_of_inertia(rotor_weight, arm_length):
    offset = 0
    angle = 2.0 * np.pi / 5.0
    moi = np.zeros((3, 3))
    for i in range(5):
        a = offset + i * angle
        v = np.array([np.cos(a), np.sin(a), 0]) * arm_length
        s = skew_matrix(v)
        moi -= np.dot(s, s) * rotor_weight
    return moi

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
    g = 9.81
    # motor_matrix * u_eq = [0, 0, -mass * g, 0, 0, 0]
    target = np.array([0, 0, -mass * g, 0, 0, 0])
    u_eq, _, _, _ = np.linalg.lstsq(motor_matrix, target)
    u_eq = u_eq.flatten()
    if not np.allclose(np.dot(motor_matrix, u_eq), target):
        utility.print_error('Cannot find force equilibrium point')
        raise ValueError
    return u_eq

def rotate_x(roll):
    c = np.cos(roll)
    s = np.sin(roll)
    return np.array([[1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c]])

def rotate_y(pitch):
    c = np.cos(pitch)
    s = np.sin(pitch)
    return np.array([[c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c]])

def rotate_z(yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array([[c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0]])

def rpy_to_rotation(rpy):
    roll, pitch, yaw = rpy
    return np.dot(np.dot(rotate_z(yaw), rotate_y(pitch)), rotate_x(roll))

def rpy_to_rotation_partial_derivative(rpy):
    roll, pitch, yaw = rpy
    R_roll = rotate_x(roll)
    R_pitch = rotate_y(pitch)
    R_yaw = rotate_z(yaw)

    dR_roll = np.dot(np.array([[1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0]]), R_roll)
    dR_roll[0, 0] = 0.0
    dR_pitch = np.dot(np.array([[0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0]]), R_pitch)
    dR_pitch[1, 1] = 0.0
    dR_yaw = np.dot(np.array([[0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]]), R_yaw)
    dR_yaw[2, 2] = 0.0

    A_left = np.dot(np.dot(R_yaw, R_pitch), dR_roll)
    A_mid = np.dot(np.dot(R_yaw, dR_pitch), R_roll)
    A_right = np.dot(np.dot(dR_yaw, R_pitch), R_roll)
    return np.hstack((A_left, A_mid, A_right))

def rpy_rate_to_body_angular_rate_matrix(rpy):
    roll, pitch, yaw = rpy
    s_roll = np.sin(roll)
    c_roll = np.cos(roll)
    s_pitch = np.sin(pitch)
    c_pitch = np.cos(pitch)
    return np.array([[1.0, 0.0, -s_pitch],
        [0.0, c_roll, s_roll * c_pitch],
        [0.0, -s_roll, c_roll * c_pitch]])

def rpy_rate_to_body_angular_rate(rpy, rpy_dot):
    return np.dot(rpy_rate_to_body_angular_rate_matrix(rpy), rpy_dot)

def rpy_rate_to_body_angular_rate_matrix_derivative(rpy, rpy_dot):
    roll, pitch, _ = rpy
    s_roll = np.sin(roll)
    c_roll = np.cos(roll)
    s_pitch = np.sin(pitch)
    c_pitch = np.cos(pitch)
    roll_dot, pitch_dot, _ = rpy_dot
    return np.array([[0.0, 0.0, -c_pitch * pitch_dot],
        [0.0, -s_roll * roll_dot, c_roll * c_pitch * roll_dot - s_roll * s_pitch * pitch_dot],
        [0.0, -c_roll * roll_dot, -s_roll * c_pitch * roll_dot - c_roll * s_pitch * pitch_dot]])

def rpy_rate_to_body_angular_rate_matrix_derivative_partial_derivative(rpy, rpy_dot):
    roll, pitch, _ = rpy
    s_roll = np.sin(roll)
    c_roll = np.cos(roll)
    s_pitch = np.sin(pitch)
    c_pitch = np.cos(pitch)
    roll_dot, pitch_dot, _ = rpy_dot
    A_left = np.array([[0.0, 0.0, 0.0],
        [0.0, -c_roll * roll_dot, -s_roll * c_pitch * roll_dot - c_roll * s_pitch * pitch_dot],
        [0.0, s_roll * roll_dot, -c_roll * c_pitch * roll_dot + s_roll * s_pitch * pitch_dot]])
    A_mid = np.array([[0.0, 0.0, s_pitch * pitch_dot],
        [0.0, 0.0, c_roll * (-s_pitch) * roll_dot - s_roll * c_pitch * pitch_dot],
        [0.0, 0.0, -s_roll * (-s_pitch) * roll_dot - c_roll * c_pitch * pitch_dot]])
    return np.hstack((A_left, A_mid, np.zeros((3, 3)), rpy_rate_to_body_angular_rate_matrix_partial_derivative(rpy)))

def rpy_rate_to_body_angular_rate_matrix_partial_derivative(rpy):
    roll, pitch, _ = rpy
    s_roll = np.sin(roll)
    c_roll = np.cos(roll)
    s_pitch = np.sin(pitch)
    c_pitch = np.cos(pitch)
    A_left = np.array([[0.0, 0.0, 0.0],
        [0.0, -s_roll, c_roll * c_pitch],
        [0.0, -c_roll, -s_roll * c_pitch]])
    A_mid = np.array([[0.0, 0.0, -c_pitch],
        [0.0, 0.0, -s_roll * s_pitch],
        [0.0, 0.0, -c_roll * s_pitch]])
    return np.hstack((A_left, A_mid, np.zeros((3, 3))))

def rpy_rate_to_body_angular_rate_partial_derivative(rpy, rpy_dot):
    B = rpy_rate_to_body_angular_rate_matrix_partial_derivative(rpy)
    rpy_dot_col = rpy_dot.reshape(-1, 1)
    return np.hstack((np.dot(B[:, :3], rpy_dot_col), np.dot(B[:, 3:6], rpy_dot_col), np.dot(B[:, 6:], rpy_dot_col), rpy_rate_to_body_angular_rate_matrix(rpy)))

def skew_matrix(x):
    x0, x1, x2 = x
    return np.array([[0, -x2, x1],
        [x2, 0, -x0],
        [-x1, x0, 0]])

def rpy_rate_to_body_angular_rate_matrix_inverse(rpy):
    # Reference:
    # http://www.princeton.edu/~stengel/MAE331Lecture9.pdf.
    roll, pitch, yaw = rpy
    s_roll = np.sin(roll)
    c_roll = np.cos(roll)
    s_pitch = np.sin(pitch)
    c_pitch = np.cos(pitch)
    t_pitch = np.tan(pitch)
    return np.array([[1.0, s_roll * t_pitch, c_roll * t_pitch],
        [0.0, c_roll, -s_roll],
        [0.0, s_roll / c_pitch, c_roll / c_pitch]])

def linearize_copter(mass, moment_of_inertia, motor_matrix, x_eq, u_eq):
    # mass is a scalar in kg. moment_of_inertia is a 3 x 3 matrix in kgm^2.
    if not np.isscalar(mass):
        utility.print_error('expect to see a scalar mass.')
        raise ValueError
    if moment_of_inertia.shape[0] != 3 or moment_of_inertia.shape[1] != 3:
        utility.print_error('expect to see a 3 x 3 moment of inertia matrix.')
        raise ValueError
    if motor_matrix.shape[0] != 6:
        utility.print_error('expect to see a 6 x N motor matrix.')
        raise ValueError
    if len(x_eq) != 12:
        utility.print_error('expect to see a 12 dimensional x_eq.')
        raise ValueError
    motor_num = motor_matrix.shape[1]
    if len(u_eq) != motor_num:
        utility.print_error('rotor number and u_eq dimension mismatch.')
        raise ValueError
    A = np.zeros((12, 12))
    B = np.zeros((12, motor_num))

    # Make sure they are column vectors.
    xyz = x_eq[:3]
    rpy = x_eq[3:6]
    uvw = x_eq[6:9]
    rpy_dot = x_eq[9:]

    R = rpy_to_rotation(rpy)
    dR_drpy = rpy_to_rotation_partial_derivative(rpy)
    A[:3, 3] = np.dot(dR_drpy[:, :3], uvw)
    A[:3, 4] = np.dot(dR_drpy[:, 3:6], uvw)
    A[:3, 5] = np.dot(dR_drpy[:, 6:], uvw)
    A[:3, 6:9] = R

    A[3:6, 9:] = np.eye(3)

    body_w = rpy_rate_to_body_angular_rate(rpy, rpy_dot)
    d_body_w = rpy_rate_to_body_angular_rate_partial_derivative(rpy, rpy_dot)
    A[6:9, 3:6] += np.dot(skew_matrix(uvw), d_body_w[:, :3])
    A[6:9, 9:12] += np.dot(skew_matrix(uvw), d_body_w[:, 3:])
    A[6:9, 6:9] += -skew_matrix(body_w)

    g = 9.81
    z = np.array([0.0, 0.0, 1.0])
    A[6:9, 3] += np.dot(np.transpose(dR_drpy[:, :3]), g * z)
    A[6:9, 4] += np.dot(np.transpose(dR_drpy[:, 3:6]), g * z)
    A[6:9, 5] += np.dot(np.transpose(dR_drpy[:, 6:]), g * z)
    
    B[6:9, :] += motor_matrix[:3, :] / mass

    H = motor_matrix[3:, :]
    I0 = moment_of_inertia
    I0_inv = np.linalg.inv(I0)
    dL_drpy = rpy_rate_to_body_angular_rate_matrix_partial_derivative(rpy)
    L_inv = rpy_rate_to_body_angular_rate_matrix_inverse(rpy)
    dL_inv_drpy = np.hstack((-np.dot(np.dot(L_inv, dL_drpy[:, :3]), L_inv),
        -np.dot(np.dot(L_inv, dL_drpy[:, 3:6]), L_inv),
        -np.dot(np.dot(L_inv, dL_drpy[:, 6:]), L_inv)))
    L_dot = rpy_rate_to_body_angular_rate_matrix_derivative(rpy, rpy_dot)
    dL_dot = rpy_rate_to_body_angular_rate_matrix_derivative_partial_derivative(rpy, rpy_dot)
    B[9:, :] = np.dot(np.dot(L_inv, I0_inv), H)

    body_torque = np.dot(H, u_eq)
    body_w_dot = np.dot(I0_inv, body_torque - np.cross(body_w, np.dot(I0, body_w)))
    d_body_w_dot_d_rpy = np.dot(np.dot(I0_inv, skew_matrix(np.dot(I0, body_w)) - np.dot(skew_matrix(body_w), I0)), d_body_w[:, :3])
    for i in range(3):
        A[9:, 3 + i] += np.dot(dL_inv_drpy[:, 3 * i : 3 * i + 3], body_w_dot - np.dot(L_dot, rpy_dot))
        A[9:, 3 + i] += np.dot(L_inv, d_body_w_dot_d_rpy[:, i] - np.dot(dL_dot[:, 3 * i : 3 * i + 3], rpy_dot))

    d_body_w_dot_d_rpy_dot = np.dot(np.dot(I0_inv, skew_matrix(np.dot(I0, body_w)) - np.dot(skew_matrix(body_w), I0)), d_body_w[:, 3:])
    A[9:, 9:] += np.dot(L_inv, d_body_w_dot_d_rpy_dot - L_dot)
    A[9:, 9] += -np.dot(np.dot(L_inv, dL_dot[:, 9:12]), rpy_dot)
    A[9:, 10] += -np.dot(np.dot(L_inv, dL_dot[:, 12:15]), rpy_dot)
    A[9:, 11] += -np.dot(np.dot(L_inv, dL_dot[:, 15:]), rpy_dot)
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

def test_linearization(mass, moment_of_inertia, motor_matrix, x_eq, u_eq, A, B, rtol=1e-5, atol=1e-4):
    A0, B0 = linearize_copter(mass, moment_of_inertia, motor_matrix, x_eq, u_eq)
    if np.allclose(A, A0, rtol=rtol, atol=atol) and np.allclose(B, B0, rtol=rtol, atol=atol):
        utility.print_success('Linearization test succeeded.')
    else:
        utility.print_error('Linearization test failed.')
        print('expected A = ')
        print(A)
        print('actual A = ')
        print(A)
        print('expected B = ')
        print(B)
        print('actual B = ')
        print(B)
        raise ValueError

if __name__ == '__main__':
    # Test linearization and LQR.
    # Quadcopter data.
    mass = 1.26652
    moment_of_inertia = np.array([[0.0225721, -2.82212e-11, 0.000104025],
        [-2.82212e-11, 0.0228311, -1.7167e-11],
        [0.000104025, -1.7167e-11, 0.0432627],
    ])
    motor_matrix = np.array([[-0, -0, -0, -0],
        [-0, -0, -0, -0],
        [-1, -1, -1, -1],
        [4.50462e-10, -0.23, 4.50462e-10, 0.23],
        [0.237563, 0.007563, -0.222437, 0.007563],
        [-0.014664, 0.014664, -0.014664, 0.014664],
    ])
    x_eq = np.zeros(12)
    u_eq = np.array([2.90185, 3.10613, 3.3104, 3.10613])
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
    test_linearization(mass, moment_of_inertia, motor_matrix, x_eq, u_eq, A, B)
    Q = np.eye(12) * 10.0
    R = np.eye(4)
    K = np.array([[-2.30408, -0.000362873, -1.48028, -0.00844603, 13.2134, -1.58114, -3.39354, -0.000876037, -1.75113, -0.00489455, 2.57541, -2.19827],
        [-0.0702132, -2.23571, -1.57958, -12.8175, 0.392351, 1.58165, -0.102781, -3.29261, -1.86958, -2.49685, 0.0730018, 2.1985],
        [2.16365, -0.000362872, -1.67888, -0.00844603, -12.4287, -1.58114, 3.18798, -0.000876036, -1.98803, -0.00489455, -2.4294, -2.19827],
        [-0.0702132, 2.23643, -1.57958, 12.8344, 0.392351, 1.58063, -0.102781, 3.29436, -1.86958, 2.50664, 0.0730018, 2.19804],
    ])
    test_lqr(A, B, Q, R, K)

    # Pentacopter data.
    mass = 2.02884
    moment_of_inertia = np.array([[0.0926367, 0.000863829, 0.000301719],
        [0.000863829, 0.135731, -0.000149694],
        [0.000301719, -0.000149694, 0.223905],
    ])
    motor_matrix = np.array([[-0, -0, -0, -0, -0],
        [-0, -0, -0, -0, -0],
        [-1, -1, -1, -1, -1],
        [0.284718, -0.275282, 0.284718, -0.275282, 0.00471781],
        [-0.370248, -0.370248, 0.189752, 0.189752, 0.489752],
        [-0.0168552, 0.0168552, 0.0168552, -0.0168552, -0.0168552],
    ])
    x_eq = np.zeros(12)
    u_eq = np.array([3.34573, 5.12607, 4.82539, 3.38041, 3.22532])
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
    test_linearization(mass, moment_of_inertia, motor_matrix, x_eq, u_eq, A, B)
    Q = np.eye(12) * 10.0
    R = np.eye(5)
    K = np.array([[1.53513, 1.60902, -1.45832, 9.8616, -9.54257, -1.71071, 2.31057, 2.41348, -1.78599, 2.04915, -2.04151, -3.78176],
        [1.5364, -1.55295, -1.58658, -9.54843, -9.53499, 1.29159, 2.31258, -2.33127, -2.03023, -1.99382, -2.00052, 2.92783],
        [-0.779363, 1.60461, -1.38145, 9.8543, 4.82567, 1.96648, -1.17176, 2.40792, -1.78745, 2.06018, 1.0342, 4.43084],
        [-0.777237, -1.55698, -1.36578, -9.56557, 4.76318, -1.03891, -1.16638, -2.33664, -1.68633, -2.00102, 0.974389, -2.28305],
        [-2.01776, 0.0216461, -1.2579, 0.135356, 12.4554, -0.67742, -3.03286, 0.0327585, -1.55883, 0.0260526, 2.5998, -1.47795],
    ])
    test_lqr(A, B, Q, R, K)

    # Vtail data.
    mass = 1.428674
    moment_of_inertia = np.array([[0.029888, -2.9816e-11, 0.00879599],
        [-2.9816e-11, 0.0585, -6.22951e-12],
        [0.00879599, -6.22951e-12, 0.0839344],
    ])
    motor_matrix = np.array([[-0, -0, 0, -0],
        [-0.707107, -0, 0.707107, -0],
        [-0.707107, -1, -0.707107, -1],
        [0.119885, -0.23, -0.119885, 0.23],
        [-0.165482, 0.240637, -0.144744, 0.240637],
        [0.144744, 0.014664, -0.165482, 0.014664],
    ])
    x_eq = np.array([0, 0, -0.75, 0.00307832, 6.61417e-25, 0, 0, 0, 0, 0, 0, 0])
    u_eq = np.array([5.21382, 3.35836, 5.15281, 3.32655])
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
    test_linearization(mass, moment_of_inertia, motor_matrix, x_eq, u_eq, A, B)
    Q = np.eye(12) * 10.0
    R = np.eye(4)
    K = np.array([[1.34545, 0.432376, -1.87183, 4.99102, -8.40221, 2.1211, 2.02617, 0.826243, -2.34776, 1.09457, -1.82723, 2.47247],
        [-1.81027, -2.21511, -1.25429, -11.9879, 10.8641, 0.492935, -2.69985, -3.20592, -1.54758, -2.39211, 2.20909, 0.352425],
        [1.14474, -0.398572, -1.83978, -4.96346, -7.02411, -2.26846, 1.71648, -0.803308, -2.30486, -1.10257, -1.48379, -2.63585],
        [-1.89796, 2.17887, -1.24025, 11.9599, 11.3704, -0.334737, -2.83063, 3.15655, -1.55017, 2.40058, 2.28072, -0.18408],
    ])
    test_lqr(A, B, Q, R, K)

    # Y6 data.
    mass = 2.07717
    moment_of_inertia = np.array([[0.0620513, 0.000933375, 0.00909391],
        [0.000933375, 0.101875, -0.000299824],
        [0.00909391, -0.000299824, 0.157657],
    ])
    motor_matrix = np.array([[-0, -0, -0, -0, -0, -0],
        [-0, -1.22465e-16, -0, -1.22465e-16, -0, -1.22465e-16],
        [-1, -1, -1, -1, -1, -1],
        [-0.275392, -0.275392, 0.284608, 0.284608, 0.00460803, 0.00460803],
        [0.182486, 0.182486, 0.182486, 0.182486, -0.377514, -0.377514],
        [-0.014664, 0.014664, -0.014664, 0.014664, 0.014664, -0.014664],
    ])
    x_eq = np.zeros(12)
    u_eq = np.array([3.51805, 3.51805, 3.35038, 3.35038, 3.32012, 3.32012])
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
    test_linearization(mass, moment_of_inertia, motor_matrix, x_eq, u_eq, A, B)
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
