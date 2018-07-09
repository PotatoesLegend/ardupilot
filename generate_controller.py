import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import utility
import copter_dynamics

################################################################################
# Helper functions.
################################################################################
def read_csv(file_name):
    # We assume header is the first line.
    header = []
    pwm = []
    thrust = []
    torque = []
    current = []
    voltage = []
    pwm_idx, thrust_idx, torque_idx, current_idx, voltage_idx = -1, -1, -1, -1, -1
    thrust_scale = 1
    for line in open(file_name):
        if not header:
            header = [item.strip() for item in line.strip().split(',')]
            for idx, h in enumerate(header):
                if 'ESC signal' in h:
                    pwm_idx = idx
                elif 'Thrust' in h:
                    thrust_idx = idx
                    # Extract unit and convert it to Newton.
                    if '(gf)' in h:
                        thrust_scale = 0.001 * 9.8
                    elif '(kgf)' in h:
                        thrust_scale = 9.8
                    elif '(N)' in h:
                        thrust_scale = 1
                    else:
                        utility.print_error('Imperical system is evil. Please use metric system.')
                        sys.exit(-1)
                elif 'Torque' in h:
                    torque_idx = idx
                elif 'Current' in h:
                    current_idx = idx
                elif 'Voltage' in h:
                    voltage_idx = idx
            print('pwm col: %d, thrust col: %d, torque col: %d, current col: %d, voltage col: %d' % \
                (pwm_idx, thrust_idx, torque_idx, current_idx, voltage_idx))
        else:
            # Data region.
            data = [float(v.strip()) if v.strip() else 0 for v in line.split(',')]
            pwm.append(data[pwm_idx])
            thrust.append(data[thrust_idx] * thrust_scale)
            torque.append(data[torque_idx])
            current.append(data[current_idx])
            voltage.append(data[voltage_idx])
    # In the default setting of RCbenchmark S1580, thrust should be negative.
    if np.average(thrust) > 0:
        utility.print_warning('Thrust should be negative numbers unless you flipped the sign in RCbenchmark or it is intentional.')
    return pwm, thrust, torque, current, voltage

def plot_2d(x_data, y_data, x_label, y_label):
    fig, ax = plt.subplots()
    ax.plot(x_data, y_data, 'o')
    ax.set(xlabel=x_label, ylabel=y_label)
    plt.show()

# Returns x0 and s such that (x - x0) * s \in [0, 1].
def normalize_input(x):
    x0 = np.min(x)
    x1 = np.max(x)
    s = 1.0 / (x1 - x0)
    return x0, s

# Fit thrust = T(voltage, pwm) and return:
# T: a function that takes voltage (V) and pwm as input and returns thrust (N).
# x, n: parameters in T. See the function below.
def fit_thrust(voltage, pwm, thrust):
    # Step 1: normalize inputs to [0, 1].
    u0, us = normalize_input(voltage)
    p0, ps = normalize_input(pwm)
    t0, ts = normalize_input(thrust)
    u = (voltage - u0) * us
    p = (pwm - p0) * ps
    t = (thrust - t0) * ts
    # Now u, p, t \in [0, 1]. Let's fit:
    # t = [u^2, up, p^2, u, p, 1] * x.
    sample_num = len(thrust)
    A = np.hstack((u * u, u * p, p * p, u, p, np.ones((sample_num, 1))))
    # min \|Ax - t\|^2
    x, _, _, _ = np.linalg.lstsq(A, t)
    n = np.array([u0, us, p0, ps, t0, ts])
    def thrust_estimate(voltage, pwm):
        u0, us, p0, ps, t0, ts = n
        u = (voltage - u0) * us
        p = (pwm - p0) * ps
        if np.isscalar(voltage):
            a = np.array([u * u, u * p, p * p, u, p, 1])
        else:
            num = len(voltage)
            a = np.hstack((u * u, u * p, p * p, u, p, np.ones((num, 1))))
        t = np.dot(a, x)
        thrust = t / ts + t0
        return thrust
    return thrust_estimate, x, n

def acquire_frame_type(args):
    frame_type_str = args.frame
    if frame_type_str == 'QUAD_X':
        frame_type = copter_dynamics.CopterFrameType.QUAD_X
    elif frame_type_str == 'QUAD_PLUS':
        frame_type = copter_dynamics.CopterFrameType.QUAD_PLUS
    elif frame_type_str == 'PENTA':
        frame_type = copter_dynamics.CopterFrameType.PENTA
    else:
        utility.print_error('Unsupported frame type.')
        sys.exit(-1)
    return frame_type

def acquire_mass(args):
    mass = args.mass
    if mass > 500 and mass < 3000:
        print('Mass = %f g' % mass)
    else:
        utility.print_warning('Your copter seems too light or too heavy.')
    mass /= 1000.0
    return mass

def acquire_arm_length(args):
    # Acquire arm length.
    arm_length = args.arm_length
    if arm_length > 50 and arm_length < 400:
        print('Arm length = %f mm' % arm_length)
    else:
        utility.print_warning('Your arm length (%f mm) seems too short or too long.' % arm_length)
    arm_length /= 1000.0
    return arm_length

def acquire_moment_of_inertia(args):
    rotor_weight = args.rotor_weight
    if rotor_weight > 30 and rotor_weight < 200:
        print('Rotor weight = %f g' % rotor_weight)
    else:
        utility.print_warning('Your rotor (%f g) seems too light or too heavy.' % rotor_weight)
    rotor_weight /= 1000.0
    Ixx = args.ixx
    Iyy = args.iyy
    Izz = args.izz
    if Ixx != 0 and Iyy != 0 and Izz != 0:
        moi = np.diag(np.array([Ixx, Iyy, Izz]))
    else:
        print('Estimate the moment of inertia using rotors and arm length.')
        frame_type = acquire_frame_type(args)
        moi = copter_dynamics.compute_moment_of_inertia(frame_type, rotor_weight, arm_length)
    print('Moment of inertia = \n' + str(moi))
    return moi

def acquire_q_matrix(args):
    q_str = args.q
    q = [float(v) for v in q_str.strip().split()]
    if len(q) != 12 and len(q) != 1:
        utility.print_error('Expect to see 1 or 12 elements.')
        sys.exit(-1)
    if len(q) == 1:
        q = np.ones(12) * q[0]
    Q = np.diag(q)
    return Q

################################################################################
# Beginning of the script.
################################################################################
parser = argparse.ArgumentParser()
parser.add_argument('--dir', help='directory of motor measurement csv files', type=str, default='motor_test/t_motor_air2216')
parser.add_argument('--frame', help='frame type', type=str, default='PENTA')
parser.add_argument('--mass', help='copter mass in grams', type=float, default=1712.0)
parser.add_argument('--arm-length', help='distance from rotor to center in millimeters', type=float, default=230.0)
parser.add_argument('--rotor-weight', help='rotor mass in grams', type=float, default=93.0)
parser.add_argument('--ixx', help='x component of the moment of inertia (type 0 if you do not have them)', type=float, default=0.0)
parser.add_argument('--iyy', help='y component of the moment of inertia (type 0 if you do not have them)', type=float, default=0.0)
parser.add_argument('--izz', help='z component of the moment of inertia (type 0 if you do not have them)', type=float, default=0.0)
parser.add_argument('--q', help='diagonal elements in Q matrix for LQR (assume R = 1).', type=str, default='10')
args = parser.parse_args()
motor_dir = args.dir
frame_type = acquire_frame_type(args)
mass = acquire_mass(args)
arm_length = acquire_arm_length(args)
moi = acquire_moment_of_inertia(args)
Q = acquire_q_matrix(args)

# Read csv files.
csv_idx = 1
pwm = []            # 1000-2000
thrust = []         # Usually 0-2kg
torque = []         # Usually 0-0.2Nm
current = []        # Usually 0-20A
voltage = []        # Usually 0-18V

while True:
    csv_file_name = os.path.join(motor_dir, 'motor_test_%02d.csv' % csv_idx)
    if not os.path.exists(csv_file_name):
        if csv_idx == 1:
            utility.print_error('%s does not exist.' % csv_file_name)
            sys.exit(-1)
        else:
            break
    # Read csv file.
    print('Collecting data from %s' % csv_file_name)
    p, t, q, c, v = read_csv(csv_file_name)
    pwm += p
    thrust += t
    torque += q
    current += c
    voltage += v
    csv_idx += 1
sample_num = len(pwm)
pwm = np.abs(np.array(pwm).reshape((sample_num, 1))).astype(np.float64)
thrust = np.abs(np.array(thrust).reshape((sample_num, 1))).astype(np.float64)
torque = np.abs(np.array(torque).reshape((sample_num, 1))).astype(np.float64)
current = np.abs(np.array(current).reshape((sample_num, 1))).astype(np.float64)
voltage = np.abs(np.array(voltage).reshape((sample_num, 1))).astype(np.float64)

# Some statistics.
print('****************************** STATISTICS *************************************')
# Throttle.
min_pwm, max_pwm = np.min(pwm), np.max(pwm)
if min_pwm < 1000 or max_pwm > 2000:
    utility.print_error('Min pwm (%f) or max pwm (%f) exceeds the [1000, 2000] range. Your measurement is likely wrong.' % \
    (min_pwm, max_pwm))
    sys.exit(-1)
else:
    utility.print_success('Min pwm: %f. Max pwm: %f' % (min_pwm, max_pwm))
# Thrust.
max_thrust = np.max(thrust)
if max_thrust > 1.5 * 9.8:
    utility.print_warning('Max thrust: %f N. Your propeller seems way more powerful than needed.' % max_thrust)
else:
    utility.print_success('Max thrust: %f N' % max_thrust)
# Torque.
max_torque = np.max(torque)
if max_torque > 0.2:
    utility.print_warning('Max torque: %f Nm. Your propeller and motor seems way too powerful.' % max_torque)
else:
    utility.print_success('Max torque: %f Nm' % max_torque)
# Current.
max_current = np.max(current)
if max_current > 30:
    utility.print_error('Max current: %f A. This seems too large. Please stop immediately and check.' % max_current)
else:
    utility.print_success('Max current: %f A' % max_current)
# Voltage.
max_voltage, min_voltage = np.max(voltage), np.min(voltage)
voltage_diff = max_voltage - min_voltage
if max_voltage > 17:
    utility.print_error('Max voltage: %f V. This seems too large. Please stop immediately and check.' % max_voltage)
elif voltage_diff > 2:
    utility.print_error('Voltage difference: %f V. This seems too large. Please stop immediately and check your battery.' % (voltage_diff))
else:
    utility.print_success('Max voltage: %f V. Voltage difference: %f V' % (max_voltage, voltage_diff))

# Fit torque to thrust ratio.
print('****************************** TORQUE TO THRUST RATIO *************************************')
# lstsq(a, b) = \|b - ax\|^2 
plot_2d(thrust, torque, 'Thrust (N)', 'Torque (Nm)')
sol, res, _, _ = np.linalg.lstsq(thrust, torque)
torque_to_thrust_ratio = sol[0][0]
if res > 0.01 ** 2 * sample_num:
    utility.print_error('Thrust and torque are not linearly dependent. Your measurement is likely wrong.')
    sys.exit(-1)
utility.print_success('Torque to thrust ratio: %f. Residual: %f' % (torque_to_thrust_ratio, res))

# Fit the resistance of the LiPo battery.
print('****************************** LIPO BATTERY RESISTANCE *************************************')
# For each consecutive rows (i0, u0) and (i1, u1), we have u0 + i0 * R = u1 + i1 * R, or (i0 - i1)R - (u1 - u0) = 0.
delta_i = current[1:] - current[:-1]
delta_u = voltage[1:] - voltage[:-1]
# Filter too big or too small delta_i.
min_cur, max_cur = 0.25, 5
idx = (delta_i > min_cur) & (delta_i < max_cur)
delta_i = delta_i[idx]
delta_u = delta_u[idx]
plot_2d(delta_i, delta_u, 'Current (A)', 'Voltage (V)')
delta_i = delta_i.reshape((len(delta_i), 1))
delta_u = delta_u.reshape((len(delta_u), 1))
sol, res, _, _ = np.linalg.lstsq(delta_i, -delta_u)
R = sol[0][0]
if res > 0.05 ** 2 * sample_num:
    utility.print_error('Battery resistance is not linear. Please stop immediately and check your battery.')
    sys.exit(-1)
utility.print_success('Resistance: %f Ohm. Max voltage estimate diff: %f V' % (R, np.max(np.abs(delta_i * R + delta_u))))

# Fit thrust = T(u, pwm). Here u is the output voltage of the battery and pwm is the control signal sent to the rotor.
print('****************************** PWM TO THRUST *************************************')
thrust_estimate, func_param, norm_param = fit_thrust(voltage, pwm, thrust)
# Evaluate and visualize thrust_estimate. 
thrust_fit = thrust_estimate(voltage, pwm)
abs_thrust_error = np.max(np.abs(thrust_fit - thrust))
rel_thrust_error = np.max(np.abs(thrust_fit - thrust) / thrust)
if abs_thrust_error < 0.05 * 9.8 or rel_thrust_error < 0.05:
    utility.print_success('Max abs error: %f N. Max rel error: %f' % (abs_thrust_error, rel_thrust_error * 100) + ' %')
else:
    utility.print_error('Fitting error is too large! Max abs error: %f N. Max rel error: %f' % (abs_thrust_error, rel_thrust_error * 100) + ' %')
    sys.exit(-1)
# Visualization.
n = 50
voltage_sample = np.linspace(min_voltage, max_voltage, n)
pwm_sample = np.linspace(min_pwm, max_pwm, n)
X, Y = np.meshgrid(voltage_sample, pwm_sample)
Z = np.zeros((n, n))
for i in range(n):
    for j in range(n):
        Z[i,j] = thrust_estimate(X[i,j], Y[i,j])
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.hold(True)
ax.plot_surface(X, Y, Z)
# Show the measurements.
ax.scatter(voltage, pwm, thrust, c='r')
ax.set_xlabel('Voltage (V)')
ax.set_ylabel('PWM')
ax.set_zlabel('Thrust (N)')
plt.show()
# Print the parameters.
utility.print_success('Thrust fit parameters:\nfunc_param = ' + str(func_param) + '\nnorm_param = ' + str(norm_param))
print('To compute thrust: let u and p be voltage and pwm. Unpack norm_param = u0, us, p0, ps, t0, ts.')
print('Let u = (voltage - u0) * us, p = (pwm - p0) * ps')
print('Let a = [u^2, up, p^2, u, p, 1]')
print('The estimated thrust in Newton is a.dot(func_param) / ts + t0')

# Code generation.
print('****************************** CODE GENERATION *************************************')
# Generate motor matrix.
motor_matrix = copter_dynamics.compute_motor_matrix(frame_type, torque_to_thrust_ratio, arm_length)
# Compute the force equilibrium.
u_eq = copter_dynamics.compute_force_equilibrium(mass, motor_matrix)
# Linearize the model around x_eq and u_eq.
x_eq = np.zeros(12)
A, B = copter_dynamics.linearize_copter(mass, moi, motor_matrix, x_eq, u_eq) 
# LQR.
R = np.diag(np.ones(motor_matrix.shape[1]))
K = copter_dynamics.lqr(A, B, Q, R)

# Update AP_MotorsMatrix.cpp.
utility.print_success('Generating code...')
cpp_file = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), 'libraries/AP_Motors/AP_MotorsMatrix.cpp')
cpp_lines = []
start_idx = end_idx = include_idx = -1
for idx, line in enumerate(open(cpp_file)):
    cpp_lines.append(line)
    if 'Beginning of LQR.' in line:
        start_idx = idx 
    elif 'End of LQR.' in line:
        end_idx = idx
    elif '#include' in line:
        include_idx = idx
if start_idx == -1 or end_idx == -1:
    start_idx = include_idx + 1
    end_idx = include_idx
new_cpp_lines = cpp_lines[:start_idx]
new_cpp_lines.append('// Beginning of LQR. This part is auto generated. DO NOT MANUALLY MODIFY IT.\n')
new_cpp_lines.append('// Tao Du\n')
new_cpp_lines.append('// taodu@csail.mit.edu\n')
new_cpp_lines.append('static const float u_eq[%d] = { %s };\n' % (len(u_eq), ', '.join(['%ff' % u for u in u_eq])))
new_cpp_lines.append('static const float x_eq[%d] = { %s };\n' % (len(x_eq), ', '.join(['%ff' % x for x in x_eq])))
new_cpp_lines.append('static const int kMotorNum = %d;\n' % K.shape[0])
new_cpp_lines.append('static const float K[kMotorNum][%d] = {\n' % K.shape[1])
for i in range(K.shape[0]):
    new_cpp_lines.append('    { %s },\n' % (', '.join(['%ff' % v for v in K[i, :]])))
new_cpp_lines.append('};\n\n')
# Thrust to pwm function.
new_cpp_lines.append('static int16_t thrust_to_pwm(const float thrust_in_newton, const float voltage)\n')
new_cpp_lines.append('{\n')
u0, us, p0, ps, t0, ts = norm_param
new_cpp_lines.append('    const float u0 = %ff, us = %ff, p0 = %ff, ps = %ff, t0 = %ff, ts = %ff;\n' % (u0, us, p0, ps, t0, ts))
new_cpp_lines.append('    const float u = (voltage - u0) * us;\n')
new_cpp_lines.append('    const float t = (thrust_in_newton - t0) * ts;\n')
new_cpp_lines.append('    // Now t = [u^2, up, p^2, u, p, 1].dot(func_param)\n')
fp = func_param.flatten()
new_cpp_lines.append('    const float s_u2 = %ff, s_up = %ff, s_p2 = %ff, s_u = %ff, s_p = %ff, s1 = %ff;\n' % (fp[0], fp[1], fp[2], fp[3], fp[4], fp[5]))
new_cpp_lines.append('    // Reorganize them into the form of ap^2 + bp + c = 0.\n')
new_cpp_lines.append('    const float a = s_p2, b = s_p + s_up * u, c = s_u2 * u * u + s_u * u + s1 - t;\n')
new_cpp_lines.append('    const float delta = b * b - 4 * a * c;\n')
new_cpp_lines.append('    const int16_t pwm_min = 1000;\n')
new_cpp_lines.append('    const int16_t pwm_max = 2000;\n')
new_cpp_lines.append('    if (delta < 0) return pwm_min;\n')
new_cpp_lines.append('    const float p = (-b + sqrtf(delta)) / (2.0 * a);\n')
new_cpp_lines.append('    int16_t pwm = static_cast<int16_t>(p / ps + p0);\n')
new_cpp_lines.append('    // clamp PWM.\n')
new_cpp_lines.append('    pwm = (pwm > pwm_min) ? pwm : pwm_min;\n')
new_cpp_lines.append('    pwm = (pwm < pwm_max) ? pwm : pwm_max;\n')
new_cpp_lines.append('    return pwm;\n')
new_cpp_lines.append('}\n')
new_cpp_lines.append('// End of LQR. This part is auto generated. DO NOT MANUALLY MODIFY IT.\n')
new_cpp_lines += cpp_lines[end_idx + 1:]
f = open(cpp_file, 'w')
for line in new_cpp_lines:
    f.write(line)
f.close()
utility.print_success('Code generation finished. libraries/AP_Motors/AP_MotorsMatrix.cpp is updated. Please rerun ./build_and_upload.sh.')
