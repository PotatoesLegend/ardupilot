import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Helper functions.
def skew_matrix(x):
    x0, x1, x2 = x
    return np.array([[0, -x2, x1],
        [x2, 0, -x0],
        [-x1, x0, 0]])

def plot_2d(x_data, y_data, x_label, y_label):
    fig, ax = plt.subplots()
    ax.plot(x_data, y_data, 'o')
    ax.set(xlabel=x_label, ylabel=y_label)
    plt.show()

# Quadcopter definition.
class Quadcopter(object):
    def __init__(self):
        self.mass = 1.5
        self.g = 9.81
        self.L = 0.25
        self.J = np.eye(3) * 0.01
        self.p = np.zeros(3) # Center of mass.
        self.R = np.eye(3)   # Rotation.
        self.v = np.zeros(3) # Velocity.
        self.w = np.zeros(3) # Angular velocity.

        M = np.zeros((6, 4))
        M[2,:] = -1.0
        L0 = self.L / np.sqrt(2.0)
        M[3,:] = np.array([-L0, -L0, L0, L0])
        M[4,:] = np.array([L0, -L0, -L0, L0])
        lbd = 0.1
        M[5,:] = np.array([lbd, -lbd, lbd, -lbd])
        self.M = M

    def step(self, thrusts, dt):
        # Compute net force.
        f = np.dot(self.M, thrusts)
        f_body = f[:3]
        tau_body = f[3:]

        # Update linear motion.
        z = np.array([0, 0, 1])
        f = self.mass * self.g * z + np.dot(self.R, f_body)
        a = f / self.mass
        self.v += a * dt
        self.p += self.v * dt
        
        # Update angular motion.
        rhs = tau_body - np.cross(self.w, np.dot(self.J, self.w))
        w_dot = np.linalg.solve(self.J, rhs)
        self.w += w_dot * dt
        R = self.R
        R += np.dot(skew_matrix(self.w), R) * dt
        U, _, V = np.linalg.svd(R)
        R = np.dot(U, V)
        self.R = R

    def position(self):
        return self.p

    def velocity(self):
        return self.v

    def attitude(self):
        return self.R

    def hover_thrust(self):
        return np.ones(4) * self.mass * self.g / 4.0

    def hover_thrust_noisy(self):
        return self.hover_thrust() * 0.6

quad = Quadcopter()
# Simulation.
n = 2000
dt = 0.01
t = np.zeros(n)
p = np.zeros((3, n))
h0 = 10.0
kp = 1.0
kd = 0.5
ki = 0.5
i_sum = 0.0
for i in range(n):
    loc = quad.position()
    vel = quad.velocity()
    h = -loc[2]
    dh = -vel[2]
    if i < n / 2:
        i_sum = 0.0
    else:
        i_sum += (h0 - h) * dt
    d_thrust = kp * (h0 - h) + kd * (-dh) + ki * i_sum
    thrusts = quad.hover_thrust_noisy() + d_thrust
    t[i] = i * dt
    quad.step(thrusts, dt)
    p[:,i] = quad.position()

plot_2d(t, -p[2,:], 'time', 'height')
