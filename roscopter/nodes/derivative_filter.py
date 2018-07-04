import math
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class DerivativeFilter(object):
    def __init__(self, buffer_size):
        self.buffer_size = buffer_size
        # Members from FilterWithBuffer.
        self.samples = np.zeros(buffer_size)
        self.sample_idx = 0   # Points to the next available sample slot.
        # Members from DerivativeFilter.
        self.new_data = False
        self.last_slope = 0.0
        self.timestamps = np.zeros(buffer_size)

    def reset(self):
        self.samples[:] = 0.0
        self.sample_idx = 0
        self.new_data = False
        self.last_slope = 0.0
        self.timestamps[:] = 0.0

    def update(self, t, v):
        i = self.sample_idx
        i1 = (i - 1) % self.buffer_size
        # i1 points to the last sample.
        if self.timestamps[i1] == t:
            # Not a new sample.
            return
        self.timestamps[i] = t
        self.samples[i] = v
        # Increment the idx.
        self.sample_idx = (self.sample_idx + 1) % self.buffer_size
        self.new_data = True

    def slope(self):
        raise ValueError('slope is not implemented')

class CentralSmoothFilter(DerivativeFilter):
    def __init__(self, buffer_size):
        super(CentralSmoothFilter, self).__init__(buffer_size)

    def slope(self):
        if not self.new_data:
            return self.last_slope
        if self.timestamps[-1] == 0.0:
            # Haven't seen enough data.
            return 0.0
        def f(i):
            return self.samples[(self.sample_idx - 1 + i) % self.buffer_size]
        def x(i):
            return self.timestamps[(self.sample_idx - 1 + i) % self.buffer_size]
        if self.buffer_size == 5:
            result = 2 * 2 * (f(1) - f(-1)) / (x(1) - x(-1)) \
                + 4 * 1 * (f(2) - f(-2)) / (x(2) - x(-2))
            result /= 8
        elif self.buffer_size == 7:
            result = 2 * 5 * (f(1) - f(-1)) / (x(1) - x(-1)) \
                + 4 * 4 * (f(2) - f(-2)) / (x(2) - x(-2)) \
                + 6 * 1 * (f(3) - f(-3)) / (x(3) - x(-3))
            result /= 32
        elif self.buffer_size == 9:
            result = 2 * 14 * (f(1) - f(-1)) / (x(1) - x(-1)) \
                + 4 * 14 * (f(2) - f(-2)) / (x(2) - x(-2)) \
                + 6 * 6 * (f(3) - f(-3)) / (x(3) - x(-3)) \
                + 8 * 1 * (f(4) - f(-4)) / (x(4) - x(-4))
            result /= 128
        elif self.buffer_size == 11:
            result =  2 * 42 * (f(1) - f(-1)) / (x(1) - x(-1)) \
                + 4 * 48 * (f(2) - f(-2)) / (x(2) - x(-2)) \
                + 6 * 27 * (f(3) - f(-3)) / (x(3) - x(-3)) \
                + 8 * 8 * (f(4) - f(-4)) / (x(4) - x(-4)) \
                + 10 * 1 * (f(5) - f(-5)) / (x(5) - x(-5))
            result /= 512
        else:
            raise ValueError('Unsupported buffer size')
        # Numerical errors.
        if np.isnan(result) or np.isinf(result):
            result = 0
        self.new_data = False
        self.last_slope = result
        return result

class OneSideSmoothFilter(DerivativeFilter):
    def __init__(self, buffer_size):
        super(OneSideSmoothFilter, self).__init__(buffer_size)

    def slope(self):
        if not self.new_data:
            return self.last_slope
        if self.timestamps[-1] == 0.0:
            # Haven't seen enough data.
            return 0.0
        def f(i):
            return self.samples[(self.sample_idx - 1 + i) % self.buffer_size]
        def x(i):
            return self.timestamps[(self.sample_idx - 1 + i) % self.buffer_size]
        result = 0
        if self.buffer_size == 5:
            result = 2 * 2 * (f(-1) - f(-3)) / (x(-1) - x(-3)) \
                + 4 * 1 * (f(0) - f(-4)) / (x(0) - x(-4))
            result /= 8
        elif self.buffer_size == 7:
            result = 2 * 5 * (f(-2) - f(-4)) / (x(-2) - x(-4)) \
                + 4 * 4 * (f(-1) - f(-5)) / (x(-1) - x(-5)) \
                + 6 * 1 * (f(0) - f(-6)) / (x(0) - x(-6))
            result /= 32
        elif self.buffer_size == 9:
            result = 2 * 14 * (f(-3) - f(-5)) / (x(-3) - x(-5)) \
                + 4 * 14 * (f(-2) - f(-6)) / (x(-2) - x(-6)) \
                + 6 * 6 * (f(-1) - f(-7)) / (x(-1) - x(-7)) \
                + 8 * 1 * (f(0) - f(-8)) / (x(0) - x(-8))
            result /= 128
        elif self.buffer_size == 11:
            result =  2 * 42 * (f(-4) - f(-6)) / (x(-4) - x(-6)) \
                + 4 * 48 * (f(-3) - f(-7)) / (x(-3) - x(-7)) \
                + 6 * 27 * (f(-2) - f(-8)) / (x(-2) - x(-8)) \
                + 8 * 8 * (f(-1) - f(-9)) / (x(-1) - x(-9)) \
                + 10 * 1 * (f(0) - f(-10)) / (x(0) - x(-10))
            result /= 512
        else:
            raise ValueError('Unsupported buffer size')
        # Numerical errors.
        if np.isnan(result) or np.isinf(result):
            result = 0
        self.new_data = False
        self.last_slope = result
        return result

class PolynomialFilter(DerivativeFilter):
    def __init__(self, degree):
        # Intentionally request more samples to smooth noise.
        super(PolynomialFilter, self).__init__(degree + 2)
        self.degree = degree

    def slope(self):
        if not self.new_data:
            return self.last_slope
        if self.timestamps[-1] == 0.0:
            # Haven't seen enough data.
            return 0.0
        # Fit f(t) = [a0, a1, ...].dot([1, t, t^2, ...])
        T = np.zeros((self.buffer_size, self.degree + 1)) 
        t = np.array(self.timestamps)
        t -= np.min(t)
        for i in range(self.degree + 1):
            T[:,i] = np.power(t, i)
        # min \|Ta - self.samples\|
        sol, res, _, _ = np.linalg.lstsq(T, self.samples)
        a = sol.flatten()
        # Now estimate the derivative at timestamps[self.sample_idx - 1]
        dt = np.zeros(self.degree + 1)
        t0 = t[self.sample_idx - 1]
        for i in range(self.degree + 1):
            dt[i] = 0 if i == 0 else i * np.power(t0, i - 1)
        result = dt.dot(a)
        # Numerical errors.
        if np.isnan(result) or np.isinf(result):
            result = 0
        self.new_data = False
        self.last_slope = result
        return result

if __name__ == '__main__':
    # Test data.
    omega = np.pi
    n = 250
    t = np.linspace(0, 10, n)
    x = 2 * np.sin(omega * t)
    k0 = 2 * omega * np.cos(omega * t)
    c5 = CentralSmoothFilter(5)
    o5 = OneSideSmoothFilter(5)
    p2 = PolynomialFilter(2)
    ck5 = np.zeros(n)
    ok5 = np.zeros(n)
    pk2 = np.zeros(n)
    for i in range(n):
        ti, xi = t[i], x[i]
        c5.update(ti, xi)
        o5.update(ti, xi)
        p2.update(ti, xi)
        ck5[i] = c5.slope()
        ok5[i] = o5.slope()
        pk2[i] = p2.slope()
    # Plot the signal and derivatives.
    fig, ax = plt.subplots()
    ax.plot(t, x, 'o-', label = 'position')
    ax.plot(t, k0, '*-', label = 'velocity')
    ax.plot(t, ck5, label = 'ck5')
    ax.plot(t, ok5, label = 'ok5')
    ax.plot(t, pk2, label = 'pk2')
    plt.legend()
    ax.set(xlabel = 't', ylabel = 'x')
    plt.show()
