import abc
import importlib.util
import math
import numpy as np

from qpsolvers import solve_qp
from scipy.integrate import solve_ivp


class Solver(abc.ABC):
    def fn_Xo(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [x],
            [y],
            [z],
        ]).flatten()

    def fn_Xo_dot(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [x_dot + 0.03625*(fv_omega_l + fv_omega_r)*math.cos(phi)],
            [y_dot + 0.03625*(fv_omega_l + fv_omega_r)*math.sin(phi)],
            [z_dot],
        ]).flatten()

    def fn_Xco(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [0.08*math.sin(theta)*math.cos(phi)],
            [0.08*math.sin(phi)*math.sin(theta)],
            [0.08*math.cos(theta)],
        ]).flatten()

    def fn_Xco_dot(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [-0.08*phi_dot*math.sin(phi)*math.sin(theta) + 0.08*theta_dot*math.cos(phi)*math.cos(theta)],
            [0.08*phi_dot*math.sin(theta)*math.cos(phi) + 0.08*theta_dot*math.sin(phi)*math.cos(theta)],
            [-0.08*theta_dot*math.sin(theta)],
        ]).flatten()

    def fn_Xc(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [x + 0.08*math.sin(theta)*math.cos(phi)],
            [y + 0.08*math.sin(phi)*math.sin(theta)],
            [z + 0.08*math.cos(theta)],
        ]).flatten()

    def fn_Xc_dot(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [-0.08*phi_dot*math.sin(phi)*math.sin(theta) + 0.08*theta_dot*math.cos(phi)*math.cos(theta) + x_dot + 0.03625*(fv_omega_l + fv_omega_r)*math.cos(phi)],
            [0.08*phi_dot*math.sin(theta)*math.cos(phi) + 0.08*theta_dot*math.sin(phi)*math.cos(theta) + y_dot + 0.03625*(fv_omega_l + fv_omega_r)*math.sin(phi)],
            [-0.08*theta_dot*math.sin(theta) + z_dot],
        ]).flatten()

    def fn_M(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [0.7, 0, 0, 0.056*math.cos(phi)*math.cos(theta), -0.056*math.sin(phi)*math.sin(theta)],
            [0, 0.7, 0, 0.056*math.sin(phi)*math.cos(theta), 0.056*math.sin(theta)*math.cos(phi)],
            [0, 0, 0.7, -0.056*math.sin(theta), 0],
            [0.056*math.cos(phi)*math.cos(theta), 0.056*math.sin(phi)*math.cos(theta), -0.056*math.sin(theta), 0.01148, 0],
            [-0.056*math.sin(phi)*math.sin(theta), 0.056*math.sin(theta)*math.cos(phi), 0, 0, 0.00896*math.sin(theta)**2 + 0.002]
        ])

    def fn_H(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [-0.056*phi_dot**2*math.sin(theta)*math.cos(phi) - 0.112*phi_dot*theta_dot*math.sin(phi)*math.cos(theta) - 1/4*phi_dot*(0.1015*fv_omega_l + 0.1015*fv_omega_r + 3.92e-6*math.sin(theta))*math.sin(phi) - 0.056*theta_dot**2*math.sin(theta)*math.cos(phi) + 9.8e-7*math.sqrt(2)*theta_dot*math.sin(theta + (1/4)*math.pi)*math.cos(phi) + 1.225e-5*x_dot + 4.440625e-7*(fv_omega_l + fv_omega_r)*math.cos(phi)],
            [-0.056*phi_dot**2*math.sin(phi)*math.sin(theta) + 0.112*phi_dot*theta_dot*math.cos(phi)*math.cos(theta) + (1/4)*phi_dot*(0.1015*fv_omega_l + 0.1015*fv_omega_r + 3.92e-6*math.sin(theta))*math.cos(phi) - 0.056*theta_dot**2*math.sin(phi)*math.sin(theta) + 9.8e-7*math.sqrt(2)*theta_dot*math.sin(phi)*math.sin(theta + (1/4)*math.pi) + 1.225e-5*y_dot + 4.440625e-7*(fv_omega_l + fv_omega_r)*math.sin(phi)],
            [-0.056*theta_dot**2*math.cos(theta) + 9.8e-7*math.sqrt(2)*theta_dot*math.cos(theta + (1/4)*math.pi) + 20.00001225*z_dot + 6.867],
            [3.5525e-8*fv_omega_l*math.cos(theta) + 0.0525625*fv_omega_l + 3.5525e-8*fv_omega_r*math.cos(theta) + 0.0525625*fv_omega_r - 0.00448*phi_dot**2*math.sin(2*theta) + 0.1051250784*theta_dot + 9.8e-7*x_dot*math.cos(phi)*math.cos(theta) + 9.8e-7*y_dot*math.sin(phi)*math.cos(theta) - 9.8e-7*z_dot*math.sin(theta) - 0.54936*math.sin(theta)],
            [0.00896*phi_dot*theta_dot*math.sin(2*theta) + 7.84e-8*phi_dot*math.sin(theta)**2 + (1/4)*x_dot*(0.1015*fv_omega_l + 0.1015*fv_omega_r - 3.92e-6*math.sin(theta))*math.sin(phi) - 1/4*y_dot*(0.1015*fv_omega_l + 0.1015*fv_omega_r - 3.92e-6*math.sin(theta))*math.cos(phi)],
        ]).flatten()

    def fn_U(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [(2.85714285714286*(0.00448*math.sin(theta)**2 + 0.002)*(0.21025*fv_omega_l + 0.21025*fv_omega_r - 0.00896*phi_dot**2*math.sin(2*theta) + 0.4205*theta_dot + 6.4*z_dot*math.sin(theta))*math.cos(phi)*math.cos(theta) + 0.178571428571429*(0.00448*math.sin(theta)**2 + 0.002)*(0.448*phi_dot**2*math.sin(theta)*math.cos(phi) + 0.896*phi_dot*theta_dot*math.sin(phi)*math.cos(theta) + 2*phi_dot*(0.1015*fv_omega_l + 0.1015*fv_omega_r + 3.92e-6*math.sin(theta))*math.sin(phi) + 0.448*theta_dot**2*math.sin(theta)*math.cos(phi) - 7.84e-6*math.sqrt(2)*theta_dot*math.sin(theta + (1/4)*math.pi)*math.cos(phi) - 9.8e-5*x_dot - 3.5525e-6*(fv_omega_l + fv_omega_r)*math.cos(phi)) + 0.014*(0.0116*fv_omega_l*phi_dot*math.sin(theta) - 0.145*fv_omega_l*x_dot*math.sin(phi) + 0.145*fv_omega_l*y_dot*math.cos(phi) + 0.0116*fv_omega_r*phi_dot*math.sin(theta) - 0.145*fv_omega_r*x_dot*math.sin(phi) + 0.145*fv_omega_r*y_dot*math.cos(phi) - 0.0256*phi_dot*theta_dot*math.sin(2*theta))*math.sin(phi)*math.sin(theta))/(0.00448*math.sin(theta)**2 + 0.002)],
            [(2.85714285714286*(0.00448*math.sin(theta)**2 + 0.002)*(0.21025*fv_omega_l + 0.21025*fv_omega_r - 0.00896*phi_dot**2*math.sin(2*theta) + 0.4205*theta_dot + 6.4*z_dot*math.sin(theta))*math.sin(phi)*math.cos(theta) - 0.178571428571429*(0.00448*math.sin(theta)**2 + 0.002)*(-0.448*phi_dot**2*math.sin(phi)*math.sin(theta) + 0.896*phi_dot*theta_dot*math.cos(phi)*math.cos(theta) + 2*phi_dot*(0.1015*fv_omega_l + 0.1015*fv_omega_r + 3.92e-6*math.sin(theta))*math.cos(phi) - 0.448*theta_dot**2*math.sin(phi)*math.sin(theta) + 7.84e-6*math.sqrt(2)*theta_dot*math.sin(phi)*math.sin(theta + (1/4)*math.pi) + 9.8e-5*y_dot + 3.5525e-6*(fv_omega_l + fv_omega_r)*math.sin(phi)) - 0.014*(0.0116*fv_omega_l*phi_dot*math.sin(theta) - 0.145*fv_omega_l*x_dot*math.sin(phi) + 0.145*fv_omega_l*y_dot*math.cos(phi) + 0.0116*fv_omega_r*phi_dot*math.sin(theta) - 0.145*fv_omega_r*x_dot*math.sin(phi) + 0.145*fv_omega_r*y_dot*math.cos(phi) - 0.0256*phi_dot*theta_dot*math.sin(2*theta))*math.sin(theta)*math.cos(phi))/(0.00448*math.sin(theta)**2 + 0.002)],
            [0.08*theta_dot**2*math.cos(theta) - 1.4e-6*math.sqrt(2)*theta_dot*math.cos(theta + (1/4)*math.pi) - 28.5714460714286*z_dot - 2.85714285714286*(0.21025*fv_omega_l + 0.21025*fv_omega_r - 0.00896*phi_dot**2*math.sin(2*theta) + 0.4205*theta_dot + 6.4*z_dot*math.sin(theta))*math.sin(theta) - 9.81],
            [-7.50892857142857*fv_omega_l - 7.50892857142857*fv_omega_r + 0.32*phi_dot**2*math.sin(2*theta) - 15.0178571428571*theta_dot - 228.571428571429*z_dot*math.sin(theta)],
            [(0.00812*fv_omega_l*phi_dot*math.sin(theta) - 0.1015*fv_omega_l*x_dot*math.sin(phi) + 0.1015*fv_omega_l*y_dot*math.cos(phi) + 0.00812*fv_omega_r*phi_dot*math.sin(theta) - 0.1015*fv_omega_r*x_dot*math.sin(phi) + 0.1015*fv_omega_r*y_dot*math.cos(phi) - 0.01792*phi_dot*theta_dot*math.sin(2*theta))/(0.01792*math.sin(theta)**2 + 0.008)],
        ]).flatten()

    def fn_M1d(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0.7, 0, 0, 0.056*math.cos(phi)*math.cos(theta), -0.056*math.sin(phi)*math.sin(theta)],
            [0, 0, 0, 0, 0, 0, 0.7, 0, 0.056*math.sin(phi)*math.cos(theta), 0.056*math.sin(theta)*math.cos(phi)],
            [0, 0, 0, 0, 0, 0, 0, 0.7, -0.056*math.sin(theta), 0],
            [0, 0, 0, 0, 0, 0.056*math.cos(phi)*math.cos(theta), 0.056*math.sin(phi)*math.cos(theta), -0.056*math.sin(theta), 0.01148, 0],
            [0, 0, 0, 0, 0, -0.056*math.sin(phi)*math.sin(theta), 0.056*math.sin(theta)*math.cos(phi), 0, 0, 0.00896*math.sin(theta)**2 + 0.002],
        ])

    def fn_H1d(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [-x_dot],
            [-y_dot],
            [-z_dot],
            [-theta_dot],
            [-phi_dot],
            [-0.025375*fv_omega_l*phi_dot*math.sin(phi) + 4.440625e-7*fv_omega_l*math.cos(phi) - 0.025375*fv_omega_r*phi_dot*math.sin(phi) + 4.440625e-7*fv_omega_r*math.cos(phi) - 0.056*phi_dot**2*math.sin(theta)*math.cos(phi) - 0.112*phi_dot*theta_dot*math.sin(phi)*math.cos(theta) - 9.8e-7*phi_dot*math.sin(phi)*math.sin(theta) - 0.056*theta_dot**2*math.sin(theta)*math.cos(phi) + 9.8e-7*math.sqrt(2)*theta_dot*math.sin(theta + (1/4)*math.pi)*math.cos(phi) + 1.225e-5*x_dot],
            [0.025375*fv_omega_l*phi_dot*math.cos(phi) + 4.440625e-7*fv_omega_l*math.sin(phi) + 0.025375*fv_omega_r*phi_dot*math.cos(phi) + 4.440625e-7*fv_omega_r*math.sin(phi) - 0.056*phi_dot**2*math.sin(phi)*math.sin(theta) + 0.112*phi_dot*theta_dot*math.cos(phi)*math.cos(theta) + 9.8e-7*phi_dot*math.sin(theta)*math.cos(phi) - 0.056*theta_dot**2*math.sin(phi)*math.sin(theta) + 9.8e-7*math.sqrt(2)*theta_dot*math.sin(phi)*math.sin(theta + (1/4)*math.pi) + 1.225e-5*y_dot],
            [-0.056*theta_dot**2*math.cos(theta) + 9.8e-7*math.sqrt(2)*theta_dot*math.cos(theta + (1/4)*math.pi) + 20.00001225*z_dot + 6.867],
            [3.5525e-8*fv_omega_l*math.cos(theta) + 0.0525625*fv_omega_l + 3.5525e-8*fv_omega_r*math.cos(theta) + 0.0525625*fv_omega_r - 0.00448*phi_dot**2*math.sin(2*theta) + 0.1051250784*theta_dot + 4.9e-7*x_dot*math.cos(phi - theta) + 4.9e-7*x_dot*math.cos(phi + theta) + 4.9e-7*y_dot*math.sin(phi - theta) + 4.9e-7*y_dot*math.sin(phi + theta) - 9.8e-7*z_dot*math.sin(theta) - 0.54936*math.sin(theta)],
            [0.025375*fv_omega_l*x_dot*math.sin(phi) - 0.025375*fv_omega_l*y_dot*math.cos(phi) + 0.025375*fv_omega_r*x_dot*math.sin(phi) - 0.025375*fv_omega_r*y_dot*math.cos(phi) + 0.00896*phi_dot*theta_dot*math.sin(2*theta) + 7.84e-8*phi_dot*math.sin(theta)**2 - 9.8e-7*x_dot*math.sin(phi)*math.sin(theta) + 9.8e-7*y_dot*math.sin(theta)*math.cos(phi)],
        ]).flatten()

    def fn_U1d(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [x_dot],
            [y_dot],
            [z_dot],
            [theta_dot],
            [phi_dot],
            [(2.85714285714286*(0.00448*math.sin(theta)**2 + 0.002)*(0.21025*fv_omega_l + 0.21025*fv_omega_r - 0.00896*phi_dot**2*math.sin(2*theta) + 0.4205*theta_dot + 6.4*z_dot*math.sin(theta))*math.cos(phi)*math.cos(theta) + 0.178571428571429*(0.00448*math.sin(theta)**2 + 0.002)*(0.203*fv_omega_l*phi_dot*math.sin(phi) - 3.5525e-6*fv_omega_l*math.cos(phi) + 0.203*fv_omega_r*phi_dot*math.sin(phi) - 3.5525e-6*fv_omega_r*math.cos(phi) + 0.448*phi_dot**2*math.sin(theta)*math.cos(phi) + 0.896*phi_dot*theta_dot*math.sin(phi)*math.cos(theta) + 7.84e-6*phi_dot*math.sin(phi)*math.sin(theta) + 0.448*theta_dot**2*math.sin(theta)*math.cos(phi) - 7.84e-6*math.sqrt(2)*theta_dot*math.sin(theta + (1/4)*math.pi)*math.cos(phi) - 9.8e-5*x_dot) + 0.014*(0.0116*fv_omega_l*phi_dot*math.sin(theta) - 0.145*fv_omega_l*x_dot*math.sin(phi) + 0.145*fv_omega_l*y_dot*math.cos(phi) + 0.0116*fv_omega_r*phi_dot*math.sin(theta) - 0.145*fv_omega_r*x_dot*math.sin(phi) + 0.145*fv_omega_r*y_dot*math.cos(phi) - 0.0256*phi_dot*theta_dot*math.sin(2*theta))*math.sin(phi)*math.sin(theta))/(0.00448*math.sin(theta)**2 + 0.002)],
            [(2.85714285714286*(0.00448*math.sin(theta)**2 + 0.002)*(0.21025*fv_omega_l + 0.21025*fv_omega_r - 0.00896*phi_dot**2*math.sin(2*theta) + 0.4205*theta_dot + 6.4*z_dot*math.sin(theta))*math.sin(phi)*math.cos(theta) - 0.178571428571429*(0.00448*math.sin(theta)**2 + 0.002)*(0.203*fv_omega_l*phi_dot*math.cos(phi) + 3.5525e-6*fv_omega_l*math.sin(phi) + 0.203*fv_omega_r*phi_dot*math.cos(phi) + 3.5525e-6*fv_omega_r*math.sin(phi) - 0.448*phi_dot**2*math.sin(phi)*math.sin(theta) + 0.896*phi_dot*theta_dot*math.cos(phi)*math.cos(theta) + 7.84e-6*phi_dot*math.sin(theta)*math.cos(phi) - 0.448*theta_dot**2*math.sin(phi)*math.sin(theta) + 7.84e-6*math.sqrt(2)*theta_dot*math.sin(phi)*math.sin(theta + (1/4)*math.pi) + 9.8e-5*y_dot) - 0.014*(0.0116*fv_omega_l*phi_dot*math.sin(theta) - 0.145*fv_omega_l*x_dot*math.sin(phi) + 0.145*fv_omega_l*y_dot*math.cos(phi) + 0.0116*fv_omega_r*phi_dot*math.sin(theta) - 0.145*fv_omega_r*x_dot*math.sin(phi) + 0.145*fv_omega_r*y_dot*math.cos(phi) - 0.0256*phi_dot*theta_dot*math.sin(2*theta))*math.sin(theta)*math.cos(phi))/(0.00448*math.sin(theta)**2 + 0.002)],
            [0.08*theta_dot**2*math.cos(theta) - 1.4e-6*math.sqrt(2)*theta_dot*math.cos(theta + (1/4)*math.pi) - 28.5714460714286*z_dot - 2.85714285714286*(0.21025*fv_omega_l + 0.21025*fv_omega_r - 0.00896*phi_dot**2*math.sin(2*theta) + 0.4205*theta_dot + 6.4*z_dot*math.sin(theta))*math.sin(theta) - 9.81],
            [-7.50892857142857*fv_omega_l - 7.50892857142857*fv_omega_r + 0.32*phi_dot**2*math.sin(2*theta) - 15.0178571428571*theta_dot - 228.571428571429*z_dot*math.sin(theta)],
            [(0.00812*fv_omega_l*phi_dot*math.sin(theta) - 0.1015*fv_omega_l*x_dot*math.sin(phi) + 0.1015*fv_omega_l*y_dot*math.cos(phi) + 0.00812*fv_omega_r*phi_dot*math.sin(theta) - 0.1015*fv_omega_r*x_dot*math.sin(phi) + 0.1015*fv_omega_r*y_dot*math.cos(phi) - 0.01792*phi_dot*theta_dot*math.sin(2*theta))/(0.01792*math.sin(theta)**2 + 0.008)],
        ]).flatten()

    def fn_Cons(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [z],
            [z + 0.25*math.cos(theta) + 0.0725],
        ]).flatten()

    def fn_Cons_gradq(self, t, *f):
        fq = f[0:2 * self.dof]
        x, y, z, theta, phi = fq[0::2]
        x_dot, y_dot, z_dot, theta_dot, phi_dot = fq[1::2]

        fv = f[2 * self.dof:]
        fv_omega_l, fv_omega_r = fv

        return np.array([
            [0, 0, 1, 0, 0],
            [0, 0, 1, -0.25*math.sin(theta), 0],
        ])

    @property
    @abc.abstractmethod
    def dof(self):
        return NotImplementedError()

    @abc.abstractmethod
    def step(self, t, fq, fv):
        return NotImplementedError()

    @abc.abstractmethod
    def solve(self, t_span, fq0, fv, dt):
        return NotImplementedError()


class SolverOde(Solver):
    def __init__(self, dof):
        self._dof = dof

    @property
    def dof(self):
        return self._dof

    def dynamics_ode(self, t, fq, fv):
        fdd = np.zeros(2 * self.dof)
        fdd[0::2] = fq[1::2]
        fdd[1::2] = self.fn_U(t, *fq, *fv)
        return fdd

    def dynamics_ode_residual(self, t, fq, fqd, res, fv):
        M = self.fn_M1d(t, *fq, *fv)
        H = self.fn_H1d(t, *fq, *fv)

        res[:self.dof] = fqd[0::2] - fq[1::2]
        res[self.dof:] = M[self.dof:, self.dof:].dot(fqd[1::2]) + H[self.dof:]
        return res

    def step(self, t, fq, fv):
        return self.solve((0.0, t), fq, fv)

    def solve(self, t_span, f0, fv, dt=None):
        # TODO: Handle better the Sundials package
        spec = importlib.util.find_spec('sksundae')
        if spec is None or True:
            sol = solve_ivp(
                self.dynamics_ode,
                args=(fv,),
                t_span=t_span, y0=f0,
                method='Radau',
                rtol=1.0e-8, atol=1.0e-9, max_step=1.0e-2,
            )
            sol_t, sol_y = sol.t, sol.y
        else:
            from sksundae.cvode import CVODE
            from sksundae.ida import IDA

            fq0 = f0
            fqd0 = np.zeros_like(f0)
            fqd0[0::2] = f0[1::2]

            solver = IDA(
                self.dynamics_ode_residual,
                algebraic_idx=[],
                first_step=1.0e-8,
                max_step=1.0e-2,
                calc_init_dt=1.0e-5,
                calc_initcond='yp0',
                max_nonlin_iters=10,
                rtol=1.0e-9, atol=1.0e-8,
            )

            sol = solver.solve(t_span, fq0, fqd0)
            sol_t, sol_y = sol.t, sol.y.T

        return sol_t, sol_y


class SolverLcp(Solver):
    def __init__(self, dof):
        self._dof = dof

    @staticmethod
    def solve_lcp(A: np.ndarray, b: np.ndarray, reg: float = 1e-8) -> np.ndarray:
        """
        Solve the LCP:
          w = A @ λ + b,    w >= 0,    λ >= 0,    wᵀ λ = 0
        by recasting as the QP:
          min_{λ >= 0} ½ λᵀ A λ + bᵀ λ

        A small diagonal regularization term (reg*I) guarantees
        that the matrix passed to quadprog is positive definite.

        Parameters
        ----------
        A : (n,n) ndarray
            Should be symmetric positive semidefinite (e.g. J M^{-1} Jᵀ).
        b : (n,) ndarray
        reg : float
            Diagonal regularization to add to A for numerical stability.

        Returns
        -------
        λ : (n,) ndarray
            The contact forces satisfying the complementarity conditions.
        """
        n = b.shape[0]

        # Symmetrize and regularize to ensure PD
        P = 0.5 * (A + A.T) + reg * np.eye(n)
        q = b.copy()

        # λ >= 0  →  G λ <= h  with  G = -I,  h = 0
        G = -np.eye(n)
        h = np.zeros(n)

        # Solve the QP
        lam = solve_qp(P, q, G=G, h=h, lb=np.zeros(n), solver='quadprog')

        # Clamp any tiny negatives
        lam = np.maximum(lam, 0.0)
        return lam

    @property
    def dof(self):
        return self._dof

    def dynamics_constrained(self, t, fq, fv):
        """
        Moreau-Jean fixed-step integrator for unilateral constraints.

        Equations:
          M(q) v_dot = -H(q,v) + J^T λ,
          q_dot = v,
        with 0 ≤ λ ⟂ [J v_plus] ≥ 0.

        At each step:
          v_minus = v^n + dt * M^{-1}(-H)
          A = dt * J M^{-1} J^T
          b = J v_minus
          solve LCP: A λ + b ≥ 0, λ ≥ 0, (Aλ + b)^T λ = 0
          v_plus = v_minus - dt * M^{-1} J^T λ
          q^{n+1} = q^n + dt * v_plus
        """
        qn = fq[0::2]
        vn = fq[1::2]
        dt = t

        M = self.fn_M(t, *fq, *fv)
        H = self.fn_H(t, *fq, *fv)

        # Free velocity update
        a_minus = np.linalg.solve(M, -H)
        v_minus = vn + dt * a_minus

        # Constraints
        C = self.fn_Cons(t, *fq, *fv)
        C_jac = self.fn_Cons_gradq(t, *fq, *fv)

        # Predicted constraints after free motion: g + dt * J v_minus
        C_pred = C + dt * (C_jac.dot(v_minus))
        C_act = C_pred < 0.0

        # Build LCP matrices (no dt scaling for stability)
        # Optimize performance by solving the LCP only when at least one restriction is set
        v_plus = v_minus  # vn + dt * ddq
        lam = np.zeros(C_jac.shape[0])
        if np.any(C_act):
            # Restrict to active set
            C_jac_act = C_jac[C_act]
            M_inv_Jt = np.linalg.solve(M, C_jac.T)
            MinvJt_act = M_inv_Jt[:, C_act]
            A_act = C_jac_act @ MinvJt_act
            b_act = C_jac_act.dot(v_minus)

            # Solve reduced LCP
            lam[C_act] = SolverLcp.solve_lcp(A_act, b_act)
            v_plus += M_inv_Jt @ lam

        # Update states
        qn_next = qn + 0.5 * dt * (v_plus + v_plus + v_plus - vn)

        fq_next = np.zeros_like(fq)
        fq_next[0::2] = qn_next
        fq_next[1::2] = v_plus
        return fq_next

    def step(self, dt, f0, fv):
        sol_t = dt
        sol_y = self.dynamics_constrained(dt, f0, fv)
        return sol_t, sol_y.T

    def solve(self, t_span, f0, fv, dt):
        n_steps = int((t_span[1] - t_span[0]) / dt)
        sol_t = np.linspace(t_span[0], t_span[1], n_steps + 1)

        sol_y = np.zeros((n_steps + 1, 2 * self.dof))
        sol_y[0] = f0

        for iter in range(1, n_steps + 1):
            sol_y[iter] = self.dynamics_constrained(dt, sol_y[iter - 1], fv)

        return sol_t, sol_y.T
