import math
import numpy as np
from scipy.integrate import solve_ivp, ode

from imu import ImuRawData, ImuData, Imu
from screen import Camera, Screen, ScreenObject, BoxSO, DiskSO, LineSO
from vec3 import Vec3


class Cart:
    def __init__(self, hbc, hb, hr, eb, ew, dw):
        self._hbc = hbc
        self._hb = hb
        self._hr = hr
        self._eb = eb
        self._ew = ew
        self._dw = dw

        self._imus = {
            'fusion': Imu(),
            'target': Imu(),
            'model': Imu(),
            'model-last': Imu(),
            'meas': Imu(),
            'meas-last': Imu(),
        }

        for imu in self._imus.values():
            imu.sglobal.x.z = self._hbc
            imu.slocal.x.z = self._hbc
            imu.sglobal.g.y = 0.01
            imu.slocal.g.y = 0.01

        self._ode = ode(self._model).set_integrator('lsoda', rtol=1e-11, atol=1e-8, first_step=1.0e-8)
        f0 = self._make_f0()
        self._ode.set_initial_value(f0, 0.0)

    @property
    def origin(self):
        return self.center - Vec3.rotate(Vec3(0, 0, self._hbc), self.theta, self.phi)

    @property
    def center(self):
        return self._imus['fusion'].sglobal.x

    @property
    def theta(self):
        return self._imus['fusion'].slocal.g.y

    @property
    def phi(self):
        return self._imus['fusion'].slocal.g.z

    @property
    def imu(self):
        return self._imus['fusion']

    @property
    def imu_target(self):
        return self._imus['target']

    @property
    def _bottom(self):
        wheels_bottom = self.origin.z - self._dw / 2.0
        body_bottom = (self.origin + Vec3.rotate(Vec3(0, 0, self._hb), self.theta, self.phi)).z
        return min(wheels_bottom, body_bottom)

    @staticmethod
    def _state_xc(f):
        x, x_dot, y, y_dot, theta, theta_dot, phi, phi_dot = f

        return Vec3(
            x + 0.08 * math.sin(theta) * math.cos(phi),
            y + 0.08 * math.sin(phi) * math.sin(theta),
            0.08 * math.cos(theta)
        )

    @staticmethod
    def _state_xc_dot(f):
        x, x_dot, y, y_dot, theta, theta_dot, phi, phi_dot = f

        return Vec3(
            0.08 * theta_dot * math.cos(phi) * math.cos(theta) + x_dot,
            0.08 * theta_dot * math.sin(phi) * math.cos(theta) + y_dot,
            -0.08 * theta_dot * math.sin(theta)
        )

    def update_imu(self, raw_data: ImuRawData):
        self._imus['meas-last'] = self._imus['meas'].copy()
        self._imus['meas'].update(raw_data.xdd, raw_data.gd, raw_data.time)

    def _update_from_model(self, t, f):
        self._imus['model-last'] = self._imus['model'].copy()
        self._imus['model'].sglobal.time = t
        x, x_dot, y, y_dot, theta, theta_dot, phi, phi_dot = f

        xc = self._state_xc(f)
        xc_dot = self._state_xc_dot(f)

        dt = self._imus['model'].sglobal.time - self._imus['model-last'].sglobal.time
        dt = min(dt, 1.0e-8)

        self._imus['model'].sglobal.x = Vec3(xc)
        self._imus['model'].sglobal.xd = Vec3(xc_dot)
        self._imus['model'].sglobal.xdd = (self._imus['model'].sglobal.xd - self._imus['model-last'].sglobal.xd) / dt

        self._imus['model'].sglobal.g = Vec3(0, theta, phi)
        self._imus['model'].sglobal.gd = Vec3(0, theta_dot, phi_dot)
        self._imus['model'].sglobal.gdd = (self._imus['model'].sglobal.gd - self._imus['model-last'].sglobal.gd) / dt

        self._imus['model'].slocal.x = Vec3(0, 0, 0)
        self._imus['model'].slocal.xd = Vec3(0, 0, 0)
        self._imus['model'].slocal.xdd = Vec3(0, 0, 0)

        self._imus['model'].slocal.g = Vec3(0, theta, phi)
        self._imus['model'].slocal.gd = Vec3(0, theta_dot, phi_dot)
        self._imus['model'].slocal.gdd = (self._imus['model'].slocal.gd - self._imus['model-last'].slocal.gd) / dt

    def _model_M(self, t, f):
        x, x_dot, y, y_dot, theta, theta_dot, phi, phi_dot = f

        return np.array([
            [0.890249702734839, 0, 0.056*math.cos(phi)*math.cos(theta), 0],
            [0, 0.890249702734839, 0.056*math.sin(phi)*math.cos(theta), 0],
            [0.056*math.cos(phi)*math.cos(theta), 0.056*math.sin(phi)*math.cos(theta), 0.01048, 0],
            [0, 0, 0, 0.00448*math.sin(theta)**2 + 0.00321759809750297]
        ])

    def _model_H(self, t, f):
        x, x_dot, y, y_dot, theta, theta_dot, phi, phi_dot = f

        return np.array([
            [-0.056*phi_dot*theta_dot*math.sin(phi)*math.cos(theta) - 0.056*theta_dot**2*math.sin(theta)*math.cos(phi) + 9.8e-7*theta_dot*math.cos(phi)*math.cos(theta) + 3.80500630469679*x_dot],
            [0.056*phi_dot*theta_dot*math.cos(phi)*math.cos(theta) - 0.056*theta_dot**2*math.sin(phi)*math.sin(theta) + 9.8e-7*theta_dot*math.sin(phi)*math.cos(theta) + 3.80500630469679*y_dot],
            [-0.00224*phi_dot**2*math.sin(2*theta) - 0.056*phi_dot*x_dot*math.sin(phi)*math.cos(theta) + 0.056*phi_dot*y_dot*math.cos(phi)*math.cos(theta) + 0.0200000784*theta_dot + 9.8e-7*x_dot*math.cos(phi)*math.cos(theta) + 9.8e-7*y_dot*math.sin(phi)*math.cos(theta) - 0.54936*math.sin(theta)],
            [0.00448*phi_dot*theta_dot*math.sin(2*theta) + 0.0243519619500595*phi_dot + 0.056*theta_dot*x_dot*math.sin(phi)*math.cos(theta) - 0.056*theta_dot*y_dot*math.cos(phi)*math.cos(theta) - 9.8e-7*x_dot*math.sin(phi)*math.sin(theta) + 9.8e-7*y_dot*math.sin(theta)*math.cos(phi)]
        ])

    def _model(self, t, f):
        M_num = np.array(self._model_M(t, f), dtype=float)
        H_num = np.array(self._model_H(t, f), dtype=float).flatten()

        xdd = np.linalg.lstsq(M_num, -H_num, rcond=1.0e-8)[0]

        return np.array([
            f[1],
            xdd[0],
            f[3],
            xdd[1],
            f[5],
            xdd[2],
            f[7],
            xdd[3],
        ])

    def _make_f0(self):
        x = self._imus['fusion'].sglobal.x
        xd = self._imus['fusion'].sglobal.xd
        g = self._imus['fusion'].sglobal.g
        gd = self._imus['fusion'].sglobal.gd

        return [
            x.x,
            xd.x,
            x.y,
            xd.y,
            g.y,
            gd.y,
            g.z,
            gd.z
        ]

    def update_model(self, dt):
        self._ode.integrate(self._ode.t + dt)
        self._update_from_model(self._ode.t, self._ode.y)

    def update_state(self, source):
        if source == 'meas':
            self._imus['fusion'] = self._imus['meas'].copy()
        elif source == 'model':
            self._imus['fusion'] = self._imus['model'].copy()
        else:
            raise ValueError("Invalid source for cart state update.")

    def draw(self, screen, camera):
        # TODO: Fix theta rotation
        axle = BoxSO(
            center=self.origin,
            width=2 * self._eb,
            height=2 * self._hr,
            depth=2 * self._hr,
            theta=self.theta,
            phi=self.phi,
            color=(0, 0, 255)
        )
        body = BoxSO(
            center=self.center,
            width=2 * self._hr,
            height=self._hb - self._hr,
            depth=2 * self._hr,
            theta=self.theta,
            phi=self.phi,
            color=(0, 0, 255)
        )
        wheel_left = DiskSO(
            center=self.origin + Vec3.rotate(Vec3(0, self._eb, 0), self.theta, self.phi),
            radius=self._dw / 2.0,
            thickness=self._ew,
            theta=0,
            phi=self.phi + np.pi / 2,
            color=(0, 200, 0),
            steps=24,
        )
        wheel_right = DiskSO(
            center=self.origin + Vec3.rotate(Vec3(0, -self._eb, 0), self.theta, self.phi),
            radius=self._dw / 2.0,
            thickness=self._ew,
            theta=0,
            phi=self.phi + np.pi / 2,
            color=(0, 200, 0),
            steps=24,
        )

        screen.draw_object(camera, axle)
        screen.draw_object(camera, body)
        screen.draw_object(camera, wheel_left)
        screen.draw_object(camera, wheel_right)
