# Packge imports
import math
import numpy as np

from scipy.integrate import solve_ivp, ode

# Local imports
from model import solver as sv

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
            'fusion-origin': Imu(),
            'target': Imu(),
            'model': Imu(),
            'model-last': Imu(),
            'model-origin': Imu(),
            'model-origin-last': Imu(),
            'meas': Imu(),
            'meas-last': Imu(),
            'meas-origin': Imu(),
            'meas-origin-last': Imu(),
        }

        for imu in self._imus.values():
            imu.sglobal.x.z = self._hbc
            imu.slocal.x.z = self._hbc
            imu.sglobal.g.y = 0.01
            imu.slocal.g.y = 0.01

        self._solver = sv.SolverLcp(5)
        print(self._make_f0())

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

    def update_imu(self, raw_data: ImuRawData):
        self._imus['meas-last'] = self._imus['meas'].copy()
        self._imus['meas'].update(raw_data.xdd, raw_data.gd, raw_data.time)

    def _update_from_model(self, t, fq, fv):
        self._imus['model-origin-last'] = self._imus['model-origin'].copy()
        self._imus['model-origin'].sglobal.time = t

        self._imus['model-last'] = self._imus['model'].copy()
        self._imus['model'].sglobal.time = t

        x, x_dot, y, y_dot, z, z_dot, theta, theta_dot, phi, phi_dot = fq

        dt = self._imus['model'].sglobal.time - self._imus['model-last'].sglobal.time
        if dt <= 0.0:
            return

        # Update origin
        xo = self._solver.fn_Xo(t, *fq, *fv)
        xo_dot = self._solver.fn_Xo_dot(t, *fq, *fv)

        self._imus['model-origin'].sglobal.x = Vec3(xo)
        self._imus['model-origin'].sglobal.xd = Vec3(xo_dot)
        self._imus['model-origin'].sglobal.xdd = (self._imus['model-origin'].sglobal.xd - self._imus['model-origin-last'].sglobal.xd) / dt

        self._imus['model-origin'].sglobal.g = Vec3(0, theta, phi)
        self._imus['model-origin'].sglobal.gd = Vec3(0, theta_dot, phi_dot)
        self._imus['model-origin'].sglobal.gdd = (self._imus['model-origin'].sglobal.gd - self._imus['model-origin-last'].sglobal.gd) / dt

        self._imus['model-origin'].slocal.x = Vec3(0, 0, 0)
        self._imus['model-origin'].slocal.xd = Vec3(0, 0, 0)
        self._imus['model-origin'].slocal.xdd = Vec3(0, 0, 0)

        self._imus['model-origin'].slocal.g = Vec3(0, theta, phi)
        self._imus['model-origin'].slocal.gd = Vec3(0, theta_dot, phi_dot)
        self._imus['model-origin'].slocal.gdd = (self._imus['model-origin'].slocal.gd - self._imus['model-origin-last'].slocal.gd) / dt

        # Update center
        xc = self._solver.fn_Xc(t, *fq, *fv)
        xc_dot = self._solver.fn_Xc_dot(t, *fq, *fv)

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

    def _make_f0(self):
        x = self._imus['fusion-origin'].sglobal.x
        xd = self._imus['fusion-origin'].sglobal.xd
        g = self._imus['fusion-origin'].sglobal.g
        gd = self._imus['fusion-origin'].sglobal.gd

        return np.array([
            x.x,
            xd.x,
            x.y,
            xd.y,
            x.z,
            xd.z,
            g.y,
            gd.y,
            g.z,
            gd.z
        ])

    def update_model(self, dt, fv):
        sol_t, sol_f = self._solver.step(dt, self._make_f0(), fv)
        self._update_from_model(self._imus['fusion'].sglobal.time + sol_t, sol_f, fv)

    def update_state(self, source):
        if source == 'meas':
            self._imus['fusion'] = self._imus['meas'].copy()
            self._imus['fusion-origin'] = self._imus['meas-origin'].copy()
        elif source == 'model':
            self._imus['fusion'] = self._imus['model'].copy()
            self._imus['fusion-origin'] = self._imus['model-origin'].copy()
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
