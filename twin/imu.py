import numpy as np

from copy import deepcopy
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from vec3 import Vec3


@dataclass
class ImuRawData:
    time: float
    xdd: Vec3
    gd: Vec3


@dataclass
class ImuData:
    time: float
    x: Vec3
    xd: Vec3
    xdd: Vec3
    g: Vec3
    gd: Vec3
    gdd: Vec3


class Imu:
    def __init__(self):
        self.sglobal = ImuData(
            0.0,
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
        )
        self.slocal = ImuData(
            0.0,
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
            Vec3(0, 0, 0),
        )

        # Orientation (quaternion) relative to global frame
        self.orientation = Rotation.identity()

    def __str__(self):
        return (
            f"position_global: {self.sglobal.x}, "
            f"velocity_global: {self.sglobal.xd}, "
            f"orientation_quat: {self.orientation.as_quat()}, "
            f"position_local: {self.slocal.x}, "
            f"velocity_local: {self.slocal.xd}, "
        )

    def copy(self):
        return deepcopy(self)

    def update(self, accel, gyro, time):
        self.slocal.time = time
        self.sglobal.time = time

        dt = time - self.sglobal.time
        dt = max(0.016, dt)  # TODO: Fix and remove
        if dt <= 0.0:
            return

        # Integrate acceleration in local frame as well
        self.slocal.xd += accel * dt
        self.slocal.x += self.slocal.xd * dt
        self.slocal.gdd = (gyro - self.slocal.gd) / dt
        self.slocal.g += gyro * dt

        # Update orientation (integrate angular velocity in local frame)
        delta_angle = gyro * dt
        delta_rotation = Rotation.from_rotvec(delta_angle.array)
        self.orientation = self.orientation * delta_rotation

        # Transform linear acceleration to global frame
        accel_global = self.orientation.apply(accel.array)
        gyro_global = self.orientation.apply(gyro.array)

        # Integrate acceleration to get velocity and position in global frame
        self.sglobal.xd += Vec3(accel_global * dt)
        self.sglobal.x += Vec3(self.sglobal.xd * dt)
        self.sglobal.gdd = (gyro_global - self.sglobal.gd) / dt
        self.sglobal.g += gyro_global * dt
