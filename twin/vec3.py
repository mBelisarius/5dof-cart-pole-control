from __future__ import annotations

import numpy as np
import numpy.typing as npt


class Vec3:
    def __init__(self, *args):
        if len(args) == 1:
            arg = args[0]
            if isinstance(arg, Vec3):
                self.array = np.array(arg.array, dtype=float)
            elif isinstance(arg, (list, tuple, np.ndarray)):
                if len(arg) != 3:
                    raise ValueError("List, tuple, or array must have exactly 3 elements.")
                self.array = np.array(arg, dtype=float)
            else:
                raise TypeError("Single argument must be Vec3, list, tuple, or ndarray with 3 elements.")

        elif len(args) == 3:
            if all(isinstance(a, (int, float)) for a in args):
                self.array = np.array(args, dtype=float)
            else:
                raise TypeError("All three arguments must be int or float.")

        else:
            raise TypeError("Vec3 constructor accepts either a single 3D input or three float arguments.")

    @property
    def x(self) -> float:
        return float(self.array[0])

    @x.setter
    def x(self, value: float):
        self.array[0] = value

    @property
    def y(self) -> float:
        return float(self.array[1])

    @y.setter
    def y(self, value: float):
        self.array[1] = value

    @property
    def z(self) -> float:
        return float(self.array[2])

    @z.setter
    def z(self, value: float):
        self.array[2] = value

    def __iter__(self):
        return iter(self.array)

    def __add__(self, other: int | float | np.ndarray | Vec3) -> Vec3:
        if isinstance(other, (int, float)):
            return Vec3(self.x + other, self.y + other, self.z + other)

        elif isinstance(other, np.ndarray):
            return Vec3(self.x + other[0], self.y + other[1], self.z + other[2])

        elif isinstance(other, Vec3):
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

        else:
            raise TypeError("Vec3 addition accepts either a single 3D input or three float arguments.")

    def __radd__(self, other: int | float) -> Vec3:
        return self.__add__(other)

    def __sub__(self, other: int | float | np.ndarray | Vec3) -> Vec3:
        if isinstance(other, (int, float)):
            return Vec3(self.x - other, self.y - other, self.z - other)

        elif isinstance(other, np.ndarray):
            return Vec3(self.x - other[0], self.y - other[1], self.z - other[2])

        elif isinstance(other, Vec3):
            return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

        else:
            raise TypeError("Vec3 subtraction accepts either a single 3D input or three float arguments.")

    def __rsub__(self, other: int | float) -> Vec3:
        return self.__sub__(other)

    def __mul__(self, other: float | int | Vec3) -> Vec3:
        if isinstance(other, (int, float)):
            return Vec3(self.x * other, self.y * other, self.z * other)

        return Vec3(self.x * other.x, self.y * other.y, self.z * other.z)

    def __rmul__(self, other: float | int) -> Vec3:
        return self.__mul__(other)

    def __matmul__(self, other: Vec3 | npt.NDArray[float]) -> npt.NDArray[float]:
        if isinstance(other, Vec3):
            return self.array @ other.array.transpose
        elif isinstance(other, np.ndarray) and other.shape == (3,):
            return self.array @ other
        else:
            raise TypeError("Vec3 multiplication with ndarray must be with shape (3,).")

    def __rmatmul__(self, other: Vec3 | npt.NDArray[float]) -> npt.NDArray[float]:
        if isinstance(other, Vec3):
            return other.array @ self.array
        elif isinstance(other, np.ndarray) and other.shape == (3,):
            return other @ self.array
        else:
            raise TypeError("Vec3 multiplication with ndarray must be with shape (3,).")

    def __truediv__(self, other: float | int | Vec3) -> Vec3:
        if isinstance(other, (int, float)):
            return Vec3(self.x / other, self.y / other, self.z / other)

        return Vec3(self.x / other.x, self.y / other.y, self.z / other.z)

    def __neg__(self) -> Vec3:
        return Vec3(-self.x, -self.y, -self.z)

    def __str__(self) -> str:
        return f"Vec3({self.x}, {self.y}, {self.z})"

    def __repr__(self) -> str:
        return f"Vec3({self.x}, {self.y}, {self.z})"

    def __eq__(self, other: Vec3) -> bool:
        return np.allclose(self.array, other.array)

    @staticmethod
    def transpose(vec: Vec3) -> Vec3:
        return Vec3(vec.array.T)

    def _instance_transpose(self) -> Vec3:
        self.array = self.array.T
        return self

    def cross_matrix(self) -> npt.NDArray[float]:
        return np.array([
            [0, -self.z, self.y],
            [self.z, 0, -self.x],
            [-self.y, self.x, 0],
        ])

    @staticmethod
    def rotate(vec: Vec3, theta: float, phi: float) -> Vec3:
        ct = np.cos(theta)
        st = np.sin(theta)
        cp = np.cos(phi)
        sp = np.sin(phi)
        rot_mat_y = np.array([
            [ct, 0, -st],
            [0, 1, 0],
            [st, 0, ct],
        ])
        rot_mat_z = np.array([
            [cp, -sp, 0],
            [sp, cp, 0],
            [0, 0, 1],
        ])
        return Vec3(rot_mat_z @ (rot_mat_y @ vec.array))

    def _instance_rotate(self, theta: float, phi: float) -> Vec3:
        self.array = Vec3.rotate(self, theta, phi).array
        return self

    @staticmethod
    def rotate_inv(vec: Vec3, theta: float, phi: float) -> Vec3:
        ct = np.cos(-theta)
        st = np.sin(-theta)
        cp = np.cos(-phi)
        sp = np.sin(-phi)
        rot_mat_y = np.array([
            [ct, 0, -st],
            [0, 1, 0],
            [st, 0, ct],
        ])
        rot_mat_z = np.array([
            [cp, -sp, 0],
            [sp, cp, 0],
            [0, 0, 1],
        ])
        return Vec3(rot_mat_y @ (rot_mat_z @ vec.array))

    def _instance_rotate_inv(self, theta: float, phi: float) -> Vec3:
        self.array = Vec3.rotate_inv(self, theta, phi).array
        return self

    @staticmethod
    def rotate_cam(vec: Vec3, theta: float, phi: float) -> Vec3:
        ct = np.cos(theta)
        st = np.sin(theta)
        cp = np.cos(phi)
        sp = np.sin(phi)
        rot_mat_x = np.array([
            [1, 0, 0],
            [0, ct, -st],
            [0, st, ct],
        ])
        rot_mat_z = np.array([
            [cp, -sp, 0],
            [sp, cp, 0],
            [0, 0, 1],
        ])
        return Vec3(rot_mat_x @ (rot_mat_z @ vec.array))

    def _instance_rotate_cam(self, theta: float, phi: float) -> Vec3:
        self.array = Vec3.rotate_cam(self, theta, phi).array
        return self

    @staticmethod
    def saturate(vec: Vec3, lower: float, upper: float) -> Vec3:
        return Vec3(np.clip(vec.array, lower, upper))

    def _instance_saturate(self, lower: float, upper: float) -> Vec3:
        self.array = Vec3.saturate(self, lower, upper).array
        return self
