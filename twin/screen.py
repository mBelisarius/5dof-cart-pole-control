from __future__ import annotations

import abc
import numpy as np
import numpy.typing as npt
import pygame
import queue

from vec3 import Vec3


class Screen:
    def __init__(self, width, height):
        self._width = width
        self._height = height

        self._pg_screen = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.SysFont(None, 28)

        self._obj_queue = queue.PriorityQueue()
        self._clock = pygame.time.Clock()
        self._running = True

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    def fill(self, color=(0, 0, 0)):
        self._pg_screen.fill(color)

    def _draw_objects(self, camera):
        while not self._obj_queue.empty():
            _, obj = self._obj_queue.get()
            proj = [camera.project(v) for v in obj.vertices]

            if isinstance(obj, LineSO):
                pygame.draw.line(self._pg_screen, obj.color, proj[0], proj[1], obj.width)
            elif isinstance(obj, TriangleSO):
                pygame.draw.polygon(self._pg_screen, obj.color, proj)
            else:
                raise RuntimeError(f"Unknown object type: {type(obj)}")

    def draw_text(self, text, p, color=(0, 0, 0)):
        text_surface = self.font.render(text, True, color)
        self._pg_screen.blit(text_surface, p)

    def draw_line(self, camera, line: LineSO):
        ScreenObject.bind_camera(camera)
        key = -float('inf')
        self._obj_queue.put((key, line))

    def draw_object(self, camera, obj: ScreenObject):
        ScreenObject.bind_camera(camera)
        for face in obj.faces:
            key = -face.cam_depth(camera)
            self._obj_queue.put((key, face))

    def render_frame(self, camera, bg_color=(240, 240, 240)):
        self.fill(bg_color)
        self._draw_objects(camera)


class Camera:
    def __init__(self, screen, fov=500, viewer_distance=5.0):
        self._screen = screen
        self.distance = viewer_distance
        self.fov = fov

        self.angle_yaw = 0  # rotation around Z axis (orbit)
        self.angle_pitch = 0  # rotation around local X axis
        self.target = Vec3(0, 0, 0)

    @property
    def pitch_adj(self):
        return self.angle_pitch + np.pi / 2.0

    @property
    def yaw_adj(self):
        return -self.angle_yaw - np.pi / 2.0

    @property
    def pos(self):
        return self.target + Vec3.rotate(Vec3(0, 0, self.distance), self.angle_pitch, self.angle_yaw)

    def project(self, point: Vec3):
        # Project only if the point is in front of the camera
        p = point - self.target
        p_rot = Vec3.rotate_cam(p, self.pitch_adj, self.yaw_adj)

        # Discard points behind the camera
        safe_factor = 1.0 - 1.0e-4
        if p_rot.z < -self.distance * safe_factor:
            p_rot.z = -self.distance * safe_factor

        # Perspective projection
        fov = self.fov / (self.distance + p_rot.z + 1.0e-7)
        px = int(self._screen.width / 2 + p_rot.x * fov)
        py = int(self._screen.height / 2 + p_rot.y * fov)
        return np.array([px, py])

    def invproject_xy(self, px, py):
        # Inverse perspective projection
        factor = self.fov / (self.distance + 1e-7)

        # Back project 2D point to 3D space (in camera coordinates)
        x = (px - self._screen.width / 2) / factor
        y = (self._screen.height / 2 - py) / factor
        z = 0  # Projecting to the ZY plane (z=0)

        # Rotate around Y-axis (inverse of yaw)
        cos_y = np.cos(-self.angle_yaw)
        sin_y = np.sin(-self.angle_yaw)
        x_rot = x * cos_y - z * sin_y
        z_rot = x * sin_y + z * cos_y

        # Rotate around X-axis (inverse of pitch)
        cos_x = np.cos(-self.angle_pitch)
        sin_x = np.sin(-self.angle_pitch)
        y_rot = y * cos_x - z_rot * sin_x
        z_rot = y * sin_x + z_rot * cos_x

        return Vec3(x_rot, y_rot, z_rot) + self.target


class ScreenObject(abc.ABC):
    _current_camera: Camera | None = None

    @classmethod
    def bind_camera(cls, cam: Camera):
        cls._current_camera = cam

    def __lt__(self, other: ScreenObject) -> bool:
        cam = ScreenObject._current_camera
        if cam is None:
            raise RuntimeError("No camera bound for ScreenObject comparison")

        # Use -depth => closer to the camera
        return self.cam_depth(cam) > other.cam_depth(cam)

    @property
    @abc.abstractmethod
    def color(self) -> tuple[int, int, int]:
        return NotImplementedError()

    @property
    @abc.abstractmethod
    def center(self) -> Vec3:
        return NotImplementedError()

    @property
    @abc.abstractmethod
    def vertices(self) -> npt.NDArray[Vec3]:
        return NotImplementedError()

    @property
    @abc.abstractmethod
    def faces(self) -> npt.NDArray[TriangleSO]:
        return NotImplementedError()

    @property
    @abc.abstractmethod
    def cam_visible(self) -> bool:
        return NotImplementedError()

    def cam_depth(self, camera):
        # project each vertex into camera space (before perspective)
        zs = np.zeros(len(self.vertices))
        for i, v in enumerate(self.vertices):
            p = v - camera.target
            p_rot = Vec3.rotate_cam(p, camera.pitch_adj, camera.yaw_adj)
            zs[i] = p_rot.z
        return np.mean(zs)


class LineSO(ScreenObject):
    def __init__(self, p1: Vec3, p2: Vec3, color=(0, 0, 0), width=1):
        self._p1 = p1
        self._p2 = p2
        self._color = color
        self.width = width

    @property
    def color(self) -> tuple[int, int, int]:
        return self._color

    @property
    def center(self) -> Vec3:
        return (self._p1 + self._p2) / 2

    @property
    def vertices(self) -> npt.NDArray[Vec3]:
        return np.array([self._p1, self._p2])

    @property
    def faces(self):
        return self

    @property
    def cam_visible(self) -> bool:
        return True


class TriangleSO(ScreenObject):
    def __init__(self, vertices: npt.NDArray[Vec3], color=(0, 0, 0)):
        self._vertices = vertices
        self._color = color

    @property
    def color(self) -> tuple[int, int, int]:
        return self._color

    @property
    def center(self) -> Vec3:
        return Vec3(np.sum(self.vertices, axis=0) / 3.0)

    @property
    def vertices(self) -> npt.NDArray[Vec3]:
        return self._vertices

    @property
    def faces(self):
        return self

    @property
    def cam_visible(self) -> bool:
        return True


class BoxSO(ScreenObject):
    def __init__(self, center: Vec3, width: float, height: float, depth: float, theta: float = 0, phi: float = 0, color=(0, 0, 0)):
        self._center = center
        self._width = width
        self._height = height
        self._depth = depth
        self._theta = theta
        self._phi = phi
        self._color = color

        self._is_vertices_updated = False
        self._vertices_buffer = None
        self._is_faces_updated = False
        self._faces_buffer = None

    @property
    def color(self) -> tuple[int, int, int]:
        return self._color

    @property
    def center(self) -> Vec3:
        return self._center

    @center.setter
    def center(self, value: float):
        self._center = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def width(self) -> float:
        return self._width

    @width.setter
    def width(self, value: float):
        self._width = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def height(self) -> float:
        return self._height

    @height.setter
    def height(self, value: float):
        self._height = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def depth(self) -> float:
        return self._depth

    @depth.setter
    def depth(self, value: float):
        self._depth = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def theta(self) -> float:
        return self._theta

    @theta.setter
    def theta(self, value: float):
        self._theta = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def phi(self) -> float:
        return self._phi

    @phi.setter
    def phi(self, value: float):
        self._phi = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def vertices(self) -> npt.NDArray[Vec3]:
        if self._is_vertices_updated:
            return self._vertices_buffer

        center = self._center
        half_w = self._width / 2
        half_h = self._height / 2
        half_d = self._depth / 2

        vertices = np.array([
            Vec3(-half_d, -half_w, -half_h),
            Vec3(+half_d, -half_w, -half_h),
            Vec3(+half_d, +half_w, -half_h),
            Vec3(-half_d, +half_w, -half_h),
            Vec3(-half_d, -half_w, +half_h),
            Vec3(+half_d, -half_w, +half_h),
            Vec3(+half_d, +half_w, +half_h),
            Vec3(-half_d, +half_w, +half_h),
        ])

        self._vertices_buffer = np.array([Vec3.rotate(v, self.theta, self.phi) + center for v in vertices], dtype=Vec3)
        self._is_vertices_updated = True

        return self._vertices_buffer

    @property
    def faces(self):
        if self._is_faces_updated:
            return self._faces_buffer

        self._faces_buffer = np.array([
            TriangleSO(self.vertices[[0, 1, 2]], self.color),
            TriangleSO(self.vertices[[2, 3, 0]], self.color),

            TriangleSO(self.vertices[[0, 1, 4]], self.color),
            TriangleSO(self.vertices[[1, 4, 5]], self.color),

            TriangleSO(self.vertices[[1, 2, 5]], self.color),
            TriangleSO(self.vertices[[2, 5, 6]], self.color),

            TriangleSO(self.vertices[[2, 3, 6]], self.color),
            TriangleSO(self.vertices[[3, 6, 7]], self.color),

            TriangleSO(self.vertices[[3, 0, 7]], self.color),
            TriangleSO(self.vertices[[0, 7, 4]], self.color),

            TriangleSO(self.vertices[[4, 5, 6]], self.color),
            TriangleSO(self.vertices[[6, 7, 4]], self.color),
        ], dtype=ScreenObject)

        self._is_faces_updated = True

        return self._faces_buffer

    @property
    def cam_visible(self) -> bool:
        return True


class DiskSO(ScreenObject):
    def __init__(self, center: Vec3, radius: float, thickness: float, theta: float, phi: float, color=(0, 0, 0), steps: int = 1000):
        self._center = center
        self._radius = radius
        self._thickness = thickness
        self._theta = theta
        self._phi = phi
        self._steps = steps
        self._normal = Vec3(1, 0, 0)
        self._color = color

        self._is_vertices_updated = False
        self._vertices_buffer = None

        self._is_faces_updated = False
        self._faces_buffer = None

    @property
    def color(self) -> tuple[int, int, int]:
        return self._color

    @property
    def center(self) -> Vec3:
        return self._center

    @center.setter
    def center(self, value: float):
        self._center = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def radius(self) -> float:
        return self._radius

    @radius.setter
    def radius(self, value: float):
        self._radius = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def thickness(self) -> float:
        return self._thickness

    @thickness.setter
    def thickness(self, value: float):
        self._thickness = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def theta(self) -> float:
        return self._theta

    @theta.setter
    def theta(self, value: float):
        self._theta = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def phi(self) -> float:
        return self._phi

    @phi.setter
    def phi(self, value: float):
        self._phi = value
        self._is_vertices_updated = False
        self._is_faces_updated = False

    @property
    def vertices(self) -> npt.NDArray[Vec3]:
        if self._is_vertices_updated:
            return self._vertices_buffer

        self._normal = Vec3.rotate(Vec3(1, 0, 0), self.theta, 0.0)
        vec_v = Vec3.rotate(Vec3(0, 1, 0), self.theta, 0.0)
        vec_w = Vec3.rotate(Vec3(0, 0, 1), self.theta, 0.0)

        half_th = self.thickness / 2
        vertices = np.empty(2 * self._steps, dtype=Vec3)
        for i in range(self._steps):
            ang_step = 2.0 * np.pi * i / self._steps
            rim_offset = vec_v * np.cos(ang_step) + vec_w * np.sin(ang_step)

            vec_w_a = vec_w.array
            rim_offset_a = rim_offset.array
            c, s = np.cos(self.phi), np.sin(self.phi)
            rim_spun = Vec3(vec_w_a * (vec_w_a.dot(rim_offset_a)) * (1 - c) + rim_offset_a * c + np.cross(vec_w_a, rim_offset_a) * s)

            vertices[2 * i + 0] = self.center + rim_spun * self.radius + self._normal * half_th
            vertices[2 * i + 1] = self.center + rim_spun * self.radius - self._normal * half_th

        self._vertices_buffer = vertices
        self._is_vertices_updated = True

        return self._vertices_buffer

    @property
    def faces(self):
        if self._is_faces_updated:
            return self._faces_buffer

        steps = self._steps
        half_th = self.thickness / 2

        self._normal = Vec3.rotate(Vec3(0, 0, 1), self.theta, 0.0)

        front_center = self.center + self._normal * half_th
        back_center = self.center - self._normal * half_th

        vertices = self.vertices
        faces = np.empty(4 * steps, dtype=TriangleSO)
        for i in range(steps):
            j = (i + 1) % steps
            rim_front_i = vertices[2 * i + 0]
            rim_back_i = vertices[2 * i + 1]
            rim_front_j = vertices[2 * j + 0]
            rim_back_j = vertices[2 * j + 1]

            # Front face (fan)
            faces[4 * i + 0] = TriangleSO(np.array([front_center, rim_front_i, rim_front_j]), self.color)

            # Back face (reverse winding)
            faces[4 * i + 1] = TriangleSO(np.array([back_center, rim_back_j, rim_back_i]), self.color)

            # Side faces (connect front and back rims)
            faces[4 * i + 2] = TriangleSO(np.array([rim_front_i, rim_back_i, rim_back_j]), self.color)
            faces[4 * i + 3] = TriangleSO(np.array([rim_front_i, rim_back_j, rim_front_j]), self.color)

        self._faces_buffer = faces
        self._is_faces_updated = True

        return self._faces_buffer

    @property
    def cam_visible(self) -> bool:
        return True
