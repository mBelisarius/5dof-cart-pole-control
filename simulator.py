import sys
import math
import pygame
import numpy as np
from pygame.locals import *


class Camera:
    def __init__(self, screen, fov=500, viewer_distance=5.0):
        self.distance = viewer_distance
        self.fov = fov
        self.width = screen.get_width()
        self.height = screen.get_height()

        self.angle_yaw = 0  # rotation around Z axis (orbit)
        self.angle_pitch = 0  # rotation around local X axis
        self.center = np.array([1.0, 0, 0])

    def project(self, x, y, z):
        # Convert point to vector relative to camera center
        p = np.array([x, y, z]) - self.center

        # Rotation around Z axis (yaw)
        cz = math.cos(self.angle_yaw)
        sz = math.sin(self.angle_yaw)
        Rz = np.array([
            [cz, -sz, 0],
            [sz, cz, 0],
            [0, 0, 1]
        ])

        # Rotation around X axis (pitch) — in rotated frame!
        cx = math.cos(-(self.angle_pitch - math.pi / 2.0))
        sx = math.sin(-(self.angle_pitch - math.pi / 2.0))
        Rx = np.array([
            [1, 0, 0],
            [0, cx, -sx],
            [0, sx, cx]
        ])

        # Apply rotation: first yaw, then pitch
        p_rot = Rx @ (Rz @ p)

        # Perspective projection
        # x, z, y = p_rot
        x, y, z = p_rot
        # z = -z
        factor = self.fov / (self.distance + z + 1.0e-7)
        px = int(self.width / 2 + x * factor)
        py = int(self.height / 2 - y * factor)
        return np.array([px, py])

    def invproject_xy(self, px, py):
        # Inverse perspective projection
        factor = self.fov / (self.distance + 1e-7)

        # Back project 2D point to 3D space (in camera coordinates)
        x = (px - self.width / 2) / factor
        y = (self.height / 2 - py) / factor
        z = 0  # Projecting to the ZY plane (z=0)

        # Rotate around Y-axis (inverse of yaw)
        cos_y = math.cos(-self.angle_yaw)
        sin_y = math.sin(-self.angle_yaw)
        x_rot = x * cos_y - z * sin_y
        z_rot = x * sin_y + z * cos_y

        # Rotate around X-axis (inverse of pitch)
        cos_x = math.cos(-self.angle_pitch)
        sin_x = math.sin(-self.angle_pitch)
        y_rot = y * cos_x - z_rot * sin_x
        z_rot = y * sin_x + z_rot * cos_x

        # Adjust for camera center
        x_final = x_rot + self.center[0]
        y_final = y_rot + self.center[1]
        z_final = z_rot + self.center[2]

        return np.array([x_final, y_final, z_final])


def generate_trajectory(num_steps=200):
    traj = []
    for i in range(num_steps):
        t = i / num_steps * 2 * math.pi
        x = 1.5 * math.sin(t)
        y = 0.0
        z = 0.0
        eb = 1.0
        w1 = (x - eb / 2, -0.5, z)
        w2 = (x + eb / 2, -0.5, z)
        origin = (x, 0.5, z)
        traj.append({'origin': origin, 'wheel1': w1, 'wheel2': w2})
    return traj


def draw_axes(screen, camera, length=1.0, offset_x=0.9, offset_y=0.8):
    # Define the colors for the axes
    color_x = (255, 0, 0)  # Red for X axis
    color_y = (0, 255, 0)  # Green for Y axis
    color_z = (0, 0, 255)  # Blue for Z axis

    screen_width = screen.get_width()
    screen_height = screen.get_height()

    # Set the origin of the axes on the screen
    xo = screen_width * offset_x
    yo = screen_height * offset_y
    po = np.array([xo, yo])
    po_3d = camera.invproject_xy(xo, yo)

    # Draw the X axis (Red) - Project the end point in the X direction
    px = camera.project(length, 0.0, 0.0)
    pygame.draw.line(screen, color_x, po, px + po, 2)

    # Draw the Y axis (Green) - Project the end point in the Y direction
    py = camera.project(0.0, length, 0.0)
    pygame.draw.line(screen, color_y, po, py + po, 2)

    # Draw the Z axis (Blue) - Project the end point in the Z direction
    pz = camera.project(0.0, 0.0, length)
    pygame.draw.line(screen, color_z, po, pz + po, 2)


def draw_box(screen, camera, center, w, h, d, fill_color, edge_color=(0, 0, 0)):
    # Filled 3D box with edge outlines
    x, y, z = center

    # compute 8 corners
    corners = [
        (x - w / 2, y - h / 2, z - d / 2), (x + w / 2, y - h / 2, z - d / 2),
        (x + w / 2, y + h / 2, z - d / 2), (x - w / 2, y + h / 2, z - d / 2),
        (x - w / 2, y - h / 2, z + d / 2), (x + w / 2, y - h / 2, z + d / 2),
        (x + w / 2, y + h / 2, z + d / 2), (x - w / 2, y + h / 2, z + d / 2),
    ]

    # define faces by corner indices
    faces = [
        [0, 1, 2, 3],  # front
        [4, 5, 6, 7],  # back
        [0, 1, 5, 4],  # bottom
        [2, 3, 7, 6],  # top
        [1, 2, 6, 5],  # right
        [0, 3, 7, 4],  # left
    ]

    # project all corners
    proj = [camera.project(*c) for c in corners]

    # draw filled faces
    for face in faces:
        pts = [proj[i] for i in face]
        pygame.draw.polygon(screen, fill_color, pts)

    # draw edges on top
    # edges = [
    #     (0, 1), (1, 2), (2, 3), (3, 0),
    #     (4, 5), (5, 6), (6, 7), (7, 4),
    #     (0, 4), (1, 5), (2, 6), (3, 7)
    # ]
    # for e in edges:
    #     pygame.draw.line(screen, edge_color, proj[e[0]], proj[e[1]], 1)


def draw_disk(screen, camera, center, radius, thickness, fill_color, edge_color=(0, 0, 0), steps=1000):
    x0, y0, z0 = center
    half_th = thickness / 2.0

    # Precompute rim points in 3D
    rim_front_3d = np.zeros((steps, 3))
    rim_back_3d = np.zeros((steps, 3))

    for i in range(steps):
        θ = 2 * math.pi * i / steps
        x = x0 + radius * math.cos(θ)
        z = z0 + radius * math.sin(θ)
        rim_front_3d[i] = [x, y0 + half_th, z]
        rim_back_3d[i] = [x, y0 - half_th, z]

    # Project 3D points
    rim_front_2d = [camera.project(*p) for p in rim_front_3d]
    rim_back_2d = [camera.project(*p) for p in rim_back_3d]
    center_front_2d = camera.project(x0, y0 + half_th, z0)
    center_back_2d = camera.project(x0, y0 - half_th, z0)

    # Draw front face (fan)
    for i in range(steps):
        p1 = rim_front_2d[i]
        p2 = rim_front_2d[(i + 1) % steps]
        pygame.draw.polygon(screen, fill_color, [center_front_2d, p1, p2])

    # Draw back face (reverse winding)
    for i in range(steps):
        p1 = rim_back_2d[(i + 1) % steps]
        p2 = rim_back_2d[i]
        pygame.draw.polygon(screen, fill_color, [center_back_2d, p1, p2])

    # Draw side faces
    for i in range(steps):
        f1 = rim_front_2d[i]
        f2 = rim_front_2d[(i + 1) % steps]
        b1 = rim_back_2d[i]
        b2 = rim_back_2d[(i + 1) % steps]
        pygame.draw.polygon(screen, fill_color, [f1, b1, b2])
        pygame.draw.polygon(screen, fill_color, [f1, b2, f2])

    # Draw rims
    pygame.draw.polygon(screen, edge_color, rim_front_2d, 1)
    pygame.draw.polygon(screen, edge_color, rim_back_2d, 1)


def draw_ground(screen, camera, size=5.0, step=1.0, color=(128, 128, 128)):
    # Draw ground as a wire‐frame grid, so it never blocks the 3D objects
    z = 0.0

    # draw lines parallel to X axis
    y = -size
    while y <= size:
        p1 = camera.project(-size, y, z)
        p2 = camera.project(size, y, z)
        pygame.draw.line(screen, color, p1, p2, 1)
        y += step

    # draw lines parallel to Y axis
    x = -size
    while x <= size:
        p1 = camera.project(x, -size, z)
        p2 = camera.project(x, size, z)
        pygame.draw.line(screen, color, p1, p2, 1)
        x += step


def draw_cart(screen, camera, center, hb, hr, eb, ew, dw):
    center = np.array(center)

    # Cart body
    draw_box(
        screen, camera,
        center,
        2.0 * hr, 2.0 * eb, 2.0 * hr,
        (0, 0, 255),
    )
    draw_box(
        screen, camera,
        center + np.array([0.0, 0.0, hr + 0.5 * (hb - hr)]),
        2.0 * hr, 2.0 * hr, hb - hr,
        (0, 0, 255),
    )

    # Wheels
    draw_disk(
        screen, camera,
        center + np.array([0.0, eb, 0.0]),
        radius=dw / 2.0,
        thickness=ew, steps=24,
        fill_color=(0, 200, 0),
        edge_color=(0, 100, 0)
    )
    draw_disk(
        screen, camera,
        center + np.array([0.0, -eb, 0.0]),
        radius=dw / 2.0,
        thickness=ew, steps=24,
        fill_color=(0, 200, 0),
        edge_color=(0, 100, 0)
    )


# Main loop
def main():
    hb = 25.0e-2
    hr = 1.0e-2
    eb = 15.0e-2 / 2.0
    ew = 8.0e-3
    dw = 15.0e-2 / 2.0

    pygame.init()
    pygame.font.init()
    font = pygame.font.SysFont(None, 28)
    screen = pygame.display.set_mode((1200, 900))
    clock = pygame.time.Clock()
    trajectory = generate_trajectory()
    idx = 0
    camera = Camera(screen)
    dragging = False
    last_mouse = (0, 0)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == MOUSEBUTTONDOWN:
                dragging = True
                last_mouse = event.pos
            elif event.type == MOUSEBUTTONUP:
                dragging = False
            elif event.type == MOUSEMOTION and dragging:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                camera.angle_yaw += dx * 0.01
                camera.angle_pitch += dy * 0.01
                camera.angle_pitch = max(0.0, min(math.pi / 2, camera.angle_pitch))
                last_mouse = event.pos
            elif event.type == MOUSEWHEEL:
                camera.distance *= 0.9 if event.y > 0 else 1.1
                camera.distance = max(0.1, min(100.0, camera.distance))

        screen.fill((240, 240, 240))

        draw_ground(screen, camera)
        draw_cart(screen, camera, [0.0, 0.0, dw / 2.0], hb=hb, hr=hr, eb=eb, ew=ew, dw=dw)

        draw_axes(screen, camera)

        text = f"Yaw: {math.degrees(camera.angle_yaw):.1f}°, Pitch: {math.degrees(camera.angle_pitch):.1f}°"
        label = font.render(text, True, (0, 0, 0))
        screen.blit(label, (10, 10))

        pygame.display.flip()
        clock.tick(60)
        idx += 1

    pygame.quit()
    sys.exit()


if __name__ == '__main__':
    main()
