import numpy as np

from vec3 import Vec3
from screen import Camera, Screen, ScreenObject, BoxSO, DiskSO, LineSO

def draw_axes(screen, camera, length=1.0, offset_x=0.9, offset_y=0.8):
    # Define the colors for the axes
    color_x = (255, 0, 0)  # Red for X axis
    color_y = (0, 255, 0)  # Green for Y axis
    color_z = (0, 0, 255)  # Blue for Z axis

    screen_width = screen.width
    screen_height = screen.height

    # Set the origin of the axes on the screen
    po = Vec3(0, 0, 0)

    # Draw the X axis (Red) - Project the end point in the X direction
    px = Vec3(length, 0.0, 0.0)
    screen.draw_line(camera, LineSO(po, px - po, color_x))

    # Draw the Y axis (Green) - Project the end point in the Y direction
    py = Vec3(0.0, length, 0.0)
    screen.draw_line(camera, LineSO(po, py - po, color_y))

    # Draw the Z axis (Blue) - Project the end point in the Z direction
    pz = Vec3(0.0, 0.0, length)
    screen.draw_line(camera, LineSO(po, pz - po, color_z))


def draw_ground(screen, camera, z=0.0, size=5.0, step=0.25, color=(128, 128, 128)):
    # draw lines parallel to X axis
    y = -size
    while y <= size:
        p1 = Vec3(-size, y, z)
        p2 = Vec3(size, y, z)
        screen.draw_line(camera, LineSO(p1, p2, color))
        y += step

    # draw lines parallel to Y axis
    x = -size
    while x <= size:
        p1 = Vec3(x, -size, z)
        p2 = Vec3(x, size, z)
        screen.draw_line(camera, LineSO(p1, p2, color))
        x += step


def draw_cart(screen, camera, origin, theta, phi, hb, hr, eb, ew, dw):
    # TODO: Fix theta rotation
    axle = BoxSO(
        center=origin,
        width=2 * eb,
        height=2 * hr,
        depth=2 * hr,
        theta=theta,
        phi=phi,
        color=(0, 0, 255)
    )
    body = BoxSO(
        center=origin + Vec3.rotate(Vec3(0, 0, hr + 0.5 * (hb - hr)), theta, phi),
        width=2 * hr,
        height=hb - hr,
        depth=2 * hr,
        theta=theta,
        phi=phi,
        color=(0, 0, 255)
    )
    wheel_left = DiskSO(
        center=origin + Vec3.rotate(Vec3(0, eb, 0), theta, phi),
        radius=dw / 2.0,
        thickness=ew,
        theta=0,
        phi=phi + np.pi / 2,
        color=(0, 200, 0),
        steps=24,
    )
    wheel_right = DiskSO(
        center=origin + Vec3.rotate(Vec3(0, -eb, 0), theta, phi),
        radius=dw / 2.0,
        thickness=ew,
        theta=0,
        phi=phi + np.pi / 2,
        color=(0, 200, 0),
        steps=24,
    )

    screen.draw_object(camera, axle)
    screen.draw_object(camera, body)
    screen.draw_object(camera, wheel_left)
    screen.draw_object(camera, wheel_right)
