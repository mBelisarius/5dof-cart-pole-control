import math

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


def draw_ground(screen, camera, z=0.0, size=2.0, step=0.1, color=(128, 128, 128)):
    num_lines = 2 * math.ceil(size / step) + 1
    for i in range(num_lines):
        # Draw lines parallel to X axis
        p1_x = Vec3(-size, -size + i * step, z)
        p2_x = Vec3(size, -size + i * step, z)
        screen.draw_line(camera, LineSO(p1_x, p2_x, color))

        # Draw lines parallel to Y axis
        p1_y = Vec3(-size + i * step, -size, z)
        p2_y = Vec3(-size + i * step, size, z)
        screen.draw_line(camera, LineSO(p1_y, p2_y, color))
