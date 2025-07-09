import numpy as np
import pygame
import sys

from pygame.locals import *

from vec3 import Vec3
from screen import Camera, Screen, ScreenObject, BoxSO, DiskSO
from drawings import draw_axes, draw_ground, draw_cart


def main():
    hb = 25.0e-2
    hr = 1.0e-2
    eb = 15.0e-2 / 2.0
    ew = 8.0e-3
    dw = 15.0e-2 / 2.0

    camera_pos_factor = 0.01

    pygame.init()
    pygame.font.init()
    screen = Screen(1200, 900)
    clock = pygame.time.Clock()
    camera = Camera(screen)

    dragging = False
    auto_center = True

    running = True
    last_mouse = (0, 0)

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
                camera.angle_pitch = max(0.0, min(np.pi / 2, camera.angle_pitch))
                last_mouse = event.pos
            elif event.type == MOUSEWHEEL:
                camera.distance *= 0.9 if event.y > 0 else 1.1
                camera.distance = max(0.1, min(100.0, camera.distance))
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False

            keys = pygame.key.get_pressed()
            if keys[K_SPACE]:
                auto_center = True
                camera.angle_yaw = 0
                camera.angle_pitch = 0
                camera.distance = 5.0
                camera.target = Vec3(0, 0, 0)
            if keys[K_w]:
                auto_center = False
                camera.target -= camera_pos_factor * Vec3(np.cos(camera.angle_yaw), np.sin(camera.angle_yaw), 0)
            if keys[K_s]:
                auto_center = False
                camera.target += camera_pos_factor * Vec3(np.cos(camera.angle_yaw), np.sin(camera.angle_yaw), 0)
            if keys[K_a]:
                auto_center = False
                camera.target -= camera_pos_factor * np.cross((camera.pos - camera.target).array, Vec3(0, 0, 1).array)
            if keys[K_d]:
                auto_center = False
                camera.target += camera_pos_factor * np.cross((camera.pos - camera.target).array, Vec3(0, 0, 1).array)

        if auto_center:
            camera.target = Vec3(0, 0, 0)

        draw_ground(screen, camera)
        draw_axes(screen, camera)

        draw_cart(screen, camera, Vec3(0.0, 0.0, dw / 2.0), -0.5, np.pi / 4, hb=hb, hr=hr, eb=eb, ew=ew, dw=dw)

        text = f"Yaw: {np.degrees(camera.angle_yaw):.1f}°, Pitch: {np.degrees(camera.angle_pitch):.1f}°"
        screen.draw_text(text, (10, 10))

        screen.render_frame(camera)
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == '__main__':
    main()
