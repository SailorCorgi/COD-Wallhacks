import serial
import pygame
import math

SERIAL_PORT = "/dev/ttyUSB0"   # change if needed
BAUD = 115200

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

pygame.init()

WIDTH = 900
HEIGHT = 700

screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT - 100

SCALE = 0.08

targets = {}

running = True

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    while ser.in_waiting:

        try:
            line = ser.readline().decode().strip()

            parts = line.split(",")

            if len(parts) == 4:

                tid = int(parts[0])
                x = int(parts[1])
                y = int(parts[2])
                speed = int(parts[3])

                targets[tid] = (x, y, speed)

        except:
            pass

    screen.fill((15, 15, 15))

    pygame.draw.circle(
        screen,
        (0, 255, 0),
        (CENTER_X, CENTER_Y),
        300,
        2
    )

    pygame.draw.circle(
        screen,
        (0, 255, 0),
        (CENTER_X, CENTER_Y),
        200,
        1
    )

    pygame.draw.circle(
        screen,
        (0, 255, 0),
        (CENTER_X, CENTER_Y),
        100,
        1
    )

    pygame.draw.line(
        screen,
        (0, 255, 0),
        (CENTER_X, CENTER_Y),
        (CENTER_X, CENTER_Y - 300),
        2
    )

    for tid, (x, y, speed) in targets.items():

        px = CENTER_X + int(x * SCALE)
        py = CENTER_Y - int(y * SCALE)

        pygame.draw.circle(
            screen,
            (255, 60, 60),
            (px, py),
            10
        )

        font = pygame.font.SysFont(None, 24)

        txt = font.render(
            f"T{tid} {speed}",
            True,
            (255,255,255)
        )

        screen.blit(txt, (px + 10, py))

    pygame.display.flip()

    clock.tick(60)

pygame.quit()
