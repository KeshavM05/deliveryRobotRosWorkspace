import pygame

pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
print(f"Joysticks found: {joystick_count}")

if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick Name: {joystick.get_name()}")
else:
    print("No joystick detected.")
