import pygame

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick Name: {joystick.get_name()}")
print("Listening for inputs... Move the joystick!")

while True:
    pygame.event.pump()  # Refresh input events
    x_axis = joystick.get_axis(0)  # Left stick X-axis
    y_axis = joystick.get_axis(1)  # Left stick Y-axis
    print(f"X: {x_axis:.2f}, Y: {y_axis:.2f}")
