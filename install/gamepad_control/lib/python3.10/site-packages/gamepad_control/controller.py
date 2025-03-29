import serial  # Import pyserial properly
import pygame
import time

# Initialize serial connection
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)

# Initialize Pygame for gamepad input
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Function to map joystick input to velocity
def get_joystick_values():
    pygame.event.pump()
    for event in pygame.event.get():  # Process events
        pass
    v_d = -joystick.get_axis(1) * 0.5  # Forward/backward speed
    omega_d = joystick.get_axis(0) * 1.0  # Turning rate
    return v_d, omega_d

while True:
    try:
        v_d, omega_d = get_joystick_values()
        print(f"Joystick Input: v_d={v_d:.2f}, omega_d={omega_d:.2f}")

        # Ensure serial is open before writing
        if ser.is_open:
            ser.write(f"{v_d} {omega_d}\n".encode())
            print("Sent to Arduino:", f"{v_d} {omega_d}")

        # Read feedback
        response = ser.readline().decode().strip()
        if response:
            print(f"Arduino Feedback: {response}")

        time.sleep(0.1)
    except KeyboardInterrupt:
        break

ser.close()
pygame.quit()
