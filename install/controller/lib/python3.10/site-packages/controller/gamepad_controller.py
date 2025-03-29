import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')
        self.arduino_serial = serial.Serial('/dev/ttyACM0', 9600)  # Open serial port to Arduino
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # Print all button states
        print(f"Button states: {msg.buttons}")

        # Control commands based on the buttons
        if msg.buttons[0] == 1:  # Button A (F) - Move Forward
            self.send_command("F")
        elif msg.buttons[1] == 1:  # Button B (B) - Move Backward
            self.send_command("B")
        elif msg.buttons[2] == 1:  # Button X (L) - Turn Left
            self.send_command("L")
        elif msg.buttons[3] == 1:  # Button Y (R) - Turn Right
            self.send_command("R")
        elif msg.buttons[6] == 1:  # Button Left Shoulder (S) - Stop
            self.send_command("S")

    def send_command(self, command):
        # Print the command being sent to the Arduino
        print(f"Sending command: {command}")
        
        # Send the command to Arduino via serial
        self.arduino_serial.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    gamepad_controller = GamepadController()
    rclpy.spin(gamepad_controller)

    # Cleanup
    gamepad_controller.arduino_serial.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
