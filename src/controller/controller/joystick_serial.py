import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import json
import time

class JoystickSerial(Node):
    def __init__(self):
        super().__init__('joystick_serial')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Initialize serial connection to Arduino
        self.serial_port = '/dev/ttyACM0'  # Change this based on your setup (use `ls /dev/tty*`)
        self.baud_rate = 115200
        self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        self.last_sent_time = time.time()
        self.send_interval = 0.1  # Send data every 100ms to prevent serial buffer overload

    def joy_callback(self, msg):
        # Right joystick of Logitech F710 (ABS_RX: index 3, ABS_RY: index 4)
        x_axis = msg.axes[3]  # Right stick X-axis
        y_axis = msg.axes[4]  # Right stick Y-axis

        # Convert joystick input to motor speed
        max_speed = 255
        left_motor_speed = int((y_axis + x_axis) * max_speed)
        right_motor_speed = int((y_axis - x_axis) * max_speed)

        # Ensure speed is within -255 to 255
        left_motor_speed = max(-255, min(255, left_motor_speed))
        right_motor_speed = max(-255, min(255, right_motor_speed))

        # Only send if interval has passed
        if time.time() - self.last_sent_time > self.send_interval:
            data = {
                "l": left_motor_speed,
                "r": right_motor_speed
            }
            json_data = json.dumps(data, separators=(',', ':'))

            try:
                self.serial_conn.write((json_data + "\n").encode())  # Send JSON + newline
                self.get_logger().info(f"Sending: {json_data}")
                self.last_sent_time = time.time()
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_conn.close()  # Close serial connection when exiting
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
