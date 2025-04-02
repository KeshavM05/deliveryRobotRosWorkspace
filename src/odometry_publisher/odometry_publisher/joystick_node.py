import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joysticknode')
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Initialize Pygame and joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected!")
            return
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Joystick initialized: {self.joystick.get_name()}")

    def timer_callback(self):
        pygame.event.pump()  # Process events

        # Read joystick axes
        x_axis = self.joystick.get_axis(2)  # Right stick X-axis
        y_axis = -self.joystick.get_axis(3)  # Right stick Y-axis (inverted)

        # Create Joy message
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [0.0] * 8  # Default size for Joy message
        joy_msg.axes[0] = x_axis  # Map to first two axes
        joy_msg.axes[1] = y_axis
        joy_msg.buttons = [0] * 12  # No buttons for now

        # Publish the message
        self.publisher.publish(joy_msg)
        self.get_logger().debug(f"Published: x={x_axis}, y={y_axis}")

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
