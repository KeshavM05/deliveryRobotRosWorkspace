import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class GamepadControl(Node):
    def __init__(self):
        super().__init__('gamepad_control')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Gamepad Control Node Started")

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[1]  # Left joystick Y-axis -> Forward/Backward
        twist.angular.z = msg.axes[2]  # Right joystick X-axis -> Turning
        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = GamepadControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
