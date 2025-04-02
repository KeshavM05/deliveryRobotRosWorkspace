import rclpy
from rclpy.node import Node
import serial
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')
        
        # Set up serial communication
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Update port as needed

        # Create an odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Transform broadcaster for TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot params
        self.wheel_radius = 0.03  # 3 cm radius
        self.wheel_base = 0.20    # Distance between wheels (meters)
        self.encoder_ticks_per_rev = 500  # Adjust based on your encoder

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = self.get_clock().now()

        # Start timer loop
        self.create_timer(0.1, self.update_odometry)

    def update_odometry(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                data = json.loads(line)

                left_ticks = data["LE"]
                right_ticks = data["RE"]

                # Time difference
                current_time = self.get_clock().now()
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                self.prev_time = current_time

                # Compute wheel velocities
                left_wheel_vel = (left_ticks / self.encoder_ticks_per_rev) * (2 * math.pi * self.wheel_radius) / dt
                right_wheel_vel = (right_ticks / self.encoder_ticks_per_rev) * (2 * math.pi * self.wheel_radius) / dt

                # Compute robot velocity
                linear_vel = (left_wheel_vel + right_wheel_vel) / 2.0
                angular_vel = (right_wheel_vel - left_wheel_vel) / self.wheel_base

                # Update pose
                self.theta += angular_vel * dt
                self.x += linear_vel * math.cos(self.theta) * dt
                self.y += linear_vel * math.sin(self.theta) * dt

                # Publish Odometry Message
                odom_msg = Odometry()
                odom_msg.header.stamp = current_time.to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_link'
                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
                odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
                odom_msg.twist.twist.linear.x = linear_vel
                odom_msg.twist.twist.angular.z = angular_vel

                self.odom_pub.publish(odom_msg)

                # Publish TF transform
                transform = TransformStamped()
                transform.header.stamp = current_time.to_msg()
                transform.header.frame_id = "odom"
                transform.child_frame_id = "base_link"
                transform.transform.translation.x = self.x
                transform.transform.translation.y = self.y
                transform.transform.rotation.z = math.sin(self.theta / 2.0)
                transform.transform.rotation.w = math.cos(self.theta / 2.0)
                self.tf_broadcaster.sendTransform(transform)

            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
