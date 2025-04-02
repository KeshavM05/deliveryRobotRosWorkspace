#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math
import serial
import tf2_ros
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.publisher_ = self.create_publisher(Odometry, '/odom', odom_qos)
        self.br = tf2_ros.TransformBroadcaster(self, qos=odom_qos)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info(f"Subscription created with topic: {self.cmd_vel_sub.topic_name}")
        self.ser = None
        self.connect_arduino()
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()
        self.target_linear, self.target_angular = 0.0, 0.0
        self.last_cmd_time = self.get_clock().now()
        self.left_count, self.right_count = 0, 0
        self.last_left, self.last_right = 0, 0
        self.last_encoder_time = self.get_clock().now()
        self.wheel_radius, self.wheel_base = 0.0625, 0.25
        self.counts_per_rev = 3000
        self.create_timer(0.1, self.timer_callback)
        self.create_timer(0.02, self.read_serial)
        self.create_timer(0.1, self.write_serial)

    def connect_arduino(self):
        max_retries = 5
        for i in range(max_retries):
            try:
                self.ser = serial.Serial('/dev/arduino', 9600, timeout=0.1, write_timeout=0.1)
                time.sleep(2)
                self.ser.reset_input_buffer()
                self.get_logger().info(f"Connected to Arduino (attempt {i+1}/{max_retries})")
                return
            except Exception as e:
                self.get_logger().warn(f"Connection failed (attempt {i+1}): {str(e)}")
                time.sleep(1)
        self.get_logger().error("Failed to establish serial connection!")
        self.ser = None

    def cmd_vel_callback(self, msg):
        self.target_linear, self.target_angular = msg.linear.x, msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def write_serial(self):
        if not self.ser:
            return
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 1e9:
            self.target_linear, self.target_angular = 0.0, 0.0
        try:
            self.ser.write(f"{self.target_linear:.3f} {self.target_angular:.3f}\n".encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial: {str(e)}")
            self.ser.close()
            self.ser = None
            self.connect_arduino()

    def read_serial(self):
        if not self.ser or not self.ser.in_waiting:
            return
        try:
            line = self.ser.readline().decode('ascii').strip()
            if not line:
                return
            self.left_count, self.right_count = map(int, line.split(','))
            self.last_encoder_time = self.get_clock().now()
        except (ValueError, UnicodeDecodeError):
            self.get_logger().warn(f"Malformed encoder data: {line}", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Serial error: {str(e)}")
            self.ser.close()
            self.ser = None
            self.connect_arduino()

    def timer_callback(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_encoder_time).nanoseconds > 1e9:
            self.get_logger().warn("No fresh encoder data!", throttle_duration_sec=1)
            return
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        left_delta, right_delta = self.left_count - self.last_left, self.right_count - self.last_right
        left_dist = 2 * math.pi * self.wheel_radius * (left_delta / self.counts_per_rev)
        right_dist = 2 * math.pi * self.wheel_radius * (right_delta / self.counts_per_rev)
        linear, angular = (left_dist + right_dist) / 2, (right_dist - left_dist) / self.wheel_base
        self.x += linear * math.cos(self.theta)
        self.y += linear * math.sin(self.theta)
        self.theta = (self.theta + angular) % (2 * math.pi)
        self.publish_odometry(current_time, linear/dt, angular/dt)
        self.publish_tf(current_time)
        self.last_left, self.last_right, self.last_time = self.left_count, self.right_count, current_time

    def publish_odometry(self, time, linear, angular):
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = time.to_msg(), 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.theta)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = linear, angular
        self.publisher_.publish(msg)

    def publish_tf(self, time):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = time.to_msg(), 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation = self.quaternion_from_euler(0, 0, self.theta)
        self.br.sendTransform(t)

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        q = Quaternion()
        q.w, q.x, q.y, q.z = cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
