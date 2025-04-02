import rclpy
from rclpy.node import Node
import serial
import json
import time
import math
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
 
class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom_node')
        # Joystick subscription
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        # Odometry publisher
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.lastL = 0.0
        self.lastR = 0.0
        self.lastTime = time.time()

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return
        self.timer = self.create_timer(0.01, self.read_serial_data)
    def joy_callback(self, msg):
        x_axis = msg.axes[0]
        y_axis = msg.axes[1]
        left_motor = int((y_axis + x_axis) * 255)
        right_motor = int((y_axis - x_axis) * 255)
        left_motor = max(min(left_motor, 255), -255)
        right_motor = max(min(right_motor, 255), -255)
        command = json.dumps({"L": left_motor, "R": right_motor}) + "\n"
        try:
            self.serial_port.write(command.encode('utf-8'))
            self.serial_port.flush()
            self.get_logger().debug(f"Sent: {command.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
    def read_serial_data(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info(f"Received: {line}")
                    newData = line.split(",")
                    if len(newData) >= 4 and newData[0] == newData[2] and newData[1] == newData[3]:
                        self.process_odometry(line)
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
            return
    def process_odometry(self, line):
        newData = line.split(",")
        newL = float(newData[0])
        newR = float(newData[1])
        deltaL = newL #- self.lastL        
        deltaR = newR #- self.lastR
        self.lastL = newL
        self.lastR = newR
        leftTravel = (deltaL / 3000) * 2 * math.pi * 0.0625    
        rightTravel = (deltaR / 3000) * 2 * math.pi * 0.0625
        averageDist = (leftTravel + rightTravel) / 2 
        deltaAngle = (leftTravel - rightTravel) / 0.2775
        self.angle += deltaAngle
        self.x += math.cos(self.angle) * averageDist
        self.y += math.sin(self.angle) * averageDist
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = math.sin(self.angle / 2)
        tf_msg.transform.rotation.w = math.cos(self.angle / 2)
        self.tf_broadcaster.sendTransform(tf_msg)
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.angle / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.angle / 2)
        self.publisher_.publish(odom_msg)
        self.get_logger().info(f"Odom: x={self.x}, y={self.y}, angle={self.angle * 180 / math.pi}")
    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()
 
 
def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
