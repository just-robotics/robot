import rclpy
import time
import serial
from rclpy.node import Node

from geometry_msgs.msg import PointStamped

from uwb_package.submodules.readSensorData import readSensorData, openSerialPort


class UWBCoordsStreamer(Node):
    def __init__(self):
        super().__init__('uwb_node')

        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f"port: {self.port}")
        self.get_logger().info(f"baudrate: {self.baudrate}")

        self.ser = serial.Serial(self.port, self.baudrate)
        openSerialPort(self.ser)
        if self.ser.is_open:
            self.get_logger().info(f'Serial {self.port} is opened')
        else:
            self.get_logger().info(f'Serial {self.port} is NOT opened')
        
        self.waiting_time = time.time()

        self.publisher_ = self.create_publisher(
            PointStamped,
            'uwb_coordinates',
            10
        )
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = PointStamped()
        data = readSensorData(self.ser)
        if data is not None:
            msg.point.x = data[0]
            msg.point.y = data[1]
            msg.point.z = data[2]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
            self.get_logger().info(f'[{msg.point.x} {msg.point.y}]')


def main(args=None):
    rclpy.init(args=args)

    uwb_publisher = UWBCoordsStreamer()

    rclpy.spin(uwb_publisher)

    uwb_publisher.destroy_node()
    rclpy.shutdown()
    