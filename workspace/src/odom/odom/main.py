import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped


class Odom(Node):

    def __init__(self):
        super().__init__('odom')

        self.subscription = self.create_subscription(
            Int64MultiArray, "/serial/pub", self.callback, 10)
        self.subscription

        self.publisher = self.create_publisher(Odometry, "/odom", 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('r', 0.0),
                ('lx', 0.0),
                ('ly', 0.0),
                ('ticks', 0.0),
                ('child_frame_id', '')])

        self.r = self.get_parameter('r').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        self.ticks = self.get_parameter('ticks').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        if self.r == 0.0 or self.lx == 0.0 or self.ly == 0.0:
            self.get_logger().fatal("Error: zero-parameters expected")
            sys.exit(1)

        kv = self.lx + self.ly
        kw = 1 / (self.lx + self.ly)

        self.Mv = np.array([[1, -1, -kv],
                            [1, 1, kv],
                            [1, 1, -kv],
                            [1, -1, kv]])

        self.Mw = np.array([[1, 1, 1, 1],
                            [-1, 1, 1, -1],
                            [-kw, kw, -kw, kw]])

        print(
            f'r = {self.r}, lx = {self.lx}, ly = {self.ly}, ticks = {self.ticks}, child_frame_id = {self.child_frame_id}')

    def callback(self, msg: Int64MultiArray):
        t0 = msg.data[0]
        t1 = msg.data[1]
        t2 = msg.data[2]
        t3 = msg.data[3]
        phi = np.array([[t0], [t1], [t2], [t3]])
        phi = self.ticks2rads(phi)
        x = self.calcInverseKinematics(phi)
        print(phi)
        print(x)
        print()

        odom = Odometry()

        odom.header.frame_id = "odom"
        odom.child_frame_id = self.child_frame_id
        odom.header.stamp = self.get_clock().now().to_msg()

        odom.pose.pose.position.x = float(x[0])
        odom.pose.pose.position.y = float(x[1])
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(x[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(x[2] / 2.0)

        self.publisher.publish(odom)

        broadcaster = TransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = self.child_frame_id

        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = odom.pose.pose.orientation.x
        transform.transform.rotation.y = odom.pose.pose.orientation.y
        transform.transform.rotation.z = odom.pose.pose.orientation.z
        transform.transform.rotation.w = odom.pose.pose.orientation.w

        broadcaster.sendTransform(transform)

    def ticks2rads(self, phi):
        return phi * 2 * math.pi / self.ticks

    def calcInverseKinematics(self, phi):
        return self.r / 4 * np.dot(self.Mw, phi)

    def calcForwardKinematics(self, x):
        return 1 / self.r * np.dot(self.Mv, x)


def main(args=None):
    rclpy.init(args=args)
    node = Odom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
