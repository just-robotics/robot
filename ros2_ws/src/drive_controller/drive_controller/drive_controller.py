#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class DriveController(Node):
    def __init__(self):
        super().__init__("drive_controller")
        
        self.declare_parameters(namespace='', parameters=[('cmd_vel_sub_topic', ''),
                                                          ('cmd_vel_pub_topic', ''),
                                                          ('odom_sub_topic', ''),
                                                          ('odom_pub_topic', ''),
                                                          ('reset_sub_topic', ''),
                                                          ('reset_pub_topic', ''),])
        
        ns = self.get_namespace()
        
        cmd_vel_sub_topic = self.get_parameter('cmd_vel_sub_topic').value
        cmd_vel_pub_topic = self.get_parameter('cmd_vel_pub_topic').value
        odom_sub_topic = self.get_parameter('odom_sub_topic').value
        odom_pub_topic = self.get_parameter('odom_pub_topic').value
        reset_sub_topic = self.get_parameter('reset_sub_topic').value
        reset_pub_topic = self.get_parameter('reset_pub_topic').value

        self.get_logger().info(f'cmd_vel_sub_topic: {os.path.join(ns, cmd_vel_sub_topic)}')
        self.get_logger().info(f'cmd_vel_pub_topic: {os.path.join(ns, cmd_vel_pub_topic)}')
        self.get_logger().info(f'odom_sub_topic: {os.path.join(ns, odom_sub_topic)}')
        self.get_logger().info(f'odom_pub_topic: {os.path.join(ns, odom_pub_topic)}')
        self.get_logger().info(f'reset_sub_topic: {os.path.join(ns, reset_sub_topic)}')
        self.get_logger().info(f'reset_pub_topic: {os.path.join(ns, reset_pub_topic)}')

        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_sub_topic, self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_sub_topic, self.odom_callback, 10)
        self.reset_sub = self.create_subscription(Bool, reset_sub_topic, self.reset_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_pub_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_pub_topic, 10)
        self.reset_pub = self.create_publisher(Bool, reset_pub_topic, 10)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        self.odom_pub.publish(msg)
        
    def reset_callback(self, msg: Bool):
        self.reset_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
