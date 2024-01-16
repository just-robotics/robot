import sys
import numpy as np

import rclpy
from rclpy.node import Node


class Odom(Node):

    def __init__(self):
        super().__init__('odom')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('r', 0.0),
                ('lx', 0.0),
                ('ly', 0.0),])

        self.r = self.get_parameter('r').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value

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

        print(f'r = {self.r}, lx = {self.lx}, ly = {self.ly}')

    def calcInverseKinematics(self, W):
        return self.r / 4 * np.dot(self.Mw, W) / self.r    # WARNING: last r

    def calcForwardKinematics(self, V):
        return 1 / self.r * np.dot(self.Mv, V)


vx = -0.25
vy = -0.1
wz = 0.2

w0 = 2.198
w1 = -2.198
w2 = -2.198
w3 = 2.198

V = np.array([[vx],
              [vy],
              [wz]])

W = np.array([[w0],
              [w1],
              [w2],
              [w3]])


def main(args=None):
    rclpy.init(args=args)
    node = Odom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
