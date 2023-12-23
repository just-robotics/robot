import rclpy
from rclpy.node import Node
from datetime import datetime
import matplotlib.pyplot as plt
from std_msgs.msg import Int64MultiArray, Float32MultiArray


start = datetime.now()


class Plotter(Node):  # Class for sonars

    def __init__(self):
        self.node_name = "plotter"
        super().__init__(self.node_name)

        self.results = Float32MultiArray()
        self.results.data = [0.0, 0.0, 0.0]
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.sub = self.create_subscription(
            Int64MultiArray,
            '/encoder_data',
            self.data_taker_callback,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/pid_coefficients',
            10
        )

    # Callback for subscriber
    def data_taker_callback(self, msg: Int64MultiArray):
        plt.plot(msg.data[0], i, color='b', label="pose")
        plt.plot(msg.data[1], i, color='r', label="target")
        plt.xlabel('Итерации')
        plt.ylabel('Значение')
        plt.title("PID")
        plt.legend()
        plt.show()
        self.i += 1

    # Callback for publisher's timer
    def timer_callback(self):
        self.results.data[0] = float(input("Input Kp:"))
        self.results.data[1] = float(input("Input Kd:"))
        self.results.data[2] = float(input("Input Ki:"))
        self.pub.publish(self.results)
        

def main(args=None):
    rclpy.init(args=args)

    plotter = Plotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
