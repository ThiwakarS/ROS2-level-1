#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("py_test")
        self.value_ = 0

        self.get_logger().info("Hello ROS")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Hello {self.value_}")
        self.value_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()