#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class robot_news_station(Node):

    def __init__(self):
        super().__init__("robot_news_station")

        self.news_value_param_ = self.declare_parameter("news", "Robot News")
        self.news_value_ = self.get_parameter("news").value

        self.publisher_ = self.create_publisher(String, "robot_news", 10)

        self.timer_ = self.create_timer(1, self.publish_news)

        self.get_logger().info("robot news station started")

    def publish_news(self):
        msg = String()
        msg.data = self.news_value_

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = robot_news_station()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
