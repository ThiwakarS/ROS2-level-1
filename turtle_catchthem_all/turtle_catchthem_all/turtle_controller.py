#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from math import sqrt, atan2, pi
from turtlesim.msg import Pose
from my_interfaces.msg import TurtlePose
from geometry_msgs.msg import Twist


class turtle_catchthem_Node(Node):

    def __init__(self):
        super().__init__("turtle_controller")

        self.turtle_info = []

        self.desti_x = -1
        self.desti_y = -1
        self.desti_name = ""

        self.curr_x = 5.0
        self.curr_y = 5.0
        self.curr_theta = 0.0

        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose_subscriber, 10)
        self.nearest_turtle_subscriber_ = self.create_subscription(
            TurtlePose, "/nearest_turtle", self.callback_nearest_turtle_subscriber, 10)

        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)

        self.turtle_control_loop_timer_ = self.create_timer(
            0.1, self.callback_turtle_control_loop_timer_)

        self.get_logger().info("Turtle controller node started!")

    def cmd_vel_publisher_func(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        self.cmd_vel_publisher_.publish(msg)

    def euclidean_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        distance = sqrt(x**2 + y**2)
        return distance, x, y

    def callback_turtle_control_loop_timer_(self):
        distance, x, y = self.euclidean_distance(
            self.curr_x, self.curr_y, self.desti_x, self.desti_y)

        if (self.curr_x == -1 or self.curr_y == -1 or self.desti_name == ""):
            return

        if (distance <= 0.3):
            diff = 0.0
            distance = 0.0

        else:
            goal_theta = atan2(y, x)
            diff = goal_theta - self.curr_theta

            if (diff > pi):
                diff -= 2*pi
            elif (diff < -pi):
                diff += 2*pi

        self.cmd_vel_publisher_func(2*distance, 6*diff)

    def callback_nearest_turtle_subscriber(self, msg):
        self.desti_x = msg.x
        self.desti_y = msg.y
        self.desti_name = msg.name

    def callback_pose_subscriber(self, msg):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_theta = msg.theta


def main(args=None):
    rclpy.init(args=args)
    node = turtle_catchthem_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
