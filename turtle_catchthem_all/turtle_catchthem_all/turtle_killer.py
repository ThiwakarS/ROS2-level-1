#!/usr/bin/env python3

from my_interfaces.msg._turtle_pose import TurtlePose
import rclpy

from rclpy.node import Node
from math import sqrt, atan2, pi
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from geometry_msgs.msg import Twist


class turtle_killer_Node(Node):

    def __init__(self):
        super().__init__("turtle_killer")

        self.turtle_info = dict()
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.nearest_turtle_name_ = ""

        self.spawn_turtle_subscriber_ = self.create_subscription(
            TurtlePose, "/spawned_turtle", self.callback_spawn_turtle_subscriber, 10)
        self.curr_pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose_subscriber, 10)

        self.nearest_turtle_publisher_ = self.create_publisher(
            TurtlePose, "/nearest_turtle", 10)

        self.turtle_killer_timer_ = self.create_timer(
            0.1, self.callback_turtle_killer_timer)

        self.get_logger().info("Turtle killer node started!")

    def callback_spawn_turtle_subscriber(self, msg):
        x = msg.x
        y = msg.y
        name = msg.name
        self.turtle_info[name] = (x, y)

    def nearest_turtle_finder(self):
        min_dist = 999.0
        for turtle_name in self.turtle_info.keys():
            x = self.turtle_info[turtle_name][0]
            y = self.turtle_info[turtle_name][1]
            distance, _, _ = self.euclidean_distance(
                self.curr_x, self.curr_y, x, y)

            if (min_dist > distance):
                min_dist = distance
                self.nearest_turtle_name_ = turtle_name

    def callback_pose_subscriber(self, msg):
        self.curr_x = msg.x
        self.curr_y = msg.y

    def euclidean_distance(self, x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        distance = sqrt(x**2 + y**2)
        return distance, x, y

    def callback_turtle_killer_timer(self):

        if (self.nearest_turtle_name_ not in self.turtle_info.keys()):
            self.nearest_turtle_finder()

        if (self.nearest_turtle_name_ == "" or len(self.turtle_info) <= 0):
            return

        x2 = self.turtle_info[self.nearest_turtle_name_][0]
        y2 = self.turtle_info[self.nearest_turtle_name_][1]

        distance, _, _ = self.euclidean_distance(
            self.curr_x, self.curr_y, x2, y2)

        msg = TurtlePose()

        if (distance > 0.3):
            msg.x = self.turtle_info[self.nearest_turtle_name_][0]
            msg.y = self.turtle_info[self.nearest_turtle_name_][1]
            msg.name = self.nearest_turtle_name_
            self.nearest_turtle_publisher_.publish(msg)

        elif (distance <= 0.3):

            client = self.create_client(Kill, "/kill")

            while not client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Service turtle kill...")

            request = Kill.Request()
            request.name = self.nearest_turtle_name_

            future = client.call_async(request)
            future.add_done_callback(self.callback_call_kill_server)

            self.turtle_info.pop(self.nearest_turtle_name_)

    def callback_call_kill_server(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Kill service call failed %r" % (e, ))


def main(args=None):
    rclpy.init(args=args)
    node = turtle_killer_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
