#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from my_interfaces.srv import LedToggle
from functools import partial


class BatteryNode(Node):

    def __init__(self):
        super().__init__("battery_node")

        self.battery_status_ = "full"

        self.battery_timer_ = self.create_timer(5, self.callback_battery_timer)

        self.ledpanel_update_timer = self.create_timer(
            2, self.callback_ledpanel_update_timer)

        self.get_logger().info("Battery Node Started")

    def callback_battery_timer(self):
        self.battery_status_ = random.choice(["full", "mid", "low", "empty"])

    def callback_ledpanel_update_timer(self):
        if (self.battery_status_ == "full"):
            self.call_ledpanel_server(3)
        elif (self.battery_status_ == "mid"):
            self.call_ledpanel_server(2)
        elif (self.battery_status_ == "low"):
            self.call_ledpanel_server(1)
        else:
            self.call_ledpanel_server(0)

    def call_ledpanel_server(self, lednum):
        client = self.create_client(LedToggle, "led_toggle")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for led_toggle server")

        request = LedToggle.Request()
        request.lednum = lednum

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_ledpanel_server, lednum=lednum))

    def callback_call_ledpanel_server(self, future, lednum):
        try:
            response = future.result()
            self.get_logger().info(f"{lednum} led(s) toggled")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
