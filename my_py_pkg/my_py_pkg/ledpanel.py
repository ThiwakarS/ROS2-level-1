#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_interfaces.msg import LedState
from my_interfaces.srv import LedToggle


class LedPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel_node")
        self.ledstate_publisher_ = self.create_publisher(
            LedState, "led_states", 10)

        self.timer_ = self.create_timer(1, self.publish_ledstate)
        self.led_states_ = [1, 1, 1]

        self.ledtoggle_server_ = self.create_service(
            LedToggle, "led_toggle", self.callback_ledtoggle_server)
        self.get_logger().info("LED panel node started")

    def publish_ledstate(self):
        msg = LedState()
        msg.state = self.led_states_
        self.ledstate_publisher_.publish(msg)

    def callback_ledtoggle_server(self, request, response):
        if (0 <= request.lednum <= 3):
            if (request.lednum == 0):
                self.led_states_ = [0, 0, 0]
            elif (request.lednum == 1):
                self.led_states_ = [0, 0, 1]
            elif (request.lednum == 2):
                self.led_states_ = [0, 1, 1]
            elif (request.lednum == 3):
                self.led_states_ = [1, 1, 1]

            response.success = True
            response.debug_message = "Service called successfully"

            self.get_logger().info(
                f"{request.lednum} led(s) toggled, service successful")

        else:
            response.success = False
            response.debud_message = "Service call failed"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
