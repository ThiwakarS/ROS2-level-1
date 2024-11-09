import rclpy
import random

from rclpy.node import Node
from turtlesim.msg import Pose
from my_interfaces.msg import TurtlePose
from turtlesim.srv import Spawn
from functools import partial


class turtle_spawner_Node(Node):

    def __init__(self):
        super().__init__("turtle_spawner")

        self.turtlename_param_ = self.declare_parameter("name", "dragon_")
        self.turtlename_ = self.get_parameter("name").value

        self.spawn_range_x = 1.0
        self.spawn_range_y = 9.0
        self.turtle_spawn_timer_ = self.create_timer(
            2, self.callback_turtle_spawn_timer)

        self.spawned_turtle_publisher_ = self.create_publisher(
            TurtlePose, "/spawned_turtle", 10)

        self.turtle_counter_ = 0

        self.get_logger().info("Turtle spawner node started!")

    def callback_turtle_spawn_timer(self):

        x = round(random.uniform(self.spawn_range_x, self.spawn_range_y), 6)
        y = round(random.uniform(self.spawn_range_x, self.spawn_range_y), 6)
        theta = round(random.uniform(-3.14, 3.14), 5)
        client = self.create_client(Spawn, "/spawn")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Service turtle spawn...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = self.turtlename_ + str(self.turtle_counter_)  # type: ignore
        self.turtle_counter_ += 1

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_spawn_service, request=request))

    def callback_spawn_service(self, future, request):
        try:
            response = future.result()
            self.get_logger().info(
                f"turtle {self.turtle_counter_ - 1} spawned with name {response.name}")

            msg = TurtlePose()
            msg.x = request.x
            msg.y = request.y
            msg.name = request.name
            self.spawned_turtle_publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error("Spawn service call failed %r" % (e, ))


def main(args=None):
    rclpy.init(args=args)
    node = turtle_spawner_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
