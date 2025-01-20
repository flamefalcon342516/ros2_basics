#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SumIntSubs(Node):
    def __init__(self):
        super().__init__("sum_subs")
        self.a = 0
        self.b = 0
        self.sum = 0
        self.subscription_a = self.create_subscription(Int32, "a", self.calling_a, 10)
        self.subscription_b = self.create_subscription(Int32, "b", self.calling_b, 10)

    def calling_a(self, msg):
        self.a = msg.data
        self.calculate_sum()

    def calling_b(self, msg):
        self.b = msg.data
        self.calculate_sum()

    def calculate_sum(self):
        self.sum = self.a + self.b
        self.get_logger().info(f"Sum: {self.sum}")


def main(args=None):
    rclpy.init(args=args)
    node = SumIntSubs()
    rclpy.spin(node)
    rclpy.shutdown()
