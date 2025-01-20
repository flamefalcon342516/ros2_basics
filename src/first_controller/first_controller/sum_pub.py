#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SumIntPublisher(Node):
    def __init__(self):
        super().__init__("sum_pub")
        self.sum_publisher1_ = self.create_publisher(Int32, "a", 10)
        self.sum_publisher2_ = self.create_publisher(Int32, "b", 10)
        self.get_logger().info("Publishing the values of 'a' and 'b'...")
        self.timer = self.create_timer(1.0, self.send_values)

    def send_values(self):
        msg1 = Int32()
        msg2 = Int32()
        msg1.data = 3
        msg2.data = 2
        self.sum_publisher1_.publish(msg1)
        self.sum_publisher2_.publish(msg2)
        self.get_logger().info("Published a and b")


def main(args=None):
    rclpy.init(args=args)
    node = SumIntPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
main()