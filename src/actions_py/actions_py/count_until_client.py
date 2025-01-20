#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_action.action import C

class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_= ActionClient(
            self,
            CountUntil,
            "count_until")

    def send_goal(self, target_number, period):
        #wait for the server
        self.count_until_client_.wait_for_server()

        #create a goal
        goal = CountUntil.Goal()     

def main(args=None):
    rclpy.init(args=args)    
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()        