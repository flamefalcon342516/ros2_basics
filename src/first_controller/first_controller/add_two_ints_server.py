#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from tutorial_interface.srv import SetLed

class AddTwoIntService(Node):

    def __init__(self):
        super().__init__('two_ints_server')
        self.srv = self.create_service(SetLed,'two_ints',self.add_ints_callback )
        self.get_logger().info('Adding Two integers.!')

    def add_ints_callback(self, request,response):
        self.get_logger().info(f'Received values: a = {request.a}, b = {request.b}')
        response.sum = request.a + request.b
        self.get_logger().info(f'sum: {request.a} + {request.b} = {response.sum}')
        return response    

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
