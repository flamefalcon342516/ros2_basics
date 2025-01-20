import rclpy
from rclpy.node import Node
from tutorial_interface.msg import Target  

class CustomMsgSubs(Node):
    def __init__(self):
        super().__init__('custom_msg_subs')
        self.subscription = self.create_subscription(
            Target, 
            'custom_topic', 
            self.Receiver, 
            10
        )
        self.get_logger().info('Subscriber ready for ACTION..!!')

    def Receiver(self, msg):
        self.get_logger().info(f'Received: Value = {msg.t_number}, Message = {msg.t_message}')

def main(args=None):
    rclpy.init(args=args)
    node = CustomMsgSubs()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
