import rclpy
from rclpy.node import Node
from tutorial_interface.msg import Target 

class CustomMsgPub(Node):
    def __init__(self):
        super().__init__('custom_msg_pub')
        self.publisher_ = self.create_publisher(Target, 'custom_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  
        self.counter = 0  
        self.get_logger().info('Publisher node started.')

    def publish_message(self):
        msg = Target()
        msg.t_number = self.counter
        msg.t_message = f'This is custom message {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: Received value = {msg.t_number}, Received Message = {msg.t_message}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CustomMsgPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
