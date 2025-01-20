import sys
import rclpy
from rclpy.node import Node
from tutorial_interface.srv import SetLed

class AddIntsClient(Node):

    def __init__(self):
        super().__init__('add_ints_client')
        self.client  = self.create_client(SetLed,'two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available..')
        self.req = SetLed.Request()  
        self.future= None
        self.create_timer(1.0 , self.timer_callback)  

    def send_request(self, a, b):
        
        self.req.a = a
        self.req.b =b

        self.future = self.client.call_async(self.req)
        self.get_logger().info('Senf Request')     
        
    def timer_callback(self):
        if self.future is None:
            return
        
        if self.future.done():
            try:
                response = self.future.result()
                if response is not None:
                    self.get_logger().info(
                        f"result of add: {self.req.a} + {self.req.b} = {response.sum}"
                    )
                else:
                    self.get_logger().error("Exception..")
            except Exception as e:
                self.get_logger().error("Exception.!!")        

def main(args=None):

    a= int(sys.argv[1])
    b= int(sys.argv[2])

    rclpy.init(args=args)
    sum_client= AddIntsClient()
    sum_client.send_request(a ,b)
    rclpy.spin(sum_client)
    sum_client.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()