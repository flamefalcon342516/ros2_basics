import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from getch import getch  
import time  

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Car Controller Node has been started. Use 'w', 's', 'a', 'd' to control the car and 'q' to quit.")

    def send_velocity_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0, t=0.0):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.linear.z = linear_z
        twist_msg.angular.x = angular_x
        twist_msg.angular.y = angular_y
        twist_msg.angular.z = angular_z
        self.velocity_publisher.publish(twist_msg)
        time.sleep(t)

    def control_loop(self):
        try:
            while rclpy.ok():
                key = getch()     
                if key == 'w':
                    self.get_logger().info("Moving forward")
                    self.send_velocity_command(linear_x=1.0, t=0.02)
                elif key == 's':
                    self.get_logger().info("Moving backward")
                    self.send_velocity_command(linear_x=-1.0, t=0.02) 
                elif key == 'a':
                    self.get_logger().info("Turning left")
                    self.send_velocity_command(angular_z=0.7, t=0.02)  
                elif key == 'd':
                    self.get_logger().info("Turning right")
                    self.send_velocity_command(angular_z=-0.7, t=0.02)  
                elif key == 'e':
                    self.get_logger().info("Moving forward and rotating right")
                    self.send_velocity_command(linear_x=1.0, angular_z=-45668.0, t=0.02)  
                elif key == 'q':
                    self.get_logger().info("Quitting...")
                    break  
                else:
                    self.get_logger().info("Invalid key. Use 'w', 's', 'a', 'd', 'e' to control the car or 'q' to quit.")
                self.send_velocity_command()  # Stop the car after the command
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received, shutting down...")
        finally:
            self.send_velocity_command()  


def main(args=None):
    rclpy.init(args=args)  
    car_controller = CarController()  
    
    try:
        car_controller.control_loop()  
    finally:
        car_controller.destroy_node()  
        rclpy.shutdown()  


if __name__ == '__main__':
    main()
