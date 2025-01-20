import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class MoveToMultiplePoints(Node):
    def __init__(self, target_points):
        super().__init__('move_to_multiple_points')
        self.target_points = target_points
        self.current_target_index = 0
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.linear_speed = 0.5
        self.angular_speed = 0.25
        self.tolerance = 0.2
        self.get_logger().info(f'Navigating through targets: {self.target_points}')
        self.reached_goal = False

    def odom_callback(self, msg):
        if self.current_target_index >= len(self.target_points):
            self.stop_car()
            self.get_logger().info('All goals reached!')
            return

        target_x, target_y = self.target_points[self.current_target_index]
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        target_angle = math.atan2(target_y - current_y, target_x - current_x)
        current_orientation = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        angle_diff = self.normalize_angle(target_angle - current_orientation)
        if distance < self.tolerance:
            self.get_logger().info(f'Target {self.current_target_index + 1} reached!')
            self.current_target_index += 1
            return

        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed if abs(angle_diff) < 0.1 else 0.0
        twist_msg.angular.z = self.angular_speed * angle_diff
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_car(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def get_yaw_from_quaternion(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    print("Enter the target positions for the car as a list (e.g., [(1, 2), (3, 4), (5, 6)]):")
    try:
        target_points = [(0,0),(-3.0,-3.0),(-1.0,-1.0),(1.0,1.0),(3.0,2.0),(5.0,4.0),(7.0,6.0)]
        if not isinstance(target_points, list) or not all(isinstance(p, tuple) and len(p) == 2 for p in target_points):
            raise ValueError
    except ValueError:
        print("Invalid input. Please enter a list of tuples with numeric coordinates.")
        return

    rclpy.init(args=args)
    move_to_multiple_points = MoveToMultiplePoints(target_points)
    rclpy.spin(move_to_multiple_points)
    move_to_multiple_points.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
