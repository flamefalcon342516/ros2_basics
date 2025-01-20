import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2 
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time
import math
from geometry_msgs.msg import Twist

k = 10
goal = np.array([10.0, 10.0])

max_vel = 2.0
min_vel = 0.001
max_omega = 0.40
min_omega = -0.40
vel_resolution = 0.1
omega_resolution = 0.1
dt = 0.1
predict_time = 2.0

obstacle_weight = 0.3
goal_weight = 0.4
speed_weight = 0.2

best_score = float('inf')
best_v = 0.00
best_w = 0.00

v = np.arange(min_vel, max_vel, vel_resolution)
w = np.arange(min_omega, max_omega, omega_resolution)
Vx, Vy = np.meshgrid(v, w)
velocity_pairs = np.column_stack((Vx.flatten(), Vy.flatten()))

class obstacle_vel_node(Node):
    def _init_(self):
        super()._init_('obstacle_vel_node')
        self.subscription = self.create_subscription(PointCloud2, '/grouped_points', self.obstacle_callback, 10)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=20))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher1 = self.create_publisher(PointCloud2, '/obstacles', 10)
        self.publisher2 = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.get_logger().info("Obstacle and Vel node started.") 

    def get_current_pose_from_tf(self):
        trans = self.tf_buffer.lookup_transform('kinect_camera', 'base_footprint', rclpy.time.Time())
        return trans.transform.translation, trans.transform.rotation

    def obstacle_callback(self, msg):
        obstacle_msg, obstacles = self.calc_obstacle(msg)
        self.publisher1.publish(obstacle_msg)
        self.get_logger().info("Obstacles detected and obstacle msg published")
        linear, angular = self.get_current_pose_from_tf()
        x, y, qx, qy, qz, qw = linear.x, linear.y, angular.x, angular.y, angular.z, angular.w
        euler = euler_from_quaternion([qx, qy, qz, qw])
        yaw = euler[2]
        current_pose = [x, y, yaw]
        vel_msg = Twist()
        vel_msg = velocity_command(current_pose, obstacles=obstacles)
        if vel_msg is not None:
            self.publisher2.publish(vel_msg)
        else:
            self.get_logger().error("Velocity message is None. Cannot publish.")

    def calc_obstacle(self, msg):
        points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        points = np.array(points)  
        points = np.random.choice(points, points.shape[0] // k)
        points[:, 1] = 0.25  
        obstacle_array = np.array([[p[0], p[1], p[2]] for p in points])
        global obstacles
        obstacles = obstacle_array
        header = msg.header
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        dtype = [("x", np.float32), ("y", np.float32), ("z", np.float32)]
        structured_points = np.array(list(map(tuple, points)), dtype=dtype)
        obstacle_cloud_msg = pc2.create_cloud(header, fields, structured_points)
        return obstacle_cloud_msg, obstacles


def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def predict_trajectory(posn, v, w):
    n_steps = int(predict_time / dt)
    x_start, y_start, theta_start = posn[0], posn[1], posn[2]
    x_inc = abs(v * math.cos(theta_start) * dt)
    y_inc = abs(v * math.sin(theta_start) * dt)
    if abs(y_inc) < 1e-6:
        y_inc = 1e-6
    theta_inc = w * dt
    x = np.arange(x_start, x_start + n_steps * x_inc, x_inc)
    y = np.arange(y_start, y_start + n_steps * y_inc, y_inc)
    theta = np.arange(theta_start, theta_start + n_steps * theta_inc, theta_inc)
    min_length = min(len(x), len(y), len(theta))
    x, y, theta = x[:min_length], y[:min_length], theta[:min_length]
    v_array = np.full(min_length, v)
    w_array = np.full(min_length, w)
    trajectory = np.column_stack((x, y, theta, v_array, w_array))
    return trajectory

def calculate_cost(trajectory, obstacles):
    v = trajectory[0, 3]
    costs = np.empty((0, 3))
    for i in range(trajectory.shape[0]):
        x, y = np.array(trajectory[i, :2]).astype(float)
        distances = np.sqrt((obstacles[:, 0] - x)*2 + (obstacles[:, 1] - y)*2)
        min_distance = np.min(distances)
        obstacle_cost = obstacle_weight / min_distance
        vel_cost = speed_weight / v
        dx, dy = goal[0] - x, goal[1] - y
        dist_cost = np.hypot(dx, dy)
        goal_cost = goal_weight / dist_cost
        final_cost = goal_cost + obstacle_cost + vel_cost
        cost_row = np.array([[final_cost, v, trajectory[0, 4]]])
        costs = np.vstack((costs, cost_row))
    total_cost = np.sum(costs[:, 0])
    return total_cost

def best_trajectory(trajectories, obstacles):
    all_costs = np.array([calculate_cost(traj, obstacles) for traj in trajectories])
    min_index = np.argmin(all_costs)
    best_trajectory = trajectories[min_index]
    best_v = best_trajectory[0, 3]
    best_w = best_trajectory[0, 4]
    return [best_v, best_w]

def velocity_command(posn, obstacles):
    trajectories = [predict_trajectory(posn, v, w) for v, w in velocity_pairs]
    best_attrib = best_trajectory(trajectories, obstacles)
    vel_msg = Twist()
    vel_msg.linear.x = best_attrib[0]
    vel_msg.angular.z = best_attrib[1]
    return vel_msg

def main():
    rclpy.init()
    node = obstacle_vel_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
