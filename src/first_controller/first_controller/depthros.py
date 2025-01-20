#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point ,Vector3
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from math import isfinite
from math import atan2, sqrt, pi
import time

class APFObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('Realsense_data')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/filtered_obstacles', 10)    
        self.target_point = Point(x=10.0, y=0.0, z=0.0)
        self.robot_position = Point(x=0.0, y=0.0, z=0.0)
        self.robot_orientation = 0.0
        self.closest_obstacles = []   
        self.obstacle_detected = False    
        self.k_attractive = 2.0
        self.k_repulsive = 10.0
        self.detection_distance = 1.0
        self.influence_radius = 1.0
        self.min_obstacle_dist = 0.2
        self.turn_direction = 1
        self.create_timer(0.05, self.control_loop)

    def filter_points_by_radius(self, points):
        return [
            point for point in points 
            if 0.3 < point[2] < 2.0 or np.linalg.norm([point[0],point[2]]) < self.influence_radius
        ]

    def pointcloud_callback(self, msg):
        self.obstacle_detected = False
        self.closest_obstacles = []  
        points = np.array([
            (point[0], point[1], point[2]) 
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        if points.shape[0] > 1000:
            indices = np.random.choice(points.shape[0], size=1000, replace=False)
            points = points[indices]
        filtered_points = self.filter_points_by_radius(points)

        header = msg.header
        header.frame_id = "map"
        pointcloud_msg = pc2.create_cloud_xyz32(header, filtered_points)
        self.publisher.publish(pointcloud_msg)
        
        for point in filtered_points:
            n_pointx, n_pointy, n_pointz = point
            # print(f"x:{n_pointx},y:{n_pointy},z:{n_pointz}")
            distance = np.sqrt(n_pointx**2 + n_pointy**2 + n_pointz**2)
            # print(distance)
            if n_pointz> 0.2:
                if distance < self.influence_radius and distance > 0.5:
                    self.closest_obstacles.append((n_pointx, n_pointy))
                    # print("obs_values")
                if distance < self.detection_distance and distance >0.5:
                    # print("detected.!!!")
                    self.obstacle_detected = True
                    self.turn_direction = -1 if n_pointy > 0 else 1

    def odom_callback(self, msg):
        self.robot_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)
        self.robot_orientation = atan2(siny_cosp, cosy_cosp)

    def calculate_potential_forces(self):
        dx = self.target_point.x - self.robot_position.x
        dy = self.target_point.y - self.robot_position.y
        # print("hiii1")
        dist_to_goal = sqrt(dx**2 + dy**2)     
        f_att_x = self.k_attractive * dx / dist_to_goal if dist_to_goal > 0 else 0
        f_att_y = self.k_attractive * dy / dist_to_goal if dist_to_goal > 0 else 0        
        f_rep_x = f_rep_y = 0.0        
        # print(f"fax:{f_att_x},fay:{f_att_y}")
        # min_distance = float('inf')
        # min_distance_obs = None

        for obs in self.closest_obstacles:
            dx = obs[0]
            dy = obs[1]
            distance = sqrt(dx**2 + dy**2)
            # print(f"obsx:{obs[0]}, obsy:{obs[1]}, distance: {distance}")
            # if distance < min_distance:
            #     min_distance = distance
            #     min_distance_obs = obs

        # print(f"Closest obstaclex: {min_distance_obs[0]}, closest obstacle y :{min_distance_obs[1]}, Minimum distance: {min_distance}")

            if self.min_obstacle_dist < distance < self.influence_radius :
                dx = obs[0]
                dy = obs[1]
                # magnitude = self.k_repulsive * (1/distance - 1/self.influence_radius)**2
                magnitude = self.k_repulsive
                f_rep_x += (magnitude * dx / distance)
                f_rep_y += (magnitude * dy / distance)
                print(f"obx:{obs[0]},oby:{obs[1]}")
                # print(f"mag:{magnitude}")
                # print(f"f_r_x:{f_rep_x}, f_r_y:{f_rep_y}")
                # print(f"frx:{f_rep_x},fax:{f_att_x},fry:{f_rep_y},fay:{f_att_y}")     
        return f_att_x - f_rep_x, f_att_y - f_rep_y

    def control_loop(self):
        # cmd_vel = Twist()      
        if self.obstacle_detected:
            for obs in self.closest_obstacles:
                dx = obs[0]
                dy = obs[1]
                print(f"obsx:{obs[0]}, obsy:{obs[1]}")
            # self.position = []
            # cmd_vel.linear = Vector3(x = 0.0, y= 0.0, z=0.0)
            # cmd_vel.angular = Vector3(x = 0.0, y= 0.0, z=0.0)
            # self.cmd_vel_pub.publish(cmd_vel)
            print("reacting to obstacles")
            # print(self.robot_position)
            # force_x, force_y = self.calculate_potential_forces()
            # # print(f"f_x:{force_x},f_y:{force_y}")
            # total_force = sqrt(force_x**2 + force_y**2)
            # print(f"total_forces:{total_force}")
            # curr_ori = self.robot_orientation
            # # print(f"cur_or:{curr_ori}")
            # if total_force > 0 and total_force < 80000:
            #     # print("hi_prior_last")
            #     desired_angle = atan2(force_y, force_x)
            #     # print(f"desired:{desired_angle}")
            #     angular_error = desired_angle - self.robot_orientation
            #     # print(angular_error)
            #     # self.position.append((self.robot_position))
            #     while angular_error > 3.14: angular_error -= 6.28
            #     while angular_error < -3.14: angular_error += 6.28    
            #     # print(angular_error)          
            #     if abs(angular_error) > 0.2:
            #         cmd_vel.linear.x = 0.2
            #         cmd_vel.angular.z = 0.3 * angular_error 
            #         # * angular_error
            #         # print("turning to avoid obstacle.")
            #     else:
            #         cmd_vel.linear.x = 0.3         
        else:
            # dx = self.target_point.x - self.robot_position.x
            # dy = self.target_point.y - self.robot_position.y
            # target_heading = atan2(dy, dx)
            print("no obstacles.!!")
        #     distance_to_target = sqrt(dx**2 + dy**2)           
        #     if distance_to_target < 0.2:
        #         cmd_vel.linear.x = 0.0
        #         cmd_vel.angular.z = 0.0
        #         print("done i'm stopping.!!")
        #         self.cmd_vel_pub.publish(cmd_vel)
        #         return
        #     else:
        #         cmd_vel.linear.x = 0.3
        #         # print("peace.!!")
        #         angular_error = target_heading - self.robot_orientation
        #         # print(f"ang:{angular_error}")
        #         while angular_error > 3.14: angular_error -= 6.28
        #         while angular_error < -3.14: angular_error += 6.28
        #         cmd_vel.angular.z = 0.2 * angular_error   
        # self.cmd_vel_pub.publish(cmd_vel)

# ros2 launch realsense2_camera rs_launch.py
# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true

def main():
    rclpy.init()
    node = APFObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()