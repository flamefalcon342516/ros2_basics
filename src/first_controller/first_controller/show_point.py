import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class DepthToPointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_visualizer')

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_image_callback, 10)

        # Camera parameters and CV Bridge
        self.camera_intrinsics = None
        self.bridge = CvBridge()

        # Initialize the plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()  # Interactive mode on

    def camera_info_callback(self, msg):
        """Callback to extract camera intrinsics."""
        self.camera_intrinsics = np.array(msg.k).reshape(3, 3)  # Intrinsic matrix

    def depth_image_callback(self, msg):
        """Callback to process depth image and visualize the point cloud."""
        if self.camera_intrinsics is None:
            self.get_logger().warn("Camera intrinsics not yet received.")
            return

        # Convert depth image to a NumPy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = depth_image.shape

        # Generate 2D grid of pixel coordinates
        fx, fy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1]
        cx, cy = self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]
        x_indices, y_indices = np.meshgrid(np.arange(width), np.arange(height))

        # Compute 3D coordinates (back-project depth image)
        z = depth_image / 1000.0  # Convert depth to meters if in millimeters
        x = (x_indices - cx) * z / fx
        y = (y_indices - cy) * z / fy

        # Flatten arrays
        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        points = points[~np.isnan(points).any(axis=1)]  # Remove invalid points

        # Plot the points
        self.ax.clear()  # Clear the previous plot
        self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], cmap='viridis', s=0.1)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title("Point Cloud Visualization")
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()  # Keep the final plot open


if __name__ == '__main__':
    main()
