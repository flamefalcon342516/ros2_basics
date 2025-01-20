import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudListener(Node):
    def __init__(self):
        super().__init__('pointcloud_listener')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )
        self.subscription  # To prevent unused variable warning

    def pointcloud_callback(self, msg):
        # Read points from the PointCloud2 message
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.get_logger().info(f"Number of points: {len(points)}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
