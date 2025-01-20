# obstacle_visualizer.py
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2

class ObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('obstacle_visualizer')
        self.subscriber = self.create_subscription(PointCloud2, '/filtered_obstacles', self.callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/rviz_obstacles', 10)

    def callback(self, msg):
        # Simply forward the data to RViz
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
