import rclpy
from rclpy.node import Node

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.get_logger().info('Visualization node online!')

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()
