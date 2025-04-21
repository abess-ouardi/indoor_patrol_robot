import rclpy
from rclpy.node import Node

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('Detection node initialized!')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()
