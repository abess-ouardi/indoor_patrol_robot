import rclpy
from rclpy.node import Node

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.get_logger().info('Patrol node started!')

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    rclpy.shutdown()
