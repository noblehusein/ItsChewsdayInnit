import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random
import math


class TargetPosIssuer(Node):
    """Node that publishes 2D target positions."""

    def __init__(self):
        super().__init__('target_pos_issuer')
        self.publisher_ = self.create_publisher(Point, '/target_pos', 10)
        self.timer = self.create_timer(20.0, self.publish_target_pos)
        self.get_logger().info('Target Position Issuer has started')

    def publish_target_pos(self):
        """Publish a random 2D position."""
        msg = Point()
        # Generate random position within a reasonable range
        msg.x = float(random.uniform(-5.0, 5.0))
        msg.y = float(random.uniform(-5.0, 5.0))
        msg.z = 0.0  # We're only using 2D

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published target position: x={msg.x:.2f}, y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = TargetPosIssuer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()