import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math


class MotionController(Node):
    """Node that subscribes to target positions and publishes velocity commands."""

    def __init__(self):
        super().__init__('motion_controller')
        self.subscription = self.create_subscription(
            Point,
            '/target_pos',
            self.target_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/vel_cmd', 10)
        
        # Current robot position (simulated)
        self.current_pos = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.get_logger().info('Motion Controller has started')

    def target_callback(self, msg):
        """React to a new target position by publishing appropriate velocity commands."""
        # Calculate direction vector
        dx = msg.x - self.current_pos["x"]
        dy = msg.y - self.current_pos["y"]
        
        # Calculate distance and angle to target
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_pos["theta"]
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        while angle_diff < -math.pi:
            angle_diff += 2*math.pi
        
        # Create velocity command (suitable for TurtleBot)
        cmd_vel = Twist()
        
        # Angular velocity (turn to face target)
        cmd_vel.angular.z = angle_diff * 0.5  # Proportional control
        
        # Linear velocity (move toward target)
        cmd_vel.linear.x = min(distance * 0.2, 0.2)  # Cap at reasonable speed
        
        self.publisher_.publish(cmd_vel)
        
        # Update simulated position (in reality, this would come from sensors/odometry)
        self.current_pos["theta"] += cmd_vel.angular.z * 0.1  # Simulate rotation
        
        # Move in the direction we're facing (simplified)
        self.current_pos["x"] += cmd_vel.linear.x * math.cos(self.current_pos["theta"]) * 0.1
        self.current_pos["y"] += cmd_vel.linear.x * math.sin(self.current_pos["theta"]) * 0.1
        
        self.get_logger().info(
            f'Target: ({msg.x:.2f}, {msg.y:.2f}), '
            f'Command: v={cmd_vel.linear.x:.2f}, w={cmd_vel.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()