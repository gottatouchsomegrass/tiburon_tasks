#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpiralDrawer(Node):
    def __init__(self):
        super().__init__('spiral_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.move_turtle)
        self.angular_speed = 1.0
        self.base_linear_speed = 0.1
        self.current_linear_speed = 0.0
        self.get_logger().info("Node Started.")

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = self.current_linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        r = self.current_linear_speed / self.angular_speed
        self.current_linear_speed += self.base_linear_speed * 0.02
        if r > 4.5:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info("Node Ended.")
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = SpiralDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
