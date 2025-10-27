#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpiralDrawer(Node):
    def __init__(self):
        super().__init__('spiral_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_turtle)
        self.declare_parameter('a', 0.0) 
        self.declare_parameter('b', 0.1) 

        self.a = self.get_parameter('a').get_parameter_value().double_value
        self.b = self.get_parameter('b').get_parameter_value().double_value

        self.theta = 0.0 
        self.get_logger().info("Turtle spiral node started.")

    def move_turtle(self):
        msg = Twist()
        # Spiral equation: r = a + b*theta
        r = self.a + self.b * self.theta
        angular_speed = 1.5
        linear_speed = 0.6 * r

        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        self.publisher_.publish(msg)
        self.theta += angular_speed * 0.11
        if r > 9.5:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info("Spiral completed. Ending...")
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
