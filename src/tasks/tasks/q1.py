import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class StarDrawer(Node):
    def __init__(self):
        super().__init__('star_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.linear_speed = 2.0
        self.angular_speed = float(144 * math.pi / 180)
        self.timer_period = 1.0 
        self.state = 0
        self.timer = self.create_timer(self.timer_period, self.move_turtle)
        self.get_logger().info('Node started')

    def move_turtle(self):
        msg = Twist()
        if self.state % 2 == 0:
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        self.state += 1
        if self.state == 10:
            self.timer.cancel()
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Node Ended')

def main(args=None):
    rclpy.init(args=args)
    node = StarDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

