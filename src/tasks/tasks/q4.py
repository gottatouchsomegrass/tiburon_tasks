#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class SpiralDrawer(Node):
    def __init__(self):
        super().__init__('param_spiral_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        linear_param_desc = ParameterDescriptor(
            description='Controls the linear speed of the spiral.')
        angular_param_desc = ParameterDescriptor(
            description='Controls the angular speed of the spiral.')
        
        self.declare_parameter('spiral_linear_velocity', 0.1, linear_param_desc)
        self.declare_parameter('spiral_angular_velocity', 1.0, angular_param_desc)

        self.base_linear_speed = self.get_parameter(
            'spiral_linear_velocity').get_parameter_value().double_value
        self.angular_speed = self.get_parameter(
            'spiral_angular_velocity').get_parameter_value().double_value

        self.current_linear_speed = 0.0

        self.add_on_set_parameters_callback(self.param_callback)

        self.timer = self.create_timer(0.02, self.publish_velocity)
        
        self.get_logger().info('Node started.')
        self.get_logger().info(f'Initial linear speed rate: {self.base_linear_speed}')
        self.get_logger().info(f'Initial angular speed: {self.angular_speed}')

    def param_callback(self, params):
        for p in params:
            if p.name == 'spiral_linear_velocity':
                self.base_linear_speed = p.value
                self.get_logger().info(f'Set linear speed rate to {self.base_linear_speed}')
            elif p.name == 'spiral_angular_velocity':
                self.angular_speed = p.value
                self.get_logger().info(f'Set angular speed to {self.angular_speed}')

        return SetParametersResult(successful=True)

    def publish_velocity(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        msg.linear.x = self.current_linear_speed
        self.publisher_.publish(msg)
        self.current_linear_speed += self.base_linear_speed * 0.02
        r = self.current_linear_speed / self.angular_speed
        if r > 4.5:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info("Node ended.")
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = SpiralDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

