#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tasks_msgs.srv import DrawShape
import math
import time


class ShapeServer(Node):
    def __init__(self):
        super().__init__('shape_server_time_based')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.service = self.create_service(DrawShape, '/draw_shape', self.draw_shape_callback)
        self.get_logger().info("Shape server Running.")

    def move_forward(self, distance: float, speed: float = 1.0):
        twist = Twist()
        twist.linear.x = speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        t0 = time.time()
        distance_travelled = 0.0

        while rclpy.ok() and distance_travelled < distance:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            t1 = time.time()
            distance_travelled = speed * (t1 - t0)
        twist.linear.x = 0.0
        for _ in range(3):
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)

    def rotate(self, angle_deg: float, angular_speed_deg: float = 60.0):
        angular_speed_rad = math.radians(angular_speed_deg)
        target_angle_rad = math.radians(angle_deg)

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed_rad

        t0 = time.time()
        angle_travelled = 0.0

        while rclpy.ok() and angle_travelled < target_angle_rad:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            t1 = time.time()
            angle_travelled = angular_speed_rad * (t1 - t0)

        twist.angular.z = 0.0
        self.publisher.publish(twist)
        rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)
        time.sleep(0.1)

    def draw_shape_callback(self, request, response):
        shape = request.shape.lower()
        size = request.size
        self.get_logger().info(f"Drawing {shape} with side {size}")

        if shape == "square":
            sides = 4
            turn_angle = 90
        elif shape == "triangle":
            sides = 3
            turn_angle = 120
        else:
            self.get_logger().warn(f"Unknown shape: {shape}")
            response.success = False
            return response

        for i in range(sides):
            self.get_logger().info(f"Drawing side {i + 1}/{sides}")
            self.move_forward(size)
            self.rotate(turn_angle)

        self.get_logger().info("Shape drawn successfully.")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ShapeServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()