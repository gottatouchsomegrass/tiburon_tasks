#!/usr/bin/env python3
"""
turtle_star.py

Draw a 5-pointed star in turtlesim by publishing to /turtle1/cmd_vel
Uses /turtle1/pose for closed-loop distance/angle control.

Run:
1) Launch turtlesim:
   ros2 run turtlesim turtlesim_node
2) Run this node:
   ros2 run <your_package> turtle_star.py
   or (for quick test) 
   python3 turtle_star.py
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi)."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi


def shortest_angular_diff(target: float, current: float) -> float:
    """Shortest signed angular difference target - current in [-pi, pi)."""
    return normalize_angle(target - current)


class StarDrawer(Node):
    def __init__(
        self,
        linear_speed: float = 1.5,
        angular_speed: float = 1.5,
        side_length: float = 4.5,
    ):
        super().__init__('turtle_star_drawer')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        self.pose = None  # type: Pose | None

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.side_length = side_length

        self.get_logger().info('StarDrawer node started. Waiting for /turtle1/pose...')

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def wait_for_pose(self, timeout_sec: float = 5.0):
        """Wait until a pose message arrives (or timeout)."""
        t0 = self.get_clock().now()
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.01)
            if (self.get_clock().now() - t0).nanoseconds / 1e9 > timeout_sec:
                raise TimeoutError('No /turtle1/pose received (is turtlesim running?)')
        self.get_logger().info('Pose received. Ready to draw.')

    def move_forward(self, distance: float):
        """Move forward a given distance (meters) using pose feedback."""
        if self.pose is None:
            raise RuntimeError('No pose available')

        start_x = self.pose.x
        start_y = self.pose.y

        twist = Twist()
        twist.angular.z = 0.0
        rate_timeout = 0.01

        while rclpy.ok():
            dx = self.pose.x - start_x
            dy = self.pose.y - start_y
            traveled = math.hypot(dx, dy)
            if traveled >= distance - 1e-3:
                break

            twist.linear.x = self.linear_speed
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=rate_timeout)

        # stop
        twist.linear.x = 0.0
        self.pub.publish(twist)

    def rotate_by(self, angle: float):
        """
        Rotate by a signed angle (radians), positive => counter-clockwise.
        Uses proportional angular velocity to avoid shaking.
        """
        if self.pose is None:
            raise RuntimeError('No pose available')

        start_theta = self.pose.theta
        target_theta = normalize_angle(start_theta + angle)

        twist = Twist()
        twist.linear.x = 0.0
        rate_timeout = 0.01

        while rclpy.ok():
            current = self.pose.theta
            ang_diff = shortest_angular_diff(target_theta, current)

            # Stop if within 0.02 radians (~1.1 degrees)
            if abs(ang_diff) < 0.02:
                break

            # Proportional speed: slow down near target
            direction = 1.0 if ang_diff > 0 else -1.0
            twist.angular.z = direction * min(self.angular_speed, max(0.5, 2.0 * abs(ang_diff)))
            self.pub.publish(twist)

            rclpy.spin_once(self, timeout_sec=rate_timeout)
            # self.rotate_by(angle)
            # time.sleep(0.3)

        # stop rotation cleanly
        twist.angular.z = 0.0
        self.pub.publish(twist)


    def draw_star(self):
        """
        Draw a 5-pointed star. Approach: move forward then turn 144 degrees left,
        repeated 5 times. Angle of 144 degrees (exterior turn) produces a star.
        """
        # ensure we have pose
        self.wait_for_pose()

        # 144 degrees in radians
        turn_angle = math.radians(144.0)

        self.get_logger().info('Starting star drawing...')
        for i in range(5):
            self.get_logger().info(f'Segment {i+1}/5: moving forward {self.side_length:.2f}')
            self.move_forward(self.side_length)
            self.get_logger().info(f'Segment {i+1}/5: rotating {math.degrees(turn_angle):.1f} deg')
            # rotate left by +144 deg
            self.rotate_by(turn_angle)

        # ensure stop
        stop = Twist()
        self.pub.publish(stop)
        self.get_logger().info('Star drawing complete. Turtle stopped.')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StarDrawer(
            linear_speed=1.5,   # tune if you want slower/faster motion
            angular_speed=2.0,  # tune rotation speed
            side_length=2.5,    # tune star size (turtlesim window is 11x11)
        )
        # Run the drawing routine once
        node.draw_star()

    except Exception as e:
        print('Error:', e)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
