#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def normalize_angle(angle: float) -> float:
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi


def shortest_angular_diff(target: float, current: float) -> float:
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
        t0 = self.get_clock().now()
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.01)
            if (self.get_clock().now() - t0).nanoseconds / 1e9 > timeout_sec:
                raise TimeoutError('No /turtle1/pose received (is turtlesim running?)')
        self.get_logger().info('Pose received. Ready to draw.')

    def move_forward(self, distance: float):
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
            if abs(ang_diff) < 0.02:
                break
            direction = 1.0 if ang_diff > 0 else -1.0
            twist.angular.z = direction * min(self.angular_speed, max(0.5, 2.0 * abs(ang_diff)))
            self.pub.publish(twist)

            rclpy.spin_once(self, timeout_sec=rate_timeout)
        twist.angular.z = 0.0
        self.pub.publish(twist)


    def draw_star(self):
        self.wait_for_pose()

        turn_angle = math.radians(144.0)

        self.get_logger().info('Starting star drawing...')
        for i in range(5):
            self.get_logger().info(f'Segment {i+1}/5: moving forward {self.side_length:.2f}')
            self.move_forward(self.side_length)
            self.get_logger().info(f'Segment {i+1}/5: rotating {math.degrees(turn_angle):.1f} deg')
            self.rotate_by(turn_angle) #left rotnation

        stop = Twist()
        self.pub.publish(stop)
        self.get_logger().info('Star drawing complete. Turtle stopped.')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StarDrawer(
            linear_speed=1.5,   
            angular_speed=2.0,  
            side_length=2.5,   
        )

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
