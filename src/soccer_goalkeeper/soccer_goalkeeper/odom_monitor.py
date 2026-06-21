#!/usr/bin/env python3

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


class OdomMonitor(Node):
    """Log the goalkeeper position for debugging."""

    def __init__(self) -> None:
        super().__init__('odom_monitor')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.get_logger().info(
            f'Goalkeeper position: '
            f'x={position.x:.2f}, y={position.y:.3f}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OdomMonitor()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
