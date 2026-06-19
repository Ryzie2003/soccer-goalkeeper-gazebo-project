#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')

        self.subscription_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    
    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.get_logger().info(f'Goalkeeper position: x ={position.x:.2f}, y={position.y:.3f}')
def main(args=None):
    rclpy.init(args=args)

    node = OdomMonitor()
    rclpy.spin(node)

    rclpy.shutdown()