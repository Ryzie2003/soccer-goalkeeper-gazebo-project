#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

class GoalkeeperController(Node):
    def __init__(self):
        super().__init__('goalkeeper_controller')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    
    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.get_logger().info(f'Goalkeeper position: x ={position.x:.2f}, y={position.y:.3f}')
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        error = 0.0 - position.x

        if abs(error) < 0.005:
            cmd.twist.linear.x = 0.0
        else:
            cmd.twist.linear.x = 0.5 * error
        
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    node = GoalkeeperController()
    rclpy.spin(node)

    rclpy.shutdown()