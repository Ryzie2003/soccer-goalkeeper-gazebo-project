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
        if position.x < 0.9:
            cmd.twist.linear.x = 0.2
        elif position.x > 1.1:
            cmd.twist.linear.x = -0.2
        else:
            cmd.twist.linear.x = 0.0
        
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    node = GoalkeeperController()
    rclpy.spin(node)

    rclpy.shutdown()