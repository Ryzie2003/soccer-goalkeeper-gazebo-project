#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

class GoalkeeperController(Node):
    def __init__(self):
        super().__init__('goalkeeper_controller')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.target_position = 0.0

        self.prediction_subscription = self.create_subscription(
            Float64,
            '/predicted_intercept',
            self.prediction_callback,
            10
        )

    def prediction_callback(self, msg: Float64) -> None:
        self.target_position = msg.data
        self.get_logger().info(
            f'New target position:{self.target_position:.2f}'
        )
    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.get_logger().info(f'Goalkeeper position: x ={position.x:.2f}, y={position.y:.3f}')
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        error = self.target_position - position.x

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