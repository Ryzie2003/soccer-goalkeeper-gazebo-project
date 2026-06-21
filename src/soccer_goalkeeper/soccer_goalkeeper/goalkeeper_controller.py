#!/usr/bin/env python3

from geometry_msgs.msg import PointStamped, TwistStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import EntityWrench

from std_msgs.msg import String


class GoalkeeperController(Node):
    """Select lateral, jump, and dive actions from camera predictions."""

    TARGET_LIMIT = 0.68
    JUMP_HEIGHT_THRESHOLD = 0.62
    JUMP_TRIGGER_TIME = 0.85
    DIVE_DISTANCE_THRESHOLD = 0.32
    DIVE_TRIGGER_TIME = 1.10

    def __init__(self) -> None:
        super().__init__('goalkeeper_controller')

        self.declare_parameter(
            'prediction_topic',
            '/camera_predicted_intercept_3d'
        )
        self.declare_parameter('control_gain', 7.0)
        self.declare_parameter('max_speed', 2.8)
        self.declare_parameter('dive_speed', 3.3)
        self.declare_parameter('jump_force', 550.0)
        self.declare_parameter('deadband', 0.01)

        prediction_topic = self.get_parameter(
            'prediction_topic'
        ).get_parameter_value().string_value
        self.control_gain = self.get_parameter(
            'control_gain'
        ).get_parameter_value().double_value
        self.max_speed = self.get_parameter(
            'max_speed'
        ).get_parameter_value().double_value
        self.dive_speed = self.get_parameter(
            'dive_speed'
        ).get_parameter_value().double_value
        self.jump_force = self.get_parameter(
            'jump_force'
        ).get_parameter_value().double_value
        self.deadband = self.get_parameter(
            'deadband'
        ).get_parameter_value().double_value

        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        self.wrench_publisher = self.create_publisher(
            EntityWrench,
            '/world/default/wrench',
            10
        )
        self.action_publisher = self.create_publisher(
            String,
            '/goalkeeper/action',
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.prediction_subscription = self.create_subscription(
            PointStamped,
            prediction_topic,
            self.prediction_callback,
            10
        )

        self.target_position = 0.0
        self.predicted_height = 0.06
        self.arrival_time = 0.0
        self.current_position = 0.0
        self.current_action = 'READY'
        self.jump_committed = False

        self.get_logger().info(
            f'Using 3D prediction topic: {prediction_topic}'
        )

    def prediction_callback(self, message: PointStamped) -> None:
        self.target_position = max(
            -self.TARGET_LIMIT,
            min(self.TARGET_LIMIT, message.point.x)
        )
        self.predicted_height = message.point.y
        self.arrival_time = message.point.z

        if self.arrival_time <= 0.0:
            self.recover()
            return

        lateral_error = self.target_position - self.current_position
        should_dive = (
            abs(lateral_error) >= self.DIVE_DISTANCE_THRESHOLD
            and self.arrival_time <= self.DIVE_TRIGGER_TIME
        )
        should_jump = (
            self.predicted_height >= self.JUMP_HEIGHT_THRESHOLD
            and self.arrival_time <= self.JUMP_TRIGGER_TIME
        )

        if should_dive:
            self.set_action('DIVE_LEFT' if lateral_error > 0 else 'DIVE_RIGHT')
            if should_jump:
                self.trigger_jump(scale=0.75)
        elif should_jump:
            self.set_action('JUMP')
            self.trigger_jump(scale=1.0)
        else:
            self.set_action('TRACK')

    def odom_callback(self, message: Odometry) -> None:
        self.current_position = message.pose.pose.position.x
        error = self.target_position - self.current_position

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()

        if abs(error) < self.deadband:
            command.twist.linear.x = 0.0
        else:
            speed_limit = (
                self.dive_speed
                if self.current_action.startswith('DIVE')
                else self.max_speed
            )
            requested_speed = self.control_gain * error
            command.twist.linear.x = max(
                -speed_limit,
                min(speed_limit, requested_speed)
            )

        self.velocity_publisher.publish(command)

    def trigger_jump(self, scale: float) -> None:
        if self.jump_committed:
            return

        jump = EntityWrench()
        jump.entity.name = 'burger::base_link'
        jump.entity.type = 3  # LINK
        jump.wrench.force.z = self.jump_force * scale
        self.wrench_publisher.publish(jump)
        self.jump_committed = True

        self.get_logger().info(
            f'Jump impulse commanded: {jump.wrench.force.z:.0f} N'
        )

    def set_action(self, action: str) -> None:
        if action == self.current_action:
            return

        self.current_action = action
        self.action_publisher.publish(String(data=action))
        self.get_logger().info(
            f'Goalkeeper action: {action}; '
            f'target={self.target_position:.2f}, '
            f'height={self.predicted_height:.2f}, '
            f'arrival={self.arrival_time:.2f}s'
        )

    def recover(self) -> None:
        self.target_position = 0.0
        self.predicted_height = 0.06
        self.arrival_time = 0.0
        self.jump_committed = False
        self.set_action('RECOVER')


def main(args=None) -> None:
    rclpy.init(args=args)

    node = GoalkeeperController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
