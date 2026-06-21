#!/usr/bin/env python3

import math

from geometry_msgs.msg import PointStamped, TwistStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ros_gz_interfaces.msg import EntityWrench

from std_msgs.msg import String


class GoalkeeperController(Node):
    """Select lateral, jump, and dive actions from camera predictions."""

    TARGET_LIMIT = 0.78
    JUMP_HEIGHT_THRESHOLD = 0.42
    JUMP_TRIGGER_TIME = 1.80
    DIVE_DISTANCE_THRESHOLD = 0.16
    DIVE_TRIGGER_TIME = 2.20
    ACTION_DURATION = 0.12
    ACTION_RATE = 150.0
    GRAVITY = 9.81
    BODY_CENTER_HEIGHT = 0.46

    def __init__(self) -> None:
        super().__init__('goalkeeper_controller')

        self.declare_parameter(
            'prediction_topic',
            '/camera_predicted_intercept_3d'
        )
        self.declare_parameter('control_gain', 7.0)
        self.declare_parameter('max_speed', 2.8)
        self.declare_parameter('dive_speed', 5.0)
        self.declare_parameter('estimated_mass', 1.2)
        self.declare_parameter('launch_force_gain', 115.0)
        self.declare_parameter('minimum_lateral_force', 150.0)
        self.declare_parameter('minimum_vertical_force', 255.0)
        self.declare_parameter('maximum_lateral_force', 340.0)
        self.declare_parameter('maximum_vertical_force', 425.0)
        self.declare_parameter('dive_roll_torque', 25.0)
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
        self.estimated_mass = self.get_parameter(
            'estimated_mass'
        ).get_parameter_value().double_value
        self.launch_force_gain = self.get_parameter(
            'launch_force_gain'
        ).get_parameter_value().double_value
        self.minimum_lateral_force = self.get_parameter(
            'minimum_lateral_force'
        ).get_parameter_value().double_value
        self.minimum_vertical_force = self.get_parameter(
            'minimum_vertical_force'
        ).get_parameter_value().double_value
        self.maximum_lateral_force = self.get_parameter(
            'maximum_lateral_force'
        ).get_parameter_value().double_value
        self.maximum_vertical_force = self.get_parameter(
            'maximum_vertical_force'
        ).get_parameter_value().double_value
        self.dive_roll_torque = self.get_parameter(
            'dive_roll_torque'
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
        self.current_height = 0.05
        self.current_action = 'READY'
        self.jump_committed = False
        self.action_committed = False
        self.action_start_time: Time | None = None
        self.action_vertical_force = 0.0
        self.action_lateral_force = 0.0
        self.action_roll_torque = 0.0
        self.action_timer = self.create_timer(
            1.0 / self.ACTION_RATE,
            self.publish_active_wrench
        )

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

        # Once a jump or dive begins, keep that action through the shot.
        # New camera estimates may refine the target but should not cancel the
        # visible movement halfway through it.
        if self.action_committed:
            return

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
            self.action_committed = True
            self.trigger_diagonal_launch(
                lateral_error,
                include_vertical_launch=should_jump
            )
        elif should_jump:
            self.set_action('JUMP')
            self.action_committed = True
            self.trigger_diagonal_launch(
                0.0,
                include_vertical_launch=True
            )
        else:
            self.set_action('TRACK')

    def odom_callback(self, message: Odometry) -> None:
        self.current_position = message.pose.pose.position.x
        self.current_height = message.pose.pose.position.z
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

    def trigger_diagonal_launch(
        self,
        lateral_error: float,
        include_vertical_launch: bool
    ) -> None:
        launch_time = max(
            0.25,
            min(0.70, self.arrival_time * 0.70)
        )
        desired_lateral_velocity = lateral_error / launch_time

        # Raise the base only enough for the player's center and fixed arms to
        # overlap the predicted ball height.
        desired_base_height = max(
            self.current_height,
            min(0.65, self.predicted_height - self.BODY_CENTER_HEIGHT)
        )
        vertical_distance = desired_base_height - self.current_height
        desired_vertical_velocity = (
            vertical_distance
            + 0.5 * self.GRAVITY * launch_time ** 2
        ) / launch_time

        lateral_force = (
            self.estimated_mass
            * desired_lateral_velocity
            * self.launch_force_gain
        )
        vertical_force = 0.0
        if include_vertical_launch:
            vertical_force = (
                self.estimated_mass
                * desired_vertical_velocity
                * self.launch_force_gain
            )

        direction = 0.0
        if abs(lateral_error) > 0.01:
            direction = math.copysign(1.0, lateral_error)
            lateral_force = direction * max(
                self.minimum_lateral_force,
                abs(lateral_force)
            )

        lateral_force = max(
            -self.maximum_lateral_force,
            min(self.maximum_lateral_force, lateral_force)
        )
        if include_vertical_launch:
            vertical_force = max(
                self.minimum_vertical_force,
                min(self.maximum_vertical_force, vertical_force)
            )

        self.action_start_time = self.get_clock().now()
        self.action_vertical_force = vertical_force
        self.action_lateral_force = lateral_force
        self.action_roll_torque = -direction * self.dive_roll_torque
        self.jump_committed = True

        self.get_logger().info(
            'Diagonal launch started: '
            f'target_y={self.target_position:.2f}m, '
            f'target_z={self.predicted_height:.2f}m, '
            f'high_shot={include_vertical_launch}, '
            f'launch_time={launch_time:.2f}s, '
            f'lateral={lateral_force:.0f} N, '
            f'vertical={vertical_force:.0f} N, '
            f'roll={self.action_roll_torque:.0f} Nm, '
            f'duration={self.ACTION_DURATION:.2f}s'
        )

    def publish_active_wrench(self) -> None:
        if self.action_start_time is None:
            return

        elapsed = (
            self.get_clock().now() - self.action_start_time
        ).nanoseconds / 1e9
        if elapsed < 0.0 or elapsed > self.ACTION_DURATION:
            self.action_start_time = None
            return

        wrench = EntityWrench()
        wrench.entity.name = 'burger::base_link'
        wrench.entity.type = 3  # LINK
        wrench.wrench.force.y = self.action_lateral_force
        wrench.wrench.force.z = self.action_vertical_force
        wrench.wrench.torque.x = self.action_roll_torque
        self.wrench_publisher.publish(wrench)

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
        self.action_committed = False
        self.action_start_time = None
        self.action_vertical_force = 0.0
        self.action_lateral_force = 0.0
        self.action_roll_torque = 0.0
        self.set_action('RECOVER')


def main(args=None) -> None:
    rclpy.init(args=args)

    node = GoalkeeperController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
