#!/usr/bin/env python3

import math

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float64


class CameraBallTracker(Node):
    """Filter 3D camera measurements and predict the goal-line intercept."""

    GOAL_X = 2.7
    BALL_RADIUS = 0.06
    GRAVITY = 9.81
    MINIMUM_FORWARD_SPEED = 0.05
    OBSERVATION_TIMEOUT = 0.5
    ALPHA = 0.65
    BETA = 0.18
    MAXIMUM_PREDICTION_TIME = 4.0

    def __init__(self) -> None:
        super().__init__('camera_ball_tracker')

        self.position_subscription = self.create_subscription(
            PointStamped,
            '/camera_ball_position',
            self.position_callback,
            10
        )
        self.prediction_publisher = self.create_publisher(
            Float64,
            '/camera_predicted_intercept',
            10
        )
        self.prediction_3d_publisher = self.create_publisher(
            PointStamped,
            '/camera_predicted_intercept_3d',
            10
        )

        self.position: list[float] | None = None
        self.velocity: list[float] = [0.0, 0.0, 0.0]
        self.previous_time: Time | None = None
        self.last_observation_time: Time | None = None
        self.shot_active = False
        self.center_published = True
        self.timeout_timer = self.create_timer(
            0.1,
            self.check_observation_timeout
        )

    def position_callback(self, message: PointStamped) -> None:
        current_time = self.get_clock().now()
        measurement = [
            message.point.x,
            message.point.y,
            message.point.z,
        ]
        self.last_observation_time = current_time

        if self.previous_time is None or self.position is None:
            self.position = measurement
            self.previous_time = current_time
            return

        elapsed = (current_time - self.previous_time).nanoseconds / 1e9
        if elapsed < 0.0:
            self.reset_filter()
            return
        if elapsed <= 0.0:
            return

        predicted = [
            self.position[index] + self.velocity[index] * elapsed
            for index in range(3)
        ]
        residual = [
            measurement[index] - predicted[index]
            for index in range(3)
        ]

        self.position = [
            predicted[index] + self.ALPHA * residual[index]
            for index in range(3)
        ]
        self.velocity = [
            self.velocity[index]
            + self.BETA * residual[index] / elapsed
            for index in range(3)
        ]
        self.previous_time = current_time

        x, y, z = self.position
        velocity_x, velocity_y, velocity_z = self.velocity

        if x >= self.GOAL_X:
            self.finish_shot('ball crossed the goal line')
            return
        if velocity_x <= self.MINIMUM_FORWARD_SPEED:
            return

        time_to_goal = (self.GOAL_X - x) / velocity_x
        if not 0.0 < time_to_goal <= self.MAXIMUM_PREDICTION_TIME:
            return

        predicted_y = y + velocity_y * time_to_goal
        predicted_z = (
            z
            + velocity_z * time_to_goal
            - 0.5 * self.GRAVITY * time_to_goal ** 2
        )
        predicted_z = max(self.BALL_RADIUS, predicted_z)

        self.shot_active = True
        self.center_published = False

        lateral_prediction = Float64()
        lateral_prediction.data = predicted_y
        self.prediction_publisher.publish(lateral_prediction)

        prediction_3d = PointStamped()
        prediction_3d.header = message.header
        prediction_3d.header.frame_id = 'goal_line'
        prediction_3d.point.x = predicted_y
        prediction_3d.point.y = predicted_z
        prediction_3d.point.z = time_to_goal
        self.prediction_3d_publisher.publish(prediction_3d)

        speed = math.sqrt(sum(component ** 2 for component in self.velocity))
        self.get_logger().info(
            'Filtered prediction: '
            f'y={predicted_y:.2f}, z={predicted_z:.2f}, '
            f'arrival={time_to_goal:.2f}s, speed={speed:.2f}m/s'
        )

    def check_observation_timeout(self) -> None:
        if not self.shot_active or self.last_observation_time is None:
            return

        elapsed = (
            self.get_clock().now() - self.last_observation_time
        ).nanoseconds / 1e9
        if elapsed < 0.0:
            self.reset_filter()
        elif elapsed >= self.OBSERVATION_TIMEOUT:
            self.finish_shot('camera observation timed out')

    def finish_shot(self, reason: str) -> None:
        self.shot_active = False
        self.publish_center_target(reason)
        self.reset_filter(publish_center=False)

    def publish_center_target(self, reason: str) -> None:
        if self.center_published:
            return

        center_target = Float64()
        center_target.data = 0.0
        self.prediction_publisher.publish(center_target)

        center_3d = PointStamped()
        center_3d.header.stamp = self.get_clock().now().to_msg()
        center_3d.header.frame_id = 'goal_line'
        center_3d.point.x = 0.0
        center_3d.point.y = self.BALL_RADIUS
        center_3d.point.z = 0.0
        self.prediction_3d_publisher.publish(center_3d)

        self.center_published = True
        self.get_logger().info(
            f'{reason}; goalkeeper returning to center'
        )

    def reset_filter(self, publish_center: bool = True) -> None:
        if publish_center:
            self.center_published = False
            self.publish_center_target('simulation reset detected')

        self.position = None
        self.velocity = [0.0, 0.0, 0.0]
        self.previous_time = None
        self.last_observation_time = None
        self.shot_active = False


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CameraBallTracker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
