#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float64

from tf2_msgs.msg import TFMessage


class BallTracker(Node):
    """Publish a ground-truth intercept for debugging and comparison."""

    GOAL_X = 2.7
    MINIMUM_FORWARD_SPEED = 0.05

    def __init__(self) -> None:
        super().__init__('ball_tracker')

        self.subscription = self.create_subscription(
            TFMessage,
            '/model/soccer_ball/pose',
            self.pose_callback,
            10
        )
        self.prediction_publisher = self.create_publisher(
            Float64,
            '/predicted_intercept',
            10
        )

        self.previous_x: float | None = None
        self.previous_y: float | None = None
        self.previous_time: Time | None = None

    def pose_callback(self, message: TFMessage) -> None:
        if not message.transforms:
            return

        position = message.transforms[0].transform.translation
        current_time = self.get_clock().now()

        if (
            self.previous_time is not None
            and self.previous_x is not None
            and self.previous_y is not None
        ):
            elapsed = (
                current_time - self.previous_time
            ).nanoseconds / 1e9

            if elapsed < 0.0:
                self.reset_prediction()
            elif elapsed > 0.0:
                velocity_x = (
                    position.x - self.previous_x
                ) / elapsed
                velocity_y = (
                    position.y - self.previous_y
                ) / elapsed
                self.publish_intercept(
                    position.x,
                    position.y,
                    velocity_x,
                    velocity_y
                )

        self.previous_x = position.x
        self.previous_y = position.y
        self.previous_time = current_time

    def publish_intercept(
        self,
        position_x: float,
        position_y: float,
        velocity_x: float,
        velocity_y: float
    ) -> None:
        if (
            velocity_x <= self.MINIMUM_FORWARD_SPEED
            or position_x >= self.GOAL_X
        ):
            return

        time_to_goal = (
            self.GOAL_X - position_x
        ) / velocity_x
        predicted_y = position_y + velocity_y * time_to_goal

        if abs(predicted_y) < 0.001:
            predicted_y = 0.0

        self.prediction_publisher.publish(Float64(data=predicted_y))
        self.get_logger().info(
            f'Ground-truth crossing: y={predicted_y:.2f} '
            f'in {time_to_goal:.2f} seconds'
        )

    def reset_prediction(self) -> None:
        self.prediction_publisher.publish(Float64(data=0.0))
        self.previous_x = None
        self.previous_y = None
        self.previous_time = None
        self.get_logger().info(
            'Simulation reset detected; comparison target centered'
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BallTracker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
