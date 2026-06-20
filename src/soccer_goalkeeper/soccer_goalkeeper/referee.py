#!/usr/bin/env python3

from geometry_msgs.msg import Vector3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Empty, String

from tf2_msgs.msg import TFMessage


class Referee(Node):
    """Classify each launched shot as either a goal or a save."""

    GOAL_LINE_X = 2.7
    GOAL_HALF_WIDTH = 1.0
    CROSSBAR_HEIGHT = 1.25
    BALL_RADIUS = 0.06
    BALL_START_X = -1.5
    MINIMUM_FORWARD_SPEED = 0.10

    def __init__(self) -> None:
        super().__init__('referee')

        self.pose_subscription = self.create_subscription(
            TFMessage,
            '/model/soccer_ball/pose',
            self.pose_callback,
            10
        )
        self.trial_ending_subscription = self.create_subscription(
            Empty,
            '/trial_ending',
            self.trial_ending_callback,
            10
        )
        self.result_publisher = self.create_publisher(
            String,
            '/shot_result',
            10
        )
        self.statistics_publisher = self.create_publisher(
            String,
            '/shot_statistics',
            10
        )
        self.goal_line_event_publisher = self.create_publisher(
            String,
            '/goal_line_event',
            10
        )

        self.previous_position: Vector3 | None = None
        self.previous_time: Time | None = None
        self.shot_active = False
        self.result_recorded = False

        self.goals = 0
        self.saves = 0

    def pose_callback(self, message: TFMessage) -> None:
        if not message.transforms:
            return

        position = message.transforms[0].transform.translation
        current_time = self.get_clock().now()

        if self.previous_time is None or self.previous_position is None:
            self.store_measurement(position, current_time)
            return

        elapsed = (current_time - self.previous_time).nanoseconds / 1e9
        ball_returned_to_start = (
            position.x < self.BALL_START_X + 0.25
            and self.previous_position.x > self.BALL_START_X + 0.50
        )

        # The launcher performs a full-world reset after every trial. If no
        # legal goal crossing occurred before that reset, the goalkeeper
        # successfully prevented a goal.
        if elapsed < 0.0 or ball_returned_to_start:
            self.finish_previous_trial()
            self.start_new_trial()
            self.store_measurement(position, current_time)
            return

        if elapsed <= 0.0:
            return

        velocity_x = (
            position.x - self.previous_position.x
        ) / elapsed

        if not self.shot_active and velocity_x > self.MINIMUM_FORWARD_SPEED:
            self.shot_active = True
            self.get_logger().info('Shot detected')

        if (
            self.shot_active
            and not self.result_recorded
            and self.fully_crossed_goal_line(position)
        ):
            crossing_y, crossing_z = self.goal_line_crossing(position)

            if self.crossing_is_inside_goal(crossing_y, crossing_z):
                self.record_result('GOAL')
            else:
                self.get_logger().info(
                    'Ball crossed outside the goal opening; '
                    'trial remains active until reset'
                )

        self.store_measurement(position, current_time)

    def trial_ending_callback(self, _message: Empty) -> None:
        if self.shot_active and not self.result_recorded:
            self.record_result('SAVE')

    def fully_crossed_goal_line(self, position: Vector3) -> bool:
        previous_position = self.previous_position
        if previous_position is None:
            return False

        previous_trailing_edge = (
            previous_position.x - self.BALL_RADIUS
        )
        current_trailing_edge = position.x - self.BALL_RADIUS
        return (
            previous_trailing_edge < self.GOAL_LINE_X
            <= current_trailing_edge
        )

    def goal_line_crossing(
        self,
        position: Vector3
    ) -> tuple[float, float]:
        previous_position = self.previous_position
        if previous_position is None:
            return position.y, position.z

        previous_edge_x = previous_position.x - self.BALL_RADIUS
        current_edge_x = position.x - self.BALL_RADIUS
        x_distance = current_edge_x - previous_edge_x
        if x_distance <= 0.0:
            return position.y, position.z

        crossing_fraction = (
            (self.GOAL_LINE_X - previous_edge_x)
            / x_distance
        )
        crossing_y = (
            previous_position.y
            + crossing_fraction
            * (position.y - previous_position.y)
        )
        crossing_z = (
            previous_position.z
            + crossing_fraction
            * (position.z - previous_position.z)
        )
        return crossing_y, crossing_z

    def crossing_is_inside_goal(
        self,
        crossing_y: float,
        crossing_z: float
    ) -> bool:
        horizontal_limit = self.GOAL_HALF_WIDTH - self.BALL_RADIUS
        vertical_limit = self.CROSSBAR_HEIGHT - self.BALL_RADIUS
        return (
            abs(crossing_y) <= horizontal_limit
            and self.BALL_RADIUS <= crossing_z <= vertical_limit
        )

    def finish_previous_trial(self) -> None:
        if self.shot_active and not self.result_recorded:
            self.record_result('SAVE')

    def record_result(self, result: str) -> None:
        if self.result_recorded:
            return

        self.result_recorded = True

        if result == 'GOAL':
            self.goals += 1
        else:
            self.saves += 1

        result_message = String()
        result_message.data = result
        self.result_publisher.publish(result_message)

        review_event = String()
        review_event.data = 'GOAL' if result == 'GOAL' else 'NO_GOAL'
        self.goal_line_event_publisher.publish(review_event)

        total_shots = self.goals + self.saves
        save_percentage = (
            100.0 * self.saves / total_shots
            if total_shots > 0
            else 0.0
        )
        statistics = (
            f'shots={total_shots}, goals={self.goals}, '
            f'saves={self.saves}, save_rate={save_percentage:.1f}%'
        )
        statistics_message = String()
        statistics_message.data = statistics
        self.statistics_publisher.publish(statistics_message)

        self.get_logger().info(f'Result: {result} | {statistics}')

    def start_new_trial(self) -> None:
        self.shot_active = False
        self.result_recorded = False
        self.get_logger().info('Ready for next shot')

    def store_measurement(
        self,
        position: Vector3,
        timestamp: Time
    ) -> None:
        self.previous_position = position
        self.previous_time = timestamp


def main(args=None) -> None:
    rclpy.init(args=args)

    node = Referee()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
