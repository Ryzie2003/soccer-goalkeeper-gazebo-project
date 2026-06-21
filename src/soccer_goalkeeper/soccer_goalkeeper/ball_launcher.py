#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ros_gz_interfaces.msg import EntityWrench
from ros_gz_interfaces.srv import ControlWorld

from std_msgs.msg import Empty

from tf2_msgs.msg import TFMessage


class BallLauncher(Node):
    LAUNCH_DELAY = 1.0
    VERIFICATION_DELAY = 0.35
    MINIMUM_MOVEMENT = 0.03
    MAXIMUM_ATTEMPTS = 5
    RESET_DELAY = 5.0
    REVIEW_LEAD_TIME = 1.0

    def __init__(self) -> None:
        super().__init__('ball_launcher')

        self.declare_parameter('minimum_forward_force', 800.0)
        self.declare_parameter('maximum_forward_force', 2900.0)
        self.declare_parameter('maximum_lateral_force', 360.0)
        self.declare_parameter('minimum_vertical_force', 180.0)
        self.declare_parameter('maximum_vertical_force', 1100.0)
        self.declare_parameter('trial_count', 30)

        self.minimum_forward_force = self.get_parameter(
            'minimum_forward_force'
        ).value
        self.maximum_forward_force = self.get_parameter(
            'maximum_forward_force'
        ).value
        self.maximum_lateral_force = self.get_parameter(
            'maximum_lateral_force'
        ).value
        self.minimum_vertical_force = self.get_parameter(
            'minimum_vertical_force'
        ).value
        self.maximum_vertical_force = self.get_parameter(
            'maximum_vertical_force'
        ).value
        self.trial_count = max(
            1,
            int(self.get_parameter('trial_count').value)
        )

        self.publisher_ = self.create_publisher(
            EntityWrench,
            '/world/default/wrench',
            10
        )
        self.pose_subscription = self.create_subscription(
            TFMessage,
            '/model/soccer_ball/pose',
            self.pose_callback,
            10
        )
        self.trial_ending_publisher = self.create_publisher(
            Empty,
            '/trial_ending',
            10
        )
        self.reset_client = self.create_client(
            ControlWorld,
            '/world/default/control'
        )

        self.sequence_timer = self.create_timer(0.05, self.update_sequence)
        self.sequence_start: Time | None = None
        self.shot_fired_ = False
        self.reset_requested = False
        self.pending_shot: EntityWrench | None = None
        self.shot_origin: tuple[float, float, float] | None = None
        self.latest_position: tuple[float, float, float] | None = None
        self.last_attempt_time: Time | None = None
        self.confirmed_shot_time: Time | None = None
        self.launch_attempts = 0
        self.waiting_for_bridge_logged = False
        self.waiting_for_pose_logged = False
        self.launch_failed = False
        self.trial_ending_published = False
        self.completed_trials = 0
        self.waiting_for_reset = False
        self.reset_response_received = False
        self.ball_reset_observed = False
        self.batch_complete = False
        self.pending_shot_style = 'UNSET'

        self.get_logger().info(
            f'Automatic trial batch configured for {self.trial_count} shots'
        )

    def update_sequence(self) -> None:
        if self.batch_complete or self.waiting_for_reset:
            return

        current_time = self.get_clock().now()

        # Start timing only after this node has received its first clock value.
        if self.sequence_start is None:
            self.sequence_start = current_time
            return

        elapsed = (current_time - self.sequence_start).nanoseconds / 1e9

        if elapsed < 0.0:
            self.sequence_start = current_time
            return

        if not self.shot_fired_ and not self.launch_failed:
            if self.pending_shot is not None:
                self.verify_or_retry_shot(current_time)
            elif elapsed >= self.LAUNCH_DELAY:
                self.prepare_shot(current_time)

        if (
            self.shot_fired_
            and not self.reset_requested
            and self.confirmed_shot_time is not None
        ):
            shot_elapsed = (
                current_time - self.confirmed_shot_time
            ).nanoseconds / 1e9

            if (
                not self.trial_ending_published
                and shot_elapsed
                >= self.RESET_DELAY - self.REVIEW_LEAD_TIME
            ):
                self.trial_ending_publisher.publish(Empty())
                self.trial_ending_published = True
                self.get_logger().info(
                    'Trial ending announced for goal-line review'
                )

            if shot_elapsed >= self.RESET_DELAY:
                self.reset_ball()

    def pose_callback(self, message: TFMessage) -> None:
        if not message.transforms:
            return

        position = message.transforms[0].transform.translation
        self.latest_position = (position.x, position.y, position.z)

        if (
            self.waiting_for_reset
            and self.reset_response_received
            and self.ball_is_at_start()
        ):
            self.ball_reset_observed = True
            self.try_start_next_trial()

    def prepare_shot(self, current_time: Time) -> None:
        if self.publisher_.get_subscription_count() == 0:
            if not self.waiting_for_bridge_logged:
                self.get_logger().warning(
                    'Waiting for the Gazebo wrench bridge'
                )
                self.waiting_for_bridge_logged = True
            return

        if self.latest_position is None:
            if not self.waiting_for_pose_logged:
                self.get_logger().warning(
                    'Waiting for the ball pose before launching'
                )
                self.waiting_for_pose_logged = True
            return

        self.waiting_for_bridge_logged = False
        self.waiting_for_pose_logged = False
        shot = EntityWrench()
        shot.entity.name = 'soccer_ball::ball_link'
        shot.entity.type = 3  # LINK
        self.configure_shot_profile(shot)

        self.pending_shot = shot
        self.shot_origin = self.latest_position
        self.launch_attempts = 0
        self.send_shot_attempt(current_time)

    def send_shot_attempt(self, current_time: Time) -> None:
        pending_shot = self.pending_shot
        if pending_shot is None:
            return

        self.publisher_.publish(pending_shot)
        self.launch_attempts += 1
        self.last_attempt_time = current_time

        self.get_logger().info(
            f'Trial {self.completed_trials + 1}/{self.trial_count}: '
            f'{self.pending_shot_style} shot sent '
            f'(attempt {self.launch_attempts}) with force '
            f'x={pending_shot.wrench.force.x:.0f}, '
            f'y={pending_shot.wrench.force.y:.0f}, '
            f'z={pending_shot.wrench.force.z:.0f} N'
        )

    def verify_or_retry_shot(self, current_time: Time) -> None:
        if self.last_attempt_time is None or self.latest_position is None:
            return
        if self.shot_origin is None:
            return

        elapsed = (
            current_time - self.last_attempt_time
        ).nanoseconds / 1e9
        if elapsed < self.VERIFICATION_DELAY:
            return

        displacement = sum(
            (
                current - origin
            ) ** 2
            for current, origin in zip(
                self.latest_position,
                self.shot_origin
            )
        ) ** 0.5

        if displacement >= self.MINIMUM_MOVEMENT:
            self.shot_fired_ = True
            self.confirmed_shot_time = current_time
            self.pending_shot = None
            self.get_logger().info(
                f'Trial {self.completed_trials + 1}/{self.trial_count}: '
                'shot confirmed; ball moved '
                f'{displacement:.2f} m'
            )
            return

        if self.launch_attempts < self.MAXIMUM_ATTEMPTS:
            self.get_logger().warning(
                'Ball did not move; retrying shot command'
            )
            self.send_shot_attempt(current_time)
            return

        self.get_logger().error(
            'Shot failed after '
            f'{self.MAXIMUM_ATTEMPTS} attempts; '
            'check the /world/default/wrench bridge'
        )
        self.pending_shot = None
        self.launch_failed = True

    def reset_ball(self) -> None:
        if not self.shot_fired_ or self.reset_requested:
            return

        if not self.reset_client.service_is_ready():
            self.get_logger().warning('Reset service is not ready')
            return

        request = ControlWorld.Request()
        request.world_control.reset.all = True

        future = self.reset_client.call_async(request)
        future.add_done_callback(self.reset_finished)

        self.reset_requested = True
        self.waiting_for_reset = True
        self.reset_response_received = False
        self.ball_reset_observed = False
        self.get_logger().info('Simulation reset requested')

    def reset_finished(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: B902
            self.get_logger().error(f'Simulation reset failed: {error}')
            self.reset_requested = False
            self.waiting_for_reset = False
            return

        if response.success:
            self.reset_response_received = True
            self.completed_trials += 1
            self.get_logger().info(
                'Simulation reset completed; '
                f'{self.completed_trials}/{self.trial_count} trials finished'
            )

            if self.ball_is_at_start():
                self.ball_reset_observed = True

            self.try_start_next_trial()
        else:
            self.get_logger().error('Gazebo rejected the simulation reset')
            self.reset_requested = False
            self.waiting_for_reset = False

    def ball_is_at_start(self) -> bool:
        if self.latest_position is None:
            return False

        x, y, z = self.latest_position
        return (
            abs(x + 1.5) <= 0.15
            and abs(y) <= 0.15
            and z <= 0.20
        )

    def try_start_next_trial(self) -> None:
        if not (
            self.waiting_for_reset
            and self.reset_response_received
            and self.ball_reset_observed
        ):
            return

        if self.completed_trials >= self.trial_count:
            self.batch_complete = True
            self.get_logger().info(
                f'Automatic batch complete: {self.trial_count} trials'
            )
            return

        self.reset_trial_state()
        self.get_logger().info(
            f'Ready for trial '
            f'{self.completed_trials + 1}/{self.trial_count}'
        )

    def reset_trial_state(self) -> None:
        self.sequence_start = self.get_clock().now()
        self.shot_fired_ = False
        self.reset_requested = False
        self.pending_shot = None
        self.shot_origin = None
        self.last_attempt_time = None
        self.confirmed_shot_time = None
        self.launch_attempts = 0
        self.launch_failed = False
        self.trial_ending_published = False
        self.pending_shot_style = 'UNSET'
        self.waiting_for_reset = False
        self.reset_response_received = False
        self.ball_reset_observed = False

    def configure_shot_profile(self, shot: EntityWrench) -> None:
        profile = random.choices(
            population=(
                'TOP_LEFT',
                'TOP_RIGHT',
                'LOW_LEFT',
                'LOW_RIGHT',
                'POWER_CENTER',
            ),
            weights=(24, 24, 20, 20, 12),
            k=1
        )[0]
        self.pending_shot_style = profile

        shot.wrench.force.x = random.triangular(
            self.minimum_forward_force,
            self.maximum_forward_force,
            1850.0
        )

        if profile.endswith('LEFT'):
            lateral_sign = 1.0
        elif profile.endswith('RIGHT'):
            lateral_sign = -1.0
        else:
            lateral_sign = random.choice((-1.0, 1.0))

        if profile.startswith('TOP'):
            lateral_magnitude = random.triangular(
                190.0,
                self.maximum_lateral_force,
                285.0
            )
            vertical_force = random.triangular(
                650.0,
                self.maximum_vertical_force,
                850.0
            )
        elif profile.startswith('LOW'):
            lateral_magnitude = random.triangular(
                220.0,
                self.maximum_lateral_force,
                300.0
            )
            vertical_force = random.triangular(
                self.minimum_vertical_force,
                480.0,
                300.0
            )
        else:
            lateral_magnitude = random.triangular(0.0, 150.0, 60.0)
            vertical_force = random.triangular(350.0, 780.0, 500.0)
            shot.wrench.force.x = random.triangular(
                2100.0,
                self.maximum_forward_force,
                2500.0
            )

        shot.wrench.force.y = lateral_sign * lateral_magnitude
        shot.wrench.force.z = vertical_force


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BallLauncher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
