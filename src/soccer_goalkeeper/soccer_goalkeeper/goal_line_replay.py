#!/usr/bin/env python3

from collections import deque
from dataclasses import dataclass
from typing import Any

import cv2

from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from sensor_msgs.msg import Image

from soccer_goalkeeper.vision import find_soccer_ball_contour

from std_msgs.msg import Empty, String

from tf2_msgs.msg import TFMessage


@dataclass
class ReplayFrame:
    image: Any
    time_seconds: float
    ball_position: tuple[float, float, float] | None


class GoalLineReplay(Node):
    """Buffer and replay annotated goal-line camera footage in RViz."""

    CAMERA_FPS = 30
    REPLAY_FPS = 15
    PRE_EVENT_SECONDS = 4.5
    REPLAY_PRE_EVENT_SECONDS = 1.5
    POST_EVENT_SECONDS = 0.5
    FINAL_FRAME_HOLD_SECONDS = 1.0
    GOAL_LINE_X = 2.7
    BALL_RADIUS = 0.06

    def __init__(self) -> None:
        super().__init__('goal_line_replay')

        self.bridge = CvBridge()
        self.latest_ball_position: tuple[float, float, float] | None = None
        self.frame_buffer: deque[ReplayFrame] = deque(
            maxlen=int(self.CAMERA_FPS * self.PRE_EVENT_SECONDS)
        )

        self.pending_decision: str | None = None
        self.event_time: float | None = None
        self.capture_frames: list[ReplayFrame] = []
        self.post_frames_remaining = 0

        self.playback_frames: list[ReplayFrame] = []
        self.review_queue: deque[
            tuple[str, float, list[ReplayFrame]]
        ] = deque()
        self.playback_index = 0
        self.playback_active = False
        self.final_frame_hold_ticks = 0
        self._playback_decision = 'NO_GOAL'

        self.image_subscription = self.create_subscription(
            Image,
            '/goal_line_camera/image',
            self.image_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            TFMessage,
            '/model/soccer_ball/pose',
            self.pose_callback,
            10
        )
        self.event_subscription = self.create_subscription(
            String,
            '/goal_line_event',
            self.event_callback,
            10
        )
        self.replay_publisher = self.create_publisher(
            Image,
            '/goal_line_replay',
            10
        )
        self.replay_complete_publisher = self.create_publisher(
            Empty,
            '/goal_line_replay_complete',
            10
        )

        self.playback_timer = self.create_timer(
            1.0 / self.REPLAY_FPS,
            self.playback_tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def pose_callback(self, message: TFMessage) -> None:
        if not message.transforms:
            return

        position = message.transforms[0].transform.translation
        self.latest_ball_position = (
            position.x,
            position.y,
            position.z
        )

    def image_callback(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8'
            )
        except CvBridgeError as error:
            self.get_logger().error(
                f'Goal-line image conversion failed: {error}'
            )
            return

        record = ReplayFrame(
            image=image.copy(),
            time_seconds=self.stamp_seconds(message),
            ball_position=self.latest_ball_position
        )
        self.frame_buffer.append(record)

        if self.pending_decision is not None:
            self.capture_frames.append(record)
            self.post_frames_remaining -= 1

            if self.post_frames_remaining <= 0:
                self.finish_capture()

        if not self.playback_active:
            status = (
                'VAR CAPTURING REVIEW'
                if self.pending_decision is not None
                else 'GOAL-LINE CAMERA - WAITING FOR SHOT'
            )
            self.publish_frame(
                self.annotate_frame(record, status, live=True)
            )

    def event_callback(self, message: String) -> None:
        if self.pending_decision is not None:
            self.get_logger().warning(
                'Ignoring goal-line event while another review is capturing'
            )
            return

        decision = message.data.strip().upper()
        if decision not in ('GOAL', 'NO_GOAL'):
            self.get_logger().warning(
                f'Ignoring unsupported goal-line decision: {message.data}'
            )
            return

        self.pending_decision = decision
        self.event_time = (
            self.frame_buffer[-1].time_seconds
            if self.frame_buffer
            else self.clock_seconds()
        )
        replay_start_time = (
            self.event_time - self.REPLAY_PRE_EVENT_SECONDS
        )
        self.capture_frames = [
            frame
            for frame in self.frame_buffer
            if frame.time_seconds >= replay_start_time
        ]
        self.post_frames_remaining = int(
            self.CAMERA_FPS * self.POST_EVENT_SECONDS
        )

        self.get_logger().info(
            f'Goal-line review triggered: {decision}; '
            f'buffered {len(self.capture_frames)} pre-event frames'
        )

    def playback_tick(self) -> None:
        if not self.playback_active:
            return

        if self.playback_index < len(self.playback_frames):
            record = self.playback_frames[self.playback_index]
            event_time = (
                self.event_time
                if self.event_time is not None
                else record.time_seconds
            )
            status = (
                'VAR CHECKING...'
                if record.time_seconds < event_time
                else self.decision_text()
            )
            self.publish_frame(
                self.annotate_frame(record, status, live=False)
            )
            self.playback_index += 1
            return

        if self.final_frame_hold_ticks > 0 and self.playback_frames:
            self.publish_frame(
                self.annotate_frame(
                    self.playback_frames[-1],
                    self.decision_text(),
                    live=False
                )
            )
            self.final_frame_hold_ticks -= 1
            return

        self.playback_active = False
        self.playback_frames = []
        self.replay_complete_publisher.publish(Empty())
        self.get_logger().info('Goal-line replay completed')

        if self.review_queue:
            decision, event_time, frames = self.review_queue.popleft()
            self.load_review(decision, event_time, frames)

    def annotate_frame(
        self,
        record: ReplayFrame,
        status: str,
        live: bool
    ) -> Any:
        image = record.image.copy()
        height, width = image.shape[:2]
        line_x = width // 2

        cv2.line(
            image,
            (line_x, 0),
            (line_x, height),
            (0, 255, 255),
            2
        )
        cv2.putText(
            image,
            'GOAL LINE',
            (line_x + 8, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        contour = find_soccer_ball_contour(image, minimum_area=1.5)
        if contour is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
            cv2.circle(
                image,
                (int(center_x), int(center_y)),
                max(3, int(radius)),
                (0, 0, 255),
                2
            )

        overlay_color = (
            (0, 220, 0)
            if 'GOAL' in status and 'NO GOAL' not in status
            else (0, 0, 255)
            if 'NO GOAL' in status
            else (0, 215, 255)
        )

        cv2.rectangle(
            image,
            (0, height - 82),
            (width, height),
            (15, 15, 15),
            -1
        )
        cv2.putText(
            image,
            'LIVE' if live else 'SLOW-MOTION REPLAY',
            (16, height - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (230, 230, 230),
            2,
            cv2.LINE_AA
        )
        cv2.putText(
            image,
            status,
            (16, height - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            overlay_color,
            2,
            cv2.LINE_AA
        )

        if record.ball_position is not None:
            center_x_world, center_y_world, center_z_world = (
                record.ball_position
            )
            trailing_edge = center_x_world - self.BALL_RADIUS
            crossed_distance = trailing_edge - self.GOAL_LINE_X
            cv2.putText(
                image,
                (
                    f'ball edge: {trailing_edge:.3f} m  '
                    f'line: {self.GOAL_LINE_X:.3f} m  '
                    f'delta: {crossed_distance:+.3f} m'
                ),
                (16, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )
            cv2.putText(
                image,
                (
                    f'y={center_y_world:+.2f} m  '
                    f'height={center_z_world:.2f} m'
                ),
                (16, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )

        return image

    def decision_text(self) -> str:
        decision = self.current_decision().replace('_', ' ')
        return f'DECISION: {decision}'

    def current_decision(self) -> str:
        return self._playback_decision

    def publish_frame(self, image: Any) -> None:
        try:
            message = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'Replay image conversion failed: {error}')
            return

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'goal_line_camera'
        self.replay_publisher.publish(message)

    def finish_capture(self) -> None:
        decision = self.pending_decision or 'NO_GOAL'
        event_time = self.event_time or self.clock_seconds()
        frames = self.capture_frames

        self.pending_decision = None
        self.capture_frames = []
        self.post_frames_remaining = 0

        if self.playback_active:
            self.review_queue.append((decision, event_time, frames))
            self.get_logger().info(
                f'Queued goal-line review: {decision}; '
                f'{len(self.review_queue)} review(s) waiting'
            )
            return

        self.load_review(decision, event_time, frames)

    def load_review(
        self,
        decision: str,
        event_time: float,
        frames: list[ReplayFrame]
    ) -> None:
        self._playback_decision = decision
        self.event_time = event_time
        self.playback_frames = frames
        self.playback_index = 0
        self.playback_active = bool(self.playback_frames)
        self.final_frame_hold_ticks = int(
            self.REPLAY_FPS * self.FINAL_FRAME_HOLD_SECONDS
        )

        self.get_logger().info(
            f'Publishing {len(self.playback_frames)} replay frames '
            f'at {self.REPLAY_FPS} FPS: {decision}'
        )

    @staticmethod
    def stamp_seconds(message: Image) -> float:
        return (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1e9
        )

    def clock_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)

    node = GoalLineReplay()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
