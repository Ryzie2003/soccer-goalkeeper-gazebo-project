#!/usr/bin/env python3

import math
from collections import deque

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time


class CameraBallFusion(Node):
    OVERHEAD_CAMERA_HEIGHT = 6.0
    SIDE_CAMERA_X = -5.0
    SIDE_CAMERA_Z = 1.5
    BALL_RADIUS = 0.06
    MAXIMUM_HEIGHT = 1.5
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    HORIZONTAL_FOV = 1.2
    MAXIMUM_SIDE_OBSERVATION_AGE = 0.12
    FILTER_WINDOW = 5
    MAXIMUM_POSITION_JUMP = 0.8

    def __init__(self) -> None:
        super().__init__('camera_ball_fusion')

        self.side_observation: PointStamped | None = None
        self.x_history: deque[float] = deque(maxlen=self.FILTER_WINDOW)
        self.y_history: deque[float] = deque(maxlen=self.FILTER_WINDOW)
        self.z_history: deque[float] = deque(maxlen=self.FILTER_WINDOW)
        self.last_position: tuple[float, float, float] | None = None

        self.overhead_subscription = self.create_subscription(
            PointStamped,
            '/overhead_ball_observation',
            self.overhead_callback,
            10
        )
        self.side_subscription = self.create_subscription(
            PointStamped,
            '/side_camera_ball_pixel',
            self.side_callback,
            10
        )
        self.position_publisher = self.create_publisher(
            PointStamped,
            '/camera_ball_position',
            10
        )

        self.focal_length_pixels = (
            self.IMAGE_WIDTH
            / (2.0 * math.tan(self.HORIZONTAL_FOV / 2.0))
        )
        self.last_log_time: Time | None = None

    def side_callback(self, message: PointStamped) -> None:
        self.side_observation = message

    def overhead_callback(self, message: PointStamped) -> None:
        ground_x = message.point.x
        ground_y = message.point.y
        estimated_height = message.point.z
        used_side_camera = False

        if self.side_observation_is_current(message):
            side_observation = self.side_observation
            if side_observation is None:
                return

            estimated_height, corrected_x = self.side_camera_height(
                side_observation.point.y,
                ground_x
            )

            # Recalculate once because side-camera depth depends on x, while
            # overhead x itself needs height-based perspective correction.
            estimated_height, corrected_x = self.side_camera_height(
                side_observation.point.y,
                corrected_x
            )
            used_side_camera = True

        perspective_scale = (
            (self.OVERHEAD_CAMERA_HEIGHT - estimated_height)
            / (self.OVERHEAD_CAMERA_HEIGHT - self.BALL_RADIUS)
        )

        measured_x = ground_x * perspective_scale
        measured_y = ground_y * perspective_scale
        measured_z = estimated_height

        if self.measurement_is_plausible(
            measured_x,
            measured_y,
            measured_z
        ):
            self.x_history.append(measured_x)
            self.y_history.append(measured_y)
            self.z_history.append(measured_z)

        if not self.x_history:
            return

        filtered_x = self.median(self.x_history)
        filtered_y = self.median(self.y_history)
        filtered_z = self.median(self.z_history)
        self.last_position = (filtered_x, filtered_y, filtered_z)

        fused_position = PointStamped()
        fused_position.header = message.header
        fused_position.header.frame_id = 'world'
        fused_position.point.x = filtered_x
        fused_position.point.y = filtered_y
        fused_position.point.z = filtered_z
        self.position_publisher.publish(fused_position)

        current_time = self.get_clock().now()
        should_log = (
            self.last_log_time is None
            or (
                current_time - self.last_log_time
            ).nanoseconds >= 1_000_000_000
        )
        if should_log:
            source = 'overhead + side' if used_side_camera else 'overhead only'
            self.get_logger().info(
                f'Fused camera position ({source}): '
                f'x={fused_position.point.x:.2f}, '
                f'y={fused_position.point.y:.2f}, '
                f'z={fused_position.point.z:.2f}'
            )
            self.last_log_time = current_time

    def side_observation_is_current(
        self,
        overhead_message: PointStamped
    ) -> bool:
        if self.side_observation is None:
            return False

        overhead_time = self.stamp_seconds(overhead_message)
        side_time = self.stamp_seconds(self.side_observation)
        return (
            abs(overhead_time - side_time)
            <= self.MAXIMUM_SIDE_OBSERVATION_AGE
        )

    def side_camera_height(
        self,
        vertical_pixel: float,
        estimated_x: float
    ) -> tuple[float, float]:
        depth = max(0.1, estimated_x - self.SIDE_CAMERA_X)
        vertical_offset = (self.IMAGE_HEIGHT / 2.0) - vertical_pixel
        raw_height = (
            self.SIDE_CAMERA_Z
            + vertical_offset * depth / self.focal_length_pixels
        )
        height = max(
            self.BALL_RADIUS,
            min(self.MAXIMUM_HEIGHT, raw_height)
        )

        perspective_scale = (
            (self.OVERHEAD_CAMERA_HEIGHT - height)
            / (self.OVERHEAD_CAMERA_HEIGHT - self.BALL_RADIUS)
        )
        corrected_x = estimated_x * perspective_scale
        return height, corrected_x

    def measurement_is_plausible(
        self,
        x: float,
        y: float,
        z: float
    ) -> bool:
        if not (-2.5 <= x <= 3.8 and -2.2 <= y <= 2.2):
            return False
        if not (self.BALL_RADIUS <= z <= self.MAXIMUM_HEIGHT):
            return False
        if self.last_position is None:
            return True

        last_x, last_y, last_z = self.last_position
        jump = math.sqrt(
            (x - last_x) ** 2
            + (y - last_y) ** 2
            + (z - last_z) ** 2
        )
        return jump <= self.MAXIMUM_POSITION_JUMP

    @staticmethod
    def median(values) -> float:
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return float(ordered[middle])
        return float((ordered[middle - 1] + ordered[middle]) / 2.0)

    @staticmethod
    def stamp_seconds(message: PointStamped) -> float:
        return (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1e9
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CameraBallFusion()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
