#!/usr/bin/env python3

from collections import deque

import cv2

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PointStamped

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image

from soccer_goalkeeper.vision import find_soccer_ball_contour


class CameraBallDetector(Node):
    CAMERA_HEIGHT = 6.0
    BALL_RADIUS = 0.06
    MAXIMUM_ESTIMATED_HEIGHT = 1.5
    REFERENCE_SAMPLE_COUNT = 15

    def __init__(self) -> None:
        super().__init__('camera_ball_detector')

        self.bridge = CvBridge()
        self.minimum_area = 1.5
        self.reference_radius_samples: list[float] = []
        self.reference_pixel_radius: float | None = None
        self.height_history: deque[float] = deque(maxlen=5)

        self.image_subscription = self.create_subscription(
            Image,
            '/field_camera/image',
            self.image_callback,
            10
        )
        self.pixel_publisher = self.create_publisher(
            PointStamped,
            '/camera_ball_pixel',
            10
        )
        self.field_position_publisher = self.create_publisher(
            PointStamped,
            '/overhead_ball_observation',
            10
        )
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/camera_ball_debug',
            10
        )

        self.last_detection_time: Time | None = None

    def image_callback(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8'
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Image conversion failed: {error}')
            return

        largest_contour = find_soccer_ball_contour(
            image,
            self.minimum_area
        )

        if largest_contour is not None:
            area = cv2.contourArea(largest_contour)

            if area >= self.minimum_area:
                moments = cv2.moments(largest_contour)

                if moments['m00'] != 0.0:
                    center_x = moments['m10'] / moments['m00']
                    center_y = moments['m01'] / moments['m00']
                    _, measured_radius = cv2.minEnclosingCircle(
                        largest_contour
                    )

                    detection = PointStamped()
                    detection.header = message.header
                    detection.header.frame_id = 'field_camera'
                    detection.point.x = center_x
                    detection.point.y = center_y
                    detection.point.z = area
                    self.pixel_publisher.publish(detection)

                    field_position = PointStamped()
                    field_position.header = message.header
                    field_position.header.frame_id = 'world'

                    image_height, image_width = image.shape[:2]
                    start_pixel_y = (89.0 / 120.0) * image_height
                    goal_pixel_distance = (81.5 / 120.0) * image_height

                    ground_x = (
                        -1.5
                        + (start_pixel_y - center_y)
                        * (4.2 / goal_pixel_distance)
                    )
                    ground_y = (
                        ((image_width / 2.0) - center_x)
                        * (4.0 / image_width)
                    )

                    self.update_reference_radius(
                        measured_radius,
                        ground_x,
                        ground_y
                    )
                    estimated_height = self.estimate_height(
                        measured_radius,
                        image_width
                    )

                    # Publish the ground-plane projection and monocular height
                    # estimate separately. The fusion node combines this with
                    # the side camera and performs perspective correction.
                    field_position.point.x = ground_x
                    field_position.point.y = ground_y
                    field_position.point.z = estimated_height
                    self.field_position_publisher.publish(field_position)

                    center = (int(round(center_x)), int(round(center_y)))
                    radius = max(
                        3,
                        int(round(np.sqrt(area / np.pi)))
                    )
                    cv2.circle(image, center, radius, (0, 255, 0), 1)
                    cv2.circle(image, center, 2, (255, 0, 0), -1)

                    current_time = self.get_clock().now()
                    should_log = (
                        self.last_detection_time is None
                        or (
                            current_time - self.last_detection_time
                        ).nanoseconds >= 1_000_000_000
                    )
                    if should_log:
                        self.get_logger().info(
                            'Ball detected at pixel '
                            f'({center_x:.1f}, {center_y:.1f}), '
                            'overhead ground projection '
                            f'({field_position.point.x:.2f}, '
                            f'{field_position.point.y:.2f}, '
                            f'{field_position.point.z:.2f}), '
                            f'area={area:.1f}'
                        )
                        self.last_detection_time = current_time

        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                image,
                encoding='bgr8'
            )
            debug_message.header = message.header
            self.debug_image_publisher.publish(debug_message)
        except CvBridgeError as error:
            self.get_logger().error(
                f'Debug image conversion failed: {error}'
            )

    def update_reference_radius(
        self,
        measured_radius: float,
        ground_x: float,
        ground_y: float
    ) -> None:
        if self.reference_pixel_radius is not None:
            return

        # The launcher waits before shooting, so the first frames should show
        # the resting ball near its known starting position.
        ball_is_near_start = (
            abs(ground_x + 1.5) < 0.25
            and abs(ground_y) < 0.20
        )
        if not ball_is_near_start:
            return

        self.reference_radius_samples.append(measured_radius)
        if len(self.reference_radius_samples) >= self.REFERENCE_SAMPLE_COUNT:
            self.reference_pixel_radius = float(
                np.median(self.reference_radius_samples)
            )
            self.get_logger().info(
                'Camera depth calibration complete: resting ball radius '
                f'{self.reference_pixel_radius:.2f} pixels'
            )

    def estimate_height(
        self,
        measured_radius: float,
        image_width: int
    ) -> float:
        if measured_radius <= 0.0:
            return self.BALL_RADIUS

        if self.reference_pixel_radius is not None:
            camera_distance = (
                (self.CAMERA_HEIGHT - self.BALL_RADIUS)
                * self.reference_pixel_radius
                / measured_radius
            )
            raw_height = self.CAMERA_HEIGHT - camera_distance
        else:
            # Fallback until resting-ball calibration is complete.
            horizontal_fov = 1.2
            focal_length_pixels = (
                image_width
                / (2.0 * np.tan(horizontal_fov / 2.0))
            )
            raw_height = (
                self.CAMERA_HEIGHT
                - focal_length_pixels
                * self.BALL_RADIUS
                / measured_radius
            )

        bounded_height = max(
            self.BALL_RADIUS,
            min(self.MAXIMUM_ESTIMATED_HEIGHT, raw_height)
        )
        self.height_history.append(bounded_height)
        return float(np.median(self.height_history))


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CameraBallDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
