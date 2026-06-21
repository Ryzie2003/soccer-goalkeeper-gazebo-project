#!/usr/bin/env python3

import cv2

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image

from soccer_goalkeeper.vision import find_soccer_ball_contour


class SideCameraBallDetector(Node):
    """Detect the ball in the side camera image."""

    def __init__(self) -> None:
        super().__init__('side_camera_ball_detector')

        self.bridge = CvBridge()
        self.minimum_area = 1.5
        self.last_log_time: Time | None = None

        self.image_subscription = self.create_subscription(
            Image,
            '/side_camera/image',
            self.image_callback,
            10
        )
        self.pixel_publisher = self.create_publisher(
            PointStamped,
            '/side_camera_ball_pixel',
            10
        )

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
        if largest_contour is None:
            return

        area = cv2.contourArea(largest_contour)
        if area < self.minimum_area:
            return

        moments = cv2.moments(largest_contour)
        if moments['m00'] == 0.0:
            return

        observation = PointStamped()
        observation.header = message.header
        observation.header.frame_id = 'side_camera'
        observation.point.x = moments['m10'] / moments['m00']
        observation.point.y = moments['m01'] / moments['m00']
        observation.point.z = area
        self.pixel_publisher.publish(observation)

        current_time = self.get_clock().now()
        should_log = (
            self.last_log_time is None
            or (
                current_time - self.last_log_time
            ).nanoseconds >= 1_000_000_000
        )
        if should_log:
            self.get_logger().info(
                'Side camera detected ball at pixel '
                f'({observation.point.x:.1f}, {observation.point.y:.1f})'
            )
            self.last_log_time = current_time


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SideCameraBallDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
