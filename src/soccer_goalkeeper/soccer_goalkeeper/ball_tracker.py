#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64


class BallTracker(Node):
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

        self.previous_x = None
        self.previous_y = None
        self.previous_time = None

    def pose_callback(self, message: TFMessage) -> None:
        if not message.transforms:
            return

        position = message.transforms[0].transform.translation
        
        #initialize a clock to track elapsed time to calculate velocity
        current_time = self.get_clock().now()

        if self.previous_time is not None:
            elapsed = (current_time - self.previous_time).nanoseconds / 1e9

            if elapsed < 0.0:
                center_target = Float64()
                center_target.data = 0.0
                self.prediction_publisher.publish(center_target)

                self.previous_x = None
                self.previous_y = None
                self.previous_time = None

                self.get_logger().info(
                    'Simulation reset detected; goalkeeper target centered'
                )
            elif elapsed > 0.0:
                velocity_x = (position.x - self.previous_x) / elapsed
                velocity_y = (position.y - self.previous_y) / elapsed
            
                goal_x = 2.7

                #ball is traveling towards the goal
                if velocity_x > 0.05 and position.x < goal_x:
                    time_to_goal = (goal_x - position.x) / velocity_x
                    predicted_y = position.y + velocity_y * time_to_goal

                    if abs(predicted_y) < 0.001:
                        predicted_y = 0.0
                        
                    prediction = Float64()
                    prediction.data = predicted_y
                    self.prediction_publisher.publish(prediction)

                    self.get_logger().info(
                        f'Predicted crossing: y={predicted_y:.2f} '
                        f'in {time_to_goal:.2f} seconds'
                    )

                if (velocity_x > 0.0 or velocity_y > 0.0) and position.x < goal_x:
                    self.get_logger().info(
                        f'Ball velocity: vx = {velocity_x:.2f},'
                        f'vy = {velocity_y:.2f} m/s'
                    )

        self.previous_x = position.x                             
        self.previous_y = position.y                             
        self.previous_time = current_time



def main(args=None) -> None:
    rclpy.init(args=args)

    node = BallTracker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
