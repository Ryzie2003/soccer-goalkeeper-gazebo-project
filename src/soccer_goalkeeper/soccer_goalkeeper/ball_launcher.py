#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench
from ros_gz_interfaces.srv import ControlWorld


class BallLauncher(Node):
    def __init__(self) -> None:
        super().__init__('ball_launcher')

        self.publisher_ = self.create_publisher(
            EntityWrench,
            '/world/default/wrench',
            10
        )

        self.reset_client = self.create_client(
            ControlWorld,
            '/world/default/control'
        )

        self.timer_ = self.create_timer(1.0, self.launch_ball)
        self.reset_timer = self.create_timer(6.0, self.reset_ball)

        self.shot_fired_ = False
        self.reset_requested = False

    def launch_ball(self) -> None:
        if self.shot_fired_:
            return

        shot = EntityWrench()
        shot.entity.name = 'soccer_ball::ball_link'
        shot.entity.type = 3  # LINK
        shot.wrench.force.x = float(random.randint(1000, 2000))
        shot.wrench.force.y = float(random.randint(-150, 150))

        self.publisher_.publish(shot)
        self.shot_fired_ = True
        self.timer_.cancel()

        self.get_logger().info('Shot fired')

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
        self.reset_timer.cancel()
        self.get_logger().info('Simulation reset requested')

    def reset_finished(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Simulation reset failed: {error}')
            return

        if response.success:
            self.get_logger().info('Simulation reset completed')
        else:
            self.get_logger().error('Gazebo rejected the simulation reset')


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BallLauncher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
