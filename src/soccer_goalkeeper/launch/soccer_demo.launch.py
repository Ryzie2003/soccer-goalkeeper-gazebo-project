import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('soccer_goalkeeper')

    world_path = os.path.join(
        package_share,
        'worlds',
        'soccer_field.sdf'
    )

    turtlebot_share = get_package_share_directory('turtlebot3_gazebo')
    bridge_config = os.path.join(
        turtlebot_share,
        'params',
        'turtlebot3_burger_bridge.yaml'
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    turtlebot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config
        }],
        output='screen'
    )

    ball_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            (
                '/world/default/wrench'
                '@ros_gz_interfaces/msg/EntityWrench'
                ']gz.msgs.EntityWrench'
            ),
            (
                '/world/default/control'
                '@ros_gz_interfaces/srv/ControlWorld'
            ),
            (
                '/model/soccer_ball/pose'
                '@tf2_msgs/msg/TFMessage'
                '[gz.msgs.Pose_V'
            ),
        ],
        output='screen'
    )

    ball_tracker = Node(
        package='soccer_goalkeeper',
        executable='ball_tracker',
        output='screen'
    )

    goalkeeper_controller = Node(
        package='soccer_goalkeeper',
        executable='goalkeeper_controller',
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_PARTITION',
            'soccer_goalkeeper'
        ),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/opt/ros/lyrical/share/turtlebot3_gazebo/models'
        ),
        SetEnvironmentVariable(
            'LIBGL_ALWAYS_SOFTWARE',
            '1'
        ),
        SetEnvironmentVariable(
            'GALLIUM_DRIVER',
            'llvmpipe'
        ),
        SetEnvironmentVariable(
            'QT_QPA_PLATFORM',
            'xcb'
        ),
        gazebo,
        turtlebot_bridge,
        ball_bridge,
        ball_tracker,
        goalkeeper_controller,
    ])
