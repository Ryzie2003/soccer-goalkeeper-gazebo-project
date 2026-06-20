import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('soccer_goalkeeper')

    world_path = os.path.join(
        package_share,
        'worlds',
        'soccer_field.sdf'
    )
    rviz_config = os.path.join(
        package_share,
        'rviz',
        'goal_line_replay.rviz'
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
            (
                '/goalkeeper/left_arm_cmd'
                '@std_msgs/msg/Float64'
                ']gz.msgs.Double'
            ),
            (
                '/goalkeeper/right_arm_cmd'
                '@std_msgs/msg/Float64'
                ']gz.msgs.Double'
            ),
        ],
        output='screen'
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/field_camera/image'],
        output='screen'
    )

    side_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/side_camera/image'],
        output='screen'
    )

    goal_line_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/goal_line_camera/image'],
        output='screen'
    )

    camera_ball_detector = Node(
        package='soccer_goalkeeper',
        executable='camera_ball_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    side_camera_ball_detector = Node(
        package='soccer_goalkeeper',
        executable='side_camera_ball_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    camera_ball_fusion = Node(
        package='soccer_goalkeeper',
        executable='camera_ball_fusion',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    camera_ball_tracker = Node(
        package='soccer_goalkeeper',
        executable='camera_ball_tracker',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ball_tracker = Node(
        package='soccer_goalkeeper',
        executable='ball_tracker',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    goalkeeper_controller = Node(
        package='soccer_goalkeeper',
        executable='goalkeeper_controller',
        parameters=[{
            'use_sim_time': True,
            'prediction_topic': '/camera_predicted_intercept_3d',
        }],
        output='screen'
    )

    referee = Node(
        package='soccer_goalkeeper',
        executable='referee',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    goal_line_replay = Node(
        package='soccer_goalkeeper',
        executable='goal_line_replay',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Open RViz on the goal-line replay topic'
        ),
        SetEnvironmentVariable(
            'GZ_PARTITION',
            'soccer_goalkeeper'
        ),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            (
                os.path.join(package_share, 'models')
                + ':'
                + '/opt/ros/lyrical/share/turtlebot3_gazebo/models'
            )
        ),
        SetEnvironmentVariable(
            'GALLIUM_DRIVER',
            'd3d12'
        ),
        SetEnvironmentVariable(
            'MESA_D3D12_DEFAULT_ADAPTER_NAME',
            'NVIDIA'
        ),
        SetEnvironmentVariable(
            'QT_QPA_PLATFORM',
            'xcb'
        ),
        gazebo,
        turtlebot_bridge,
        ball_bridge,
        image_bridge,
        side_image_bridge,
        goal_line_image_bridge,
        camera_ball_detector,
        side_camera_ball_detector,
        camera_ball_fusion,
        camera_ball_tracker,
        ball_tracker,
        goalkeeper_controller,
        referee,
        goal_line_replay,
        rviz,
    ])
