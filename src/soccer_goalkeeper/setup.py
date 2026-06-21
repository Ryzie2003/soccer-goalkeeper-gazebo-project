import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'soccer_goalkeeper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')
        ),
        (
            os.path.join('share', package_name, 'models', 'soccer_ball'),
            glob('models/soccer_ball/*')
        ),
        (
            os.path.join('share', package_name, 'models', 'goalkeeper'),
            glob('models/goalkeeper/*')
        ),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan Zheng',
    maintainer_email='rzpersonal1@gmail.com',
    description=(
        'Camera-guided autonomous soccer goalkeeper for ROS 2 and Gazebo Sim'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_monitor = soccer_goalkeeper.odom_monitor:main',
            (
                'goalkeeper_controller = '
                'soccer_goalkeeper.goalkeeper_controller:main'
            ),
            'ball_launcher = soccer_goalkeeper.ball_launcher:main',
            'ball_tracker = soccer_goalkeeper.ball_tracker:main',
            (
                'camera_ball_detector = '
                'soccer_goalkeeper.camera_ball_detector:main'
            ),
            (
                'camera_ball_tracker = '
                'soccer_goalkeeper.camera_ball_tracker:main'
            ),
            (
                'side_camera_ball_detector = '
                'soccer_goalkeeper.side_camera_ball_detector:main'
            ),
            (
                'camera_ball_fusion = '
                'soccer_goalkeeper.camera_ball_fusion:main'
            ),
            (
                'goal_line_replay = '
                'soccer_goalkeeper.goal_line_replay:main'
            ),
            'referee = soccer_goalkeeper.referee:main',
        ],
    },
)
