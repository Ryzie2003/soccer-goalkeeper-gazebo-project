from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'soccer_goalkeeper'

setup(
    name=package_name,
    version='0.0.0',
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
            os.path.join('share', package_name, 'models', 'soccer_ball'),
            glob('models/soccer_ball/*')
        ),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='ryan@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_monitor = soccer_goalkeeper.odom_monitor:main',
            'goalkeeper_controller = soccer_goalkeeper.goalkeeper_controller:main',
            'ball_launcher = soccer_goalkeeper.ball_launcher:main',
            'ball_tracker = soccer_goalkeeper.ball_tracker:main',
        ],
    },
)
