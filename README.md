# ROS 2 Soccer Goalkeeper

A ROS 2 Lyrical and Gazebo Sim project in which a TurtleBot3 goalkeeper
estimates where a moving ball will cross the goal line and drives to intercept
it.

The current implementation uses Gazebo ground-truth ball poses as a working
tracking baseline. Camera-based perception is planned as the next major
upgrade.

## Current Features

- Custom Gazebo soccer field, goal, net, and physical ball
- TurtleBot3 Burger goalkeeper positioned along the goal line
- Randomized forward and sideways shot forces
- Ball position and velocity estimation
- Linear goal-line intersection prediction
- Closed-loop proportional goalkeeper control
- Automatic reset of the ball, goalkeeper, velocities, and simulation time
- One-command launch for Gazebo, bridges, tracking, and control

## System Architecture

```text
ball_launcher
    |
    | /world/default/wrench
    v
Gazebo ball physics
    |
    | /model/soccer_ball/pose
    v
ball_tracker
    |
    | /predicted_intercept
    v
goalkeeper_controller <--- /odom
    |
    | /cmd_vel
    v
TurtleBot3 goalkeeper
```

`ros_gz_bridge` translates messages between ROS 2 and Gazebo Transport.

## ROS 2 Nodes

| Node | Responsibility |
| --- | --- |
| `ball_launcher` | Applies a randomized force to the ball and resets the simulation after each trial |
| `ball_tracker` | Estimates ball velocity and predicts its goal-line crossing position |
| `goalkeeper_controller` | Converts the predicted intercept into TurtleBot velocity commands |
| `odom_monitor` | Optional debugging node that prints goalkeeper odometry |

## Important Interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/model/soccer_ball/pose` | `tf2_msgs/msg/TFMessage` | Ball world pose from Gazebo |
| `/predicted_intercept` | `std_msgs/msg/Float64` | Predicted ball crossing position along the goal |
| `/odom` | `nav_msgs/msg/Odometry` | Goalkeeper position |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Goalkeeper motion command |
| `/world/default/wrench` | `ros_gz_interfaces/msg/EntityWrench` | Ball shot command |
| `/world/default/control` | `ros_gz_interfaces/srv/ControlWorld` | Simulation reset |

## Build

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in each new terminal unless it is already loaded
from `~/.bashrc`.

## Run the Demo

Start Gazebo, both bridges, the ball tracker, and the goalkeeper controller:

```bash
ros2 launch soccer_goalkeeper soccer_demo.launch.py
```

In another terminal, begin one shot:

```bash
ros2 run soccer_goalkeeper ball_launcher
```

The launcher fires after one second and resets the simulation after six
seconds.

## Current Prediction Model

The tracker estimates velocity from consecutive pose samples:

```text
velocity = change in position / elapsed time
```

It then assumes approximately constant velocity:

```text
time to goal = remaining x distance / x velocity
predicted y  = current y + y velocity * time to goal
```

The controller uses the predicted `y` position as its target along the goal
line.

## Limitations

- Ball tracking currently uses perfect Gazebo pose data rather than camera
  detections.
- The trajectory model assumes constant velocity and does not explicitly
  model acceleration or measurement uncertainty.
- The TurtleBot still looks like a mobile robot rather than a soccer player.
- Goal, save, and miss statistics are not yet recorded.

## Roadmap

- Add repeatable trials and goal/save/miss metrics
- Add a simulated camera and ROS image stream
- Detect and track the ball using OpenCV
- Replace ground-truth tracking with camera-derived field coordinates
- Keep ground-truth tracking as an optional debugging baseline
- Customize or replace the goalkeeper model to resemble a soccer player
- Add launch parameters, documentation, tests, and a polished demo video

## Repository Layout

```text
src/soccer_goalkeeper/
├── launch/                  # One-command demo launch
├── models/                  # Standalone Gazebo models
├── soccer_goalkeeper/       # ROS 2 Python nodes
├── worlds/                  # Custom soccer simulation
├── package.xml
└── setup.py
```

Generated `build/`, `install/`, and `log/` directories are intentionally
excluded from Git.
