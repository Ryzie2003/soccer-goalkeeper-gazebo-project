# ROS 2 Soccer Goalkeeper

A ROS 2 Lyrical and Gazebo Sim project in which a mobile goalkeeper uses
fused overhead and side cameras to estimate where a moving ball will cross
the goal line and drives to intercept it.

Gazebo ground truth is reserved for debugging and officiating. It does not
control the goalkeeper.

## Current Features

- Custom Gazebo soccer field, enlarged goal, physical net, and soccer ball
- Soccer-player goalkeeper driven by hidden TurtleBot3 differential-drive
  physics
- Named upper-corner, lower-corner, and power-center shot profiles with varied
  speed and airborne height
- Fused overhead and side-camera 3D ball estimation
- Median-filtered measurements with outlier rejection
- Alpha-beta 3D state estimation and ballistic goal-line prediction
- Closed-loop lateral control with fixed physical arms, jumping, and diving
- Geometry-based goal and save classification with cumulative statistics
- Dedicated goal-line camera and buffered slow-motion VAR-style replay
- RViz replay with goal-line overlay, tracked ball, measurements, and decision
- Automatic reset of the ball, goalkeeper, velocities, and simulation time
- One-command launch for Gazebo, bridges, perception, control, and RViz

## System Architecture

```text
overhead camera ----\
                     > filtered camera fusion -> 3D state estimator
side camera --------/                            |
                                                 | intercept y, z, arrival time
                                                 v
                                          action controller
                                             /          \
                                      jump wrench    dive motion

goal-line camera -> replay buffer -> /goal_line_replay -> RViz
                          ^
                          | /goal_line_event
                    ground-truth referee
```

`ros_gz_bridge` translates messages between ROS 2 and Gazebo Transport.
Ground truth enters only the referee and comparison tracker.

## ROS 2 Nodes

| Node | Responsibility |
| --- | --- |
| `ball_launcher` | Applies a randomized force to the ball and resets the simulation after each trial |
| `ball_tracker` | Estimates ball velocity and predicts its goal-line crossing position |
| `camera_ball_detector` | Detects the black-and-white ball in the overhead camera image |
| `side_camera_ball_detector` | Detects the ball in the side image for height estimation |
| `camera_ball_fusion` | Combines overhead ground projection with side-camera height |
| `camera_ball_tracker` | Estimates velocity from camera-derived field positions and predicts the goal-line intercept |
| `goalkeeper_controller` | Selects tracking, jump, and dive actions from predicted lateral position, height, and arrival time |
| `referee` | Uses ground-truth goal-line geometry to classify each trial as a goal or save and publishes cumulative statistics |
| `goal_line_replay` | Buffers goal-line footage and publishes annotated half-speed reviews to RViz |
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
| `/shot_result` | `std_msgs/msg/String` | Latest `GOAL` or `SAVE` result |
| `/shot_statistics` | `std_msgs/msg/String` | Shot count, goals, saves, and save rate |
| `/camera_ball_pixel` | `geometry_msgs/msg/PointStamped` | Detected ball pixel center; `z` stores contour area |
| `/camera_ball_position` | `geometry_msgs/msg/PointStamped` | Camera-estimated ball position in field metres |
| `/camera_predicted_intercept` | `std_msgs/msg/Float64` | Goal-line prediction derived only from camera measurements |
| `/camera_predicted_intercept_3d` | `geometry_msgs/msg/PointStamped` | Predicted lateral intercept, height, and seconds until arrival |
| `/goalkeeper/action` | `std_msgs/msg/String` | Current `TRACK`, `JUMP`, `DIVE_LEFT`, `DIVE_RIGHT`, or `RECOVER` action |
| `/camera_ball_debug` | `sensor_msgs/msg/Image` | Camera image annotated with the detected ball |
| `/goal_line_camera/image` | `sensor_msgs/msg/Image` | Raw dedicated goal-line camera |
| `/goal_line_event` | `std_msgs/msg/String` | Official `GOAL` or `NO_GOAL` replay trigger |
| `/goal_line_replay` | `sensor_msgs/msg/Image` | Live and slow-motion annotated RViz feed |

## Build

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in each new terminal unless it is already loaded
from `~/.bashrc`.

## Run the Demo

Start Gazebo and all ROS nodes:

```bash
ros2 launch soccer_goalkeeper soccer_demo.launch.py
```

Start the complete demo and open RViz directly on the replay:

```bash
ros2 launch soccer_goalkeeper soccer_demo.launch.py launch_rviz:=true
```

In another terminal, begin the default 30-trial batch:

```bash
ros2 run soccer_goalkeeper ball_launcher --ros-args -p use_sim_time:=true
```

The launcher verifies that Gazebo applied every shot, announces the end of
each trial one second before reset, confirms the ball returned to its starting
pose, and automatically continues until all 30 trials are complete. Goal-line
reviews run independently and queue if necessary. Override the batch size
with:

```bash
ros2 run soccer_goalkeeper ball_launcher --ros-args \
  -p use_sim_time:=true -p trial_count:=10
```

## Goal-Line Replay

The goal-line camera is positioned so the `x=2.7 m` goal plane appears at the
center of its image. The replay node continuously retains 4.5 seconds of
camera footage. When the referee publishes `/goal_line_event`, it adds 0.6
seconds of post-event footage and publishes the sequence at 15 FPS, half the
camera capture rate.

The referee applies the whole-ball rule. For a ball moving into the goal, its
trailing edge must cross the plane:

```text
ball_center_x - ball_radius >= goal_line_x
```

The crossing must also be between the posts and below the crossbar. The replay
shows the tracked ball, goal line, ball-edge coordinate, signed distance from
the line, height, and final `GOAL` or `NO GOAL` decision.

## Current Prediction Model

The camera pipeline detects the black-and-white ball, rejects implausible
measurements, median-filters the fused views, and uses an alpha-beta filter to
estimate 3D position and velocity.

```text
velocity = change in position / elapsed time
```

Horizontal crossing uses the filtered velocity:

```text
time to goal = remaining x distance / x velocity
predicted y  = current y + y velocity * time to goal
```

Vertical crossing uses a ballistic model with gravity. The action controller
uses predicted lateral position, crossing height, and arrival time to choose
between normal tracking, a jump, or a dive. Low dives combine wheel motion,
lateral impulse, and body roll torque while remaining near the ground. High
dives add vertical impulse so the goalkeeper launches diagonally toward an
upper corner. The arms remain fixed and provide physical blocking coverage
without separate joint control.

## Limitations

- Camera calibration is tied to the configured simulated camera poses.
- Horizontal prediction assumes approximately constant velocity between
  bounces.
- Jump and dive thresholds are tuned for the simulated player mass and goal.
- Replay decisions use Gazebo ground truth as the authoritative simulated
  goal-line sensor; camera footage provides the visual evidence.

## Roadmap

- Add prediction-error and reaction-latency metrics
- Add contact sensors for explicit glove and body saves
- Record a polished demo video and representative trial statistics

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
