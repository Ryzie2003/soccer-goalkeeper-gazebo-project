# ROS 2 Soccer Goalkeeper

A ROS 2 Lyrical and Gazebo Sim project where a camera-guided goalkeeper
predicts soccer shots, moves across the goal, and performs jumping dives.

The controller uses simulated camera measurements. Gazebo ground truth is used
only by the referee and VAR system.

## Demo
![Watch the ROS 2 Soccer Goalkeeper Demo](https://img.youtube.com/vi/DX0d_M0N9cw/maxresdefault.jpg)  ([https://youtu.be/abc123XYZ](https://youtu.be/DX0d_M0N9cw)) 

## Features

- Custom soccer field, goal, net, ball, and player-style goalkeeper
- Overhead and side-camera ball detection
- Fused 3D position, velocity, and goal-line intercept prediction
- Closed-loop tracking with low and high diving actions
- Randomized corner, low, high, and power-center shots
- Automatic 30-shot trials with reset and cumulative statistics
- Goal-line referee and slow-motion VAR replay in RViz
- Single launch file for Gazebo, bridges, perception, and control

## Architecture

```text
overhead camera --\
                   > camera fusion -> 3D tracker -> goalkeeper controller
side camera ------/                         |
                                            v
                                  track / low dive / high dive

Gazebo ball pose -> referee -> goal-line event -> VAR replay -> RViz
```

The camera tracker publishes a predicted goal-line intercept containing:

```text
point.x = lateral position
point.y = ball height
point.z = seconds until arrival
```

Low dives use a short sideways launch with modest lift. High corner dives add
more vertical force and body roll. Center shots remain standing saves.

## Main Nodes

| Node | Purpose |
| --- | --- |
| `ball_launcher` | Generates randomized shots and manages trials |
| `camera_ball_detector` | Detects the ball from the overhead camera |
| `side_camera_ball_detector` | Measures the ball from the side camera |
| `camera_ball_fusion` | Produces a fused 3D ball position |
| `camera_ball_tracker` | Predicts where and when the ball reaches the goal |
| `goalkeeper_controller` | Tracks the prediction and selects dive actions |
| `referee` | Classifies goals and saves |
| `goal_line_replay` | Publishes annotated VAR replay footage to RViz |

## Build

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in each new terminal unless it is already loaded
from `~/.bashrc`.

## Run

Start the full simulation:

```bash
ros2 launch soccer_goalkeeper soccer_demo.launch.py
```

Launch with RViz:

```bash
ros2 launch soccer_goalkeeper soccer_demo.launch.py launch_rviz:=true
```

In another terminal, run the default 30-shot trial:

```bash
ros2 run soccer_goalkeeper ball_launcher --ros-args -p use_sim_time:=true
```

Use a smaller batch:

```bash
ros2 run soccer_goalkeeper ball_launcher --ros-args \
  -p use_sim_time:=true -p trial_count:=10
```

The launcher verifies each shot, resets the world, waits for the VAR replay to
finish, and then starts the next trial.

## Useful Topics

| Topic | Purpose |
| --- | --- |
| `/camera_ball_position` | Fused camera-estimated ball position |
| `/camera_predicted_intercept_3d` | Predicted lateral intercept, height, and arrival time |
| `/goalkeeper/action` | Current tracking or dive action |
| `/shot_result` | Latest `GOAL` or `SAVE` result |
| `/shot_statistics` | Cumulative save statistics |
| `/goal_line_replay` | Annotated live and replay image for RViz |

## Repository Layout

```text
src/soccer_goalkeeper/
├── launch/              # Demo launch files
├── models/              # Gazebo models
├── rviz/                # RViz configuration
├── soccer_goalkeeper/   # ROS 2 Python nodes
├── worlds/              # Soccer simulation world
├── package.xml
└── setup.py
```

## Limitations

- Camera calibration is specific to the simulated camera poses.
- Prediction assumes approximately ballistic vertical motion and steady
  horizontal velocity.
- Dive forces and thresholds are tuned for this simulated goalkeeper.
