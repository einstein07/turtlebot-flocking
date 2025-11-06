# TurtleBot Flocking

## Overview

`turtlebot_flocking` is a ROS 2 (Humble) package that ports an ARGoS-style flocking controller to TurtleBot3 robots. It replaces camera and LED based neighbour detection with LiDAR-derived interactions, and combines a Lennard–Jones style cohesion/repulsion term with a goal-directed attraction vector. The package is meant to run alongside the `argos3_ros2_bridge`, allowing multiple simulated TurtleBots to flock toward a nominated target point while maintaining separation.

Key features:
- LiDAR-only neighbour perception via the standard `sensor_msgs/msg/LaserScan`, using a configurable interaction range.
- Goal attraction driven by standard `nav_msgs/msg/Odometry` pose data.
- Differential-drive wheel speed calculation compatible with TurtleBot3 kinematics.
- Launch scripts for ARGoS simulation plus ROS 2 controller deployment.

## Package Layout

```
turtlebot_flocking/
  config/config.yaml           # Example parameter set for controllers.
  include/turtlebot_flocking/  # Controller headers.
  launch/                      # ARGoS world and helper launch scripts.
  src/flocking.cpp             # Main controller implementation.
  CMakeLists.txt, package.xml  # Build and metadata.
```

## External Interfaces

The controller subscribes to:

- `sensor_msgs/msg/LaserScan` on the configured scan topic (`/scan` by default). Each range reading is treated as a polar vector using the reported bearing.
- `nav_msgs/msg/Odometry` on the configured odometry topic (`/odom` by default). Only planar position and yaw are used.

## Building

```bash
cd ~/ros2_dev
source /opt/ros/humble/setup.bash
colcon build --packages-select argos3_ros2_bridge turtlebot_flocking
source install/setup.bash
```

If you already have a workspace overlay, source the lower layers before building.

## Parameters

Parameters can be supplied via YAML (see `config/config.yaml`) or set on the node directly.

| Namespace            | Parameter                              | Description                                                                                          | Default          |
|---------------------|----------------------------------------|------------------------------------------------------------------------------------------------------|------------------|
| `wheel_turning`     | `hard_turn_on_angle_threshold` (deg)   | Switch to hard turn above this heading error. Converted to radians internally.                       | `0.5`            |
|                     | `soft_turn_on_angle_threshold` (deg)   | Switch down to soft turn once heading error drops below this value.                                  | `0.3`            |
|                     | `no_turn_angle_threshold` (deg)        | Enter straight motion below this error.                                                              | `0.1`            |
|                     | `max_speed` (rad/s)                    | Wheel speed limit.                                                                                    | `1.0`            |
| `flocking`          | `target_distance` (m)                  | Preferred inter-robot spacing.                                                                       | `1.0`            |
|                     | `gain`                                 | Gain on the generalized Lennard–Jones potential.                                                     | `1.0`            |
|                     | `exponent`                             | Potential exponent.                                                                                  | `2.0`            |
|                     | `interaction_range` (m)                | Ignore LiDAR readings beyond this distance. Defaults to `interaction_cutoff * target_distance`.      | `1.8`            |
| (root namespace)    | `wheel_separation` (m)                 | Track width used to convert wheel speeds to `cmd_vel`.                                               | `0.14`           |
|                     | `wheel_radius` (m)                     | Wheel radius.                                                                                        | `0.029112741`    |
|                     | `goal.x`, `goal.y` (m)                 | Target point in the world frame.                                                                     | `0.0`, `0.0`     |
|                     | `goal.gain`                            | Linear gain toward the goal. Clamped to non-negative values.                                         | `1.0`            |
|                     | `lidar_topic`                          | Optional LaserScan topic override; if unset, `<namespace>/scan` is used.                             | derived          |
|                     | `odom_topic`                           | Optional Odometry topic override; if unset, `<namespace>/odom` is used.                              | derived          |

## Running the Simulation

1. Bridge the ARGoS world into ROS 2:
   ```bash
   cd ~/ros2_dev/src/turtlebot_flocking/launch
   ./launch-argos-bridge.sh
   ```

2. In a new terminal (with the workspace sourced), start the TurtleBot controllers:
   ```bash
   cd ~/ros2_dev/src/turtlebot_flocking/launch
   ./launch-controllers.sh
   ```

The script instantiates one controller node per robot namespace defined in `config/config.yaml`, remapping LiDAR and odometry topics accordingly.

3. Observe the flock behaviour in the ARGoS visualiser or inspect ROS 2 topics (`ros2 topic list`, `ros2 topic echo /robot_1/cmd_vel`, etc.).

## Visualization and Debugging

- View graph connections:
  ```bash
  ros2 run rqt_graph rqt_graph
  ```
- Inspect LiDAR data:
  ```bash
  ros2 topic echo /robot_1/scan
  ```
- Inspect controller output:
  ```bash
  ros2 topic echo /robot_1/cmd_vel
  ```
- Inspect odometry feed:
  ```bash
  ros2 topic echo /robot_1/odom
  ```

## Extending

- To experiment with different flocking potentials, modify `turtlebot_flocking/src/flocking.cpp` inside `FlockingController::lidarFlockingVector()`.
- Additional sensing modalities (e.g., shared poses) can be fused by adding new vector terms prior to `setWheelSpeedsFromVector`.
- To deploy on physical TurtleBots, replace the ARGoS bridge topics with real `sensor_msgs/msg/LaserScan` data (after converting to the `LidarList` structure or adapting the controller).

## License

Add your chosen license text to `package.xml` and include a LICENSE file at the package root to remove the ament warnings during builds.
