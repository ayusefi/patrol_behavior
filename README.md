# patrol_behavior

A ROS 2 C++ package for autonomous patrol navigation using the Nav2 stack. This node loads a sequence of waypoints from a YAML file and sends them one by one to the Nav2 NavigateToPose action server, enabling a robot to patrol a custom route.

## Features
- Loads waypoints from a YAML file
- Sends waypoints sequentially to the Nav2 action server
- Waits for each goal to succeed before sending the next
- Loops the patrol route indefinitely

## Requirements
- ROS 2 (tested with Humble)
- [bcr_bot robot simulation](https://github.com/blackcoffeerobotics/bcr_bot) (or any robot running Nav2)
- Nav2 stack
- yaml-cpp

## Installation
Clone this package into your ROS 2 workspace `src` directory:

```bash
cd ~/workspaces/career_sprint/ros2_ws/src
# If not already present:
git clone <this_repo_url> patrol_behavior
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select patrol_behavior
```

## Usage
1. **Launch your robot and Nav2 stack** (e.g., using [bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot)).
2. **Source your workspace:**
   ```bash
   source install/setup.bash
   ```
3. **Edit or create your waypoints YAML file:**
   Example:
   ```yaml
   waypoints:
     - frame_id: "map"
       x: 1.0
       y: 1.0
       z: 0.0
       qx: 0.0
       qy: 0.0
       qz: 0.0
       qw: 1.0
     # ... more waypoints ...
   ```
4. **Run the patrol node:**
   ```bash
   ros2 run patrol_behavior patrol_node --ros-args -p waypoints_yaml:=/absolute/path/to/waypoints.yaml
   ```

## Parameters
- `waypoints_yaml` (string): Path to the YAML file containing the waypoints. Default: `waypoints.yaml` in the package directory.

## Example YAML
```
waypoints:
  - frame_id: "map"
    x: 1.0
    y: 1.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0
  - frame_id: "map"
    x: 2.0
    y: 1.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.7071
    qw: 0.7071
```

## Demo

[![Patrol Behavior Demo]()]()

## References
- [bcr_bot robot platform](https://github.com/blackcoffeerobotics/bcr_bot)
- [Nav2 Navigation Stack](https://navigation.ros.org/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

## License
Apache-2.0
