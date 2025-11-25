# ROS2 A* Navigation

A simplified ROS2 navigation stack using the A* pathfinding algorithm for robot navigation. This package provides a lightweight alternative to Nav2 while maintaining core navigation functionality.

## Overview

This package implements a complete navigation solution consisting of:
- **A* Global Planner**: Classic A* pathfinding algorithm for finding optimal paths
- **Pure Pursuit Controller**: Path following controller for smooth trajectory execution
- **Navigation Node**: ROS2 node integrating planner and controller with action interface

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Navigation Node                               │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    Action Server                             │   │
│  │              (navigate_to_pose)                              │   │
│  └─────────────────────────────────────────────────────────────┘   │
│           │                                      │                   │
│           ▼                                      ▼                   │
│  ┌─────────────────┐                  ┌─────────────────────────┐  │
│  │  A* Planner     │                  │  Pure Pursuit           │  │
│  │                 │                  │  Controller             │  │
│  │ - Plan path     │ ──── Path ────► │                         │  │
│  │ - Obstacle      │                  │ - Follow path           │  │
│  │   inflation     │                  │ - Velocity commands     │  │
│  └─────────────────┘                  └─────────────────────────┘  │
│           ▲                                      │                   │
│           │                                      ▼                   │
│  ┌─────────────────┐                  ┌─────────────────────────┐  │
│  │  /map           │                  │  /cmd_vel               │  │
│  │  (OccupancyGrid)│                  │  (Twist)                │  │
│  └─────────────────┘                  └─────────────────────────┘  │
│           ▲                                      │                   │
│           │                                      ▼                   │
│  ┌─────────────────┐                  ┌─────────────────────────┐  │
│  │  TF2 Listener   │◄──── /odom ────┤  Robot Base              │  │
│  └─────────────────┘                  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## Installation

### Prerequisites
- ROS2 Jazzy (or compatible version)
- nav2_msgs package
- tf2_ros package

### Build from Source

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository (if not already in workspace)
# git clone <repository_url>

# Build the package
cd ~/ros2_ws
colcon build --packages-select ros2_astar_nav

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Navigation Node

```bash
# Using default parameters
ros2 launch ros2_astar_nav astar_navigation.launch.py

# With simulation time
ros2 launch ros2_astar_nav astar_navigation.launch.py use_sim_time:=true

# With custom parameter file
ros2 launch ros2_astar_nav astar_navigation.launch.py params_file:=/path/to/params.yaml

# With custom topic remappings
ros2 launch ros2_astar_nav astar_navigation.launch.py \
    map_topic:=/my_map \
    odom_topic:=/my_odom \
    cmd_vel_topic:=/my_cmd_vel
```

### Send Navigation Goals

Using ROS2 CLI:
```bash
# Send a goal using action client
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Using Python:
```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    node = rclpy.create_node('navigation_client')
    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = 1.0
    goal_msg.pose.pose.position.y = 2.0
    goal_msg.pose.pose.orientation.w = 1.0
    
    action_client.wait_for_server()
    future = action_client.send_goal_async(goal_msg)
    
    rclpy.spin_until_future_complete(node, future)
```

### Visualize Path in RViz

Add the following displays in RViz:
- **Map**: Topic `/map`
- **Path**: Topic `/plan`
- **Pose**: Topic `/navigate_to_pose/feedback` (current pose)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| **Planner Parameters** |
| `allow_diagonal` | bool | `true` | Enable 8-directional movement |
| `heuristic_weight` | double | `1.0` | Weight for heuristic function |
| `obstacle_cost_threshold` | int | `90` | Occupancy value threshold for obstacles |
| `inflation_radius` | double | `0.3` | Safety margin around obstacles (meters) |
| **Controller Parameters** |
| `max_linear_velocity` | double | `0.5` | Maximum linear velocity (m/s) |
| `max_angular_velocity` | double | `1.0` | Maximum angular velocity (rad/s) |
| `lookahead_distance` | double | `0.5` | Pure Pursuit lookahead distance (meters) |
| `goal_position_tolerance` | double | `0.1` | Position tolerance for goal (meters) |
| `goal_orientation_tolerance` | double | `0.1` | Orientation tolerance for goal (radians) |
| **General Parameters** |
| `control_frequency` | double | `10.0` | Control loop frequency (Hz) |
| `robot_base_frame` | string | `"base_link"` | Robot base frame name |
| `global_frame` | string | `"map"` | Global/map frame name |

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Static occupancy grid map |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/plan` | `nav_msgs/Path` | Planned path for visualization |

### Actions
| Action | Type | Description |
|--------|------|-------------|
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | Navigation goal interface |

## Comparison with Nav2

| Feature | This Package | Nav2 |
|---------|-------------|------|
| **Planner** | A* algorithm | Multiple planners (Navfn, Smac, etc.) |
| **Controller** | Pure Pursuit | Multiple controllers (DWB, MPPI, etc.) |
| **Recovery** | Basic | Comprehensive recovery behaviors |
| **Costmap** | Simple inflation | Layered costmap with multiple layers |
| **Complexity** | Low | High |
| **Use Case** | Simple robots, learning | Production, complex environments |

### When to Use This Package
- Learning ROS2 navigation concepts
- Simple robot applications
- Resource-constrained systems
- Prototyping and testing
- Educational purposes

### When to Use Nav2
- Production deployments
- Complex environments
- Need for advanced features (behaviors, waypoints, etc.)
- Multi-robot navigation
- Dynamic obstacle avoidance

## Algorithm Details

### A* Pathfinding
The A* algorithm finds the optimal path using:
- **Cost function**: `f(n) = g(n) + h(n)`
  - `g(n)`: Actual cost from start
  - `h(n)`: Heuristic (Euclidean distance to goal)
- **Open set**: Priority queue (min-heap) ordered by f-cost
- **Closed set**: Hash set of visited cells
- **Movement**: 8-directional (diagonal allowed by default)

### Pure Pursuit Controller
The controller uses the Pure Pursuit algorithm:
- Finds a lookahead point on the path
- Calculates curvature: `κ = 2 * sin(α) / L`
- Angular velocity: `ω = v * κ`

Where:
- `α`: Angle to lookahead point
- `L`: Lookahead distance
- `v`: Linear velocity

## Troubleshooting

### Common Issues

1. **"No map received"**
   - Ensure a map is being published to `/map`
   - Check that the map topic remapping is correct

2. **"No odometry received"**
   - Verify odometry is being published to `/odom`
   - Check TF tree for transforms

3. **"Failed to plan path"**
   - Verify start and goal positions are valid (not on obstacles)
   - Check inflation radius isn't too large
   - Ensure map resolution is appropriate

4. **"TF transform failed"**
   - Verify TF tree is complete (map → odom → base_link)
   - Check frame names in parameters match actual frames

5. **Robot moves erratically**
   - Reduce `max_linear_velocity` and `max_angular_velocity`
   - Increase `lookahead_distance`
   - Check control frequency

### Debug Tips

```bash
# Check topics
ros2 topic list
ros2 topic echo /map --once
ros2 topic echo /odom --once

# Check TF tree
ros2 run tf2_tools view_frames

# Check parameters
ros2 param list /astar_navigation
ros2 param get /astar_navigation max_linear_velocity

# Monitor action
ros2 action info /navigate_to_pose
```

## Future Improvements

- [ ] Dynamic obstacle detection
- [ ] Path smoothing (splines)
- [ ] Costmap layers (inflation, obstacle)
- [ ] Recovery behaviors
- [ ] Velocity smoothing
- [ ] Goal queueing
- [ ] Lifecycle management
- [ ] Component node support

## License

Apache License 2.0

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## Authors

Maintainer <maintainer@example.com>
