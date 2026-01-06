# ROS2 Indoor Autonomous Car Navigation

[Watch Demo](https://github.com/wilsonchenghy/ROS2_Indoor_Autonomous_Car_Navigation/blob/main/ROS2_Indoor_Nav.MOV)

![](https://github.com/wilsonchenghy/ROS2_Indoor_Autonomous_Car_Navigation/blob/main/ROS2_Indoor_Nav.jpeg)

## Technical Summary

This project implements a complete ROS2 navigation stack for indoor autonomous navigation. The system consists of four main nodes that work together to enable point-to-point navigation while avoiding static obstacles.

### Architecture Overview

The navigation pipeline follows a standard robotics architecture: **perception → mapping → planning → control**. Each node handles a specific responsibility and communicates through ROS2 topics.

### Main Nodes

#### 1. Costmap Node
The costmap node processes raw LiDAR scan data and converts it into a local occupancy grid centered around the robot. It subscribes to `/lidar` and publishes `/costmap`. 

**Key features:**
- Converts polar LiDAR coordinates to Cartesian grid cells
- Marks obstacles at detected points
- Implements obstacle inflation (1.0m radius) to account for robot footprint and safety margins
- Uses a 10m x 10m local grid with 0.1m resolution

I chose to inflate obstacles rather than just marking them because the robot has physical dimensions—we need to ensure the entire robot body can pass through, not just a point.

#### 2. Map Memory Node
The map memory node integrates local costmaps into a persistent global map. It subscribes to `/costmap` and `/odom/filtered`, and publishes `/map`.

**Key features:**
- Maintains a 30m x 30m global map
- Updates the map when the robot moves more than 1.5m (reduces computational overhead)
- Transforms costmap data from robot frame to world frame using odometry
- Uses linear fusion (max operation) to merge new observations with existing map data

The 1.5m update threshold is a balance between map freshness and performance. Linear fusion with max ensures obstacles persist even if they're temporarily out of view, which is crucial for static obstacle avoidance.

#### 3. Planner Node
The planner node generates collision-free paths from the robot's current position to a goal point. It subscribes to `/map`, `/goal_point`, and `/odom/filtered`, and publishes `/path`.

**Algorithm: A\* Path Planning**

I chose A\* over other algorithms (like Dijkstra or RRT) for a few reasons:
- **Optimality**: A\* guarantees finding the shortest path if one exists (unlike RRT which is probabilistic)
- **Efficiency**: The heuristic function makes it much faster than Dijkstra for most cases
- **Deterministic**: Unlike sampling-based methods, A\* always produces the same result for the same input

**Implementation details:**
- 8-directional movement (including diagonals) with proper cost weighting (√2 for diagonal, 1.0 for straight)
- Euclidean distance heuristic with obstacle proximity penalty
- Replans every 500ms while navigating to handle dynamic map updates
- Goal tolerance of 0.5m to account for control precision

The obstacle-aware heuristic adds a penalty when nodes are near obstacles, which naturally biases the path away from walls and tight spaces—making the robot prefer safer, wider corridors.

#### 4. Control Node
The control node executes path following using velocity commands. It subscribes to `/path` and `/odom/filtered`, and publishes `/cmd_vel`.

**Algorithm: Pure Pursuit Control**

I chose Pure Pursuit over other path tracking controllers (like PID or Model Predictive Control) because:
- **Simplicity**: Easy to implement and tune with just one parameter (lookahead distance)
- **Smoothness**: Produces naturally smooth trajectories without jerky movements
- **Robustness**: Works well for differential drive robots and handles path curvature gracefully
- **Industry standard**: Widely used in autonomous vehicles and mobile robotics

**Implementation details:**
- Lookahead distance of 1.0m (tuned for the robot's dynamics)
- Constant linear speed of 0.5 m/s
- Angular velocity computed from steering angle to lookahead point
- Updates at 10Hz (100ms timer) for responsive control

The lookahead distance is critical—too small and the robot oscillates, too large and it cuts corners. 1.0m works well for this robot's turning radius and speed.

### System Integration

The nodes form a closed-loop system:
1. **Costmap** creates local obstacle awareness
2. **Map Memory** builds global understanding over time
3. **Planner** finds safe paths through the known environment
4. **Control** executes the plan with smooth motion

The system handles the case where the planner might initially plan through unseen obstacles—as the robot moves and the map updates, the planner automatically replans to avoid newly discovered obstacles.

### Technical Decisions

- **ROS2 over ROS1**: Modern API, better performance, and active development
- **C++ implementation**: Performance-critical for real-time navigation
- **Modular design**: Each node is independent, making debugging and testing easier
- **Grid-based representation**: Simple, efficient, and sufficient for static indoor environments
