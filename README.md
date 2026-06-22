# ROS Obstacle Avoidance Navigation

Autonomous obstacle avoidance and goal-based navigation for TurtleBot3 using ROS, Gazebo, RViz, Occupancy Grid Mapping, A* Path Planning, and Path Following.

---

## Project Overview

This project implements a complete autonomous navigation pipeline for TurtleBot3 in a simulated Gazebo environment.

The robot receives a navigation goal, detects obstacles using LiDAR, generates an occupancy grid map, plans a path using A* search, and follows the generated path until reaching the target location.

The objective of this project is to demonstrate fundamental robotics concepts including:

* Autonomous Navigation
* Occupancy Grid Mapping
* Obstacle Detection
* Path Planning
* Path Following
* ROS Node Communication
* Gazebo Simulation
* RViz Visualization

---

## Features

* Real-time LiDAR obstacle detection
* Occupancy grid map generation
* A* path planning
* Goal-based navigation
* Path following controller
* Goal reached detection
* Gazebo simulation environment
* RViz visualization

---

## System Architecture

```text
LiDAR Scan
     |
     v
Occupancy Grid Builder
     |
     v
Occupancy Grid Map
     |
     v
A* Planner
     |
     v
Planned Path
     |
     v
Path Follower
     |
     v
Velocity Commands
     |
     v
TurtleBot3
```

---

## Implemented ROS Nodes

### Occupancy Grid Builder

**Purpose**

Generates a 2D occupancy grid map from LiDAR scan data.

**Subscribed Topics**

* `/scan`

**Published Topics**

* `/occupancy_grid`

---

### A* Planner

**Purpose**

Plans a path from the robot position to the target goal while avoiding detected obstacles.

**Subscribed Topics**

* `/occupancy_grid`
* `/goal`
* `/odom`

**Published Topics**

* `/planned_path`

---

### Path Follower

**Purpose**

Follows the generated path and produces robot velocity commands.

**Subscribed Topics**

* `/planned_path`
* `/odom`

**Published Topics**

* `/cmd_vel`

---

### Goal Reached Manager

**Purpose**

Monitors the robot position and determines when the goal has been reached.

**Subscribed Topics**

* `/odom`
* `/goal`

**Published Topics**

* `/goal_reached`

---

## ROS Topics

| Topic           | Message Type              | Description             |
| --------------- | ------------------------- | ----------------------- |
| /scan           | sensor_msgs/LaserScan     | LiDAR scan data         |
| /occupancy_grid | nav_msgs/OccupancyGrid    | Generated occupancy map |
| /goal           | geometry_msgs/PoseStamped | Navigation goal         |
| /planned_path   | nav_msgs/Path             | Planned path            |
| /cmd_vel        | geometry_msgs/Twist       | Robot velocity commands |
| /odom           | nav_msgs/Odometry         | Robot odometry          |
| /goal_reached   | std_msgs/Bool             | Goal completion status  |

---

## Software Stack

* ROS Noetic
* Gazebo
* RViz
* TurtleBot3
* C++
* Occupancy Grid Mapping
* A* Search Algorithm
* Differential Drive Control

---

## Project Structure

```text
turtlebot3_description
│
├── launch
│   ├── obstacle_detection.launch
│
├── src
│   ├── occupancy_grid_builder.cpp
│   ├── astar_planner.cpp
│   ├── path_follower.cpp
│   └── goal_reached_manager.cpp
│
├── urdf
│
├── meshes
│
├── rviz
│
├── CMakeLists.txt
│
└── package.xml
```

---

## Running the Project

### Build Workspace

```bash
cd ~/workspaces/Udemy_Ws

catkin_make

source devel/setup.bash
```

### Launch Simulation

```bash
roslaunch turtlebot3_description obstacle_detection.launch
```

### Publish Navigation Goal

```bash
rostopic pub -1 /goal geometry_msgs/PoseStamped "
header:
  frame_id: 'base_link'
pose:
  position:
    x: 3.0
    y: 2.0
    z: 0.0
  orientation:
    w: 1.0"
```

---

## Simulation Results

### Gazebo Environment

<p align="center">
  <img src="docs/images/gazebo_environment.png" width="700">
</p>

### Occupancy Grid Mapping

<p align="center">
  <img src="docs/images/occupancy_grid.png" width="700">
</p>

### Path Planning Visualization

<p align="center">
  <img src="docs/images/path_planning.png" width="700">
</p>

### Obstacle Detection

<p align="center">
  <img src="docs/images/obstacle_detection.png" width="700">
</p>

---

## Future Improvements

* Dynamic obstacle avoidance
* Dijkstra path planning
* RRT path planning
* Pure Pursuit controller
* SLAM integration
* Real TurtleBot3 deployment
* RGB-D camera integration
* Multi-obstacle navigation

---

## Learning Outcomes

Through this project, the following robotics concepts were implemented and tested:

* ROS Publisher/Subscriber communication
* Occupancy Grid Mapping
* A* Path Planning
* Autonomous Navigation
* Robot Motion Control
* Gazebo Simulation
* RViz Visualization
* Mobile Robot Software Development

---

## Author

**Zameer Hussain**

M.Eng Robotics Engineering

University of Applied Sciences Hamburg

Robotics | ROS | Autonomous Navigation | Computer Vision

