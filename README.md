# 🧠 Project 2 Bonus: Hybrid Deliberative–Reactive Robotics using ROS & Gazebo

**Author:** Subhash Chandra
**Course:** CS 5023 / 4023 – Introduction to Intelligent Robotics
**Institution:** The University of Oklahoma
**Semester:** Fall 2025

---

## 📘 Overview

This project implements a **Hybrid Deliberative–Reactive Architecture** for autonomous robot control using **ROS Melodic** and **Gazebo**.
A simulated **TurtleBot 2** (Kobuki base with ASUS Xtion sensor) operates within a mapped environment to perform multiple goal-directed navigation tasks.

The system integrates **high-level deliberation (task and path planning)** with **low-level reactive behaviors (collision avoidance and teleoperation)**.
It demonstrates how deliberative reasoning can be effectively combined with reactive execution in robotics.

---

## 🎯 Objectives

1. Develop an autonomous robot that can **plan and execute multiple navigation tasks**.
2. Integrate **task planning**, **path planning**, **execution monitoring**, and **reactive control** within a unified ROS framework.
3. Perform **simultaneous localization and mapping (SLAM)** using the GMapping algorithm.
4. Visualize robot movement, sensor data, and map generation in **RViz**.
5. Evaluate the system’s ability to recover from navigation failures through execution monitoring.

---

## 🧩 System Architecture

The architecture consists of **four primary functional nodes**:

| Layer                          | Node                     | Description                                                                      |
| ------------------------------ | ------------------------ | -------------------------------------------------------------------------------- |
| **Deliberative Layer**         | `task_planner.py`        | Determines the optimal order of tasks to minimize total travel distance.         |
| **Path Planning Layer**        | `path_planner.py`        | Computes heading angles and linear distances between waypoints.                  |
| **Execution Monitoring Layer** | `execution_monitor.py`   | Monitors navigation progress and triggers recovery upon failure.                 |
| **Reactive Layer**             | `reactive_controller.py` | Handles real-time obstacle avoidance, teleoperation override, and bumper events. |

Auxiliary nodes:

* `slam_gmapping`: Performs real-time mapping using laser scans.
* `depthimage_to_laserscan`: Converts depth camera data into laser scans.
* `rviz`: Visualizes the map, robot, and sensor data.

---

## 🧠 Data Flow Diagram

```text
       +-------------------+
       |   Task Planner    |
       +--------+----------+
                |
                v
       +-------------------+
       |   Path Planner    |
       +--------+----------+
                |
                v
       +-------------------+
       | Execution Monitor |
       +--------+----------+
                |
                v
       +-------------------+
       | Reactive Control  |
       +--------+----------+
                |
                v
       +-------------------+
       |  Gazebo + Sensors |
       +-------------------+
```

---

## 🗺️ Environment & Coordinates

* The simulation world replicates a **15 × 20 ft** environment with a rectangular room and hallway (same as Project 1).
* Coordinates are expressed in **feet**, with `(0, 0)` at the lower-left corner.
* The robot’s initial pose and subsequent task goals are specified as ordered pairs of points:

```
((2, 3), (9, 8))
((12, 9), (4, 14))
```

Leave a blank line after the final pair to trigger the task planner.

---

## 🧱 Project Structure

```
catkin_wsp1/
│
├── src/
│   └── project2_pkg/
│       ├── launch/
│       │   ├── project2_world.launch
│       │   └── project1_mapping.launch
│       ├── scripts/
│       │   ├── task_planner.py
│       │   ├── path_planner.py
│       │   ├── execution_monitor.py
│       │   └── reactive_controller.py
│       ├── worlds/
│       │   └── project2_world.world
│       ├── rviz/
│       │   └── project2_mapping.rviz
│       └── CMakeLists.txt
│
├── .catkin_workspace
└── .gitignore
```

---

## ⚙️ How to Run the Project

### Step 1: Build the Workspace

```bash
cd ~/project2/catkin_wsp1
catkin_make
source devel/setup.bash
```

### Step 2: Launch the Hybrid System

```bash
roslaunch project2_pkg project2_world.launch
```

This launches:

* Gazebo simulation
* SLAM mapping (`gmapping`)
* All four core hybrid control nodes
* RViz visualization

### Step 3: Provide Task Coordinates

Enter task pairs in the console as:

```
((x1, y1), (x2, y2))
((x3, y3), (x4, y4))
```

Then press **Enter** on a blank line to start execution.

---

## 🧭 Verification & Debugging

| Purpose              | Command                                |
| -------------------- | -------------------------------------- |
| List active topics   | `rostopic list`                        |
| Check TF transforms  | `rosrun tf view_frames`                |
| Monitor transforms   | `rosrun tf tf_echo map base_footprint` |
| View map metadata    | `rostopic echo /map_metadata`          |
| Check LaserScan rate | `rostopic hz /scan`                    |

---

## 🧰 Dependencies

| Package                                                  | Purpose                  |
| -------------------------------------------------------- | ------------------------ |
| `ros-melodic-desktop-full`                               | Base ROS environment     |
| `turtlebot`, `turtlebot_gazebo`, `turtlebot_description` | Robot model & simulation |
| `gmapping`                                               | SLAM mapping             |
| `depthimage_to_laserscan`                                | Depth → Laser conversion |
| `rviz`                                                   | Visualization            |
| `xacro`                                                  | URDF processing          |

---

## 📊 Results

* The robot successfully completes multi-task navigation with map-based visualization.
* Execution monitor detects and reports stalled progress or collision events.
* GMapping node continuously updates the occupancy grid map.
* TF frames (`map`, `odom`, `base_footprint`) are maintained consistently throughout navigation.

---

## 🧩 Key Learning Outcomes

1. Integration of **deliberative planning** with **reactive behavior control**.
2. Implementation of **ROS multi-node architecture** for autonomous navigation.
3. Application of **GMapping** for SLAM in unknown environments.
4. Visualization of sensor data and transformation trees using **RViz**.
5. Understanding of **execution monitoring** and recovery mechanisms in robotics.

---

## 👨‍💻 Author

**Subhash Chandra**
Graduate Research Assistant, University of Oklahoma
Email: [subhash.chandra@ou.edu](mailto:subhash.chandra@ou.edu)
GitHub: [@subhashchandra001](https://github.com/subhashchandra001)

---

> *"Hybrid intelligence emerges when deliberation and reaction cooperate harmoniously in autonomous systems."*
