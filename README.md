# Omnidirectional Vision‑Guided Autonomous Robot with Object Recognition and Navigation (ROS 2)

## 📌 Project Overview

This project presents the design and implementation of an **omnidirectional autonomous mobile robot** equipped with **Mecanum wheels**, **vision‑based perception**, **ultrasonic sensing**, and a **ROS 2 navigation stack**. The robot is capable of:

* Omnidirectional motion using Mecanum wheel kinematics
* Vision‑guided object recognition and tracking
* Autonomous navigation with global & local planning
* Real‑time obstacle avoidance in dynamic environments
* Hybrid planning using **RRT + D* Lite**
* Manual and autonomous mode switching

The system is designed with a **modular ROS 2 architecture**, enabling scalability, reusability, and easy integration of additional sensors or algorithms.

---

## 🎯 Key Features

* **Omnidirectional Mobility** – Smooth holonomic motion (forward, sideways, diagonal, rotation)
* **ROS 2 (Nav2 Stack)** – Localization, mapping, and navigation
* **Vision‑Based Object Recognition** – Camera‑based perception pipeline
* **Hybrid Path Planning** – RRT for global path, D* Lite for dynamic replanning
* **Reactive Obstacle Avoidance** – Ultrasonic sensor–based safety layer
* **Costmap‑Aware Navigation** – Dynamic obstacle updates
* **Manual + Autonomous Modes** – Priority‑based command fusion
* **Real‑Time Feedback** – Sensor data and camera feed visualization

---

## 🧠 System Architecture

### High‑Level Architecture

The system is divided into the following subsystems:

1. **Perception Subsystem**

   * Camera processing (object detection, tracking)
   * Ultrasonic distance sensing
   * Noise filtering and thresholding

2. **Planning Subsystem**

   * Global planner (RRT)
   * Incremental replanning (D* Lite)
   * Local planner (Nav2)

3. **Decision & Control Subsystem**

   * Behavior decision engine
   * Command fusion and priority resolver
   * Mecanum kinematic controller

4. **Execution Subsystem**

   * Motor actuation
   * Servo camera control

---

## 🔁 Autonomous Robot Flow

1. System start & hardware initialization
2. ROS 2 nodes and parameters loaded
3. Sensors, camera, and SLAM nodes activated
4. User selects **Manual** or **Autonomous** mode
5. Autonomous stack initializes planners
6. Continuous perception → planning → control loop
7. Robot navigates until goal is reached or stop signal received

---

## 🗺️ Path Planning Logic (RRT + D* Lite)

### Step‑by‑Step Flow

* Initialize environment map
* Take start and goal positions
* Generate **global path using RRT**
* Load RRT path into **D* Lite**
* Robot begins motion along the path
* Continuously sense environment
* If environment unchanged → follow path
* If obstacle detected → D* Lite updates only affected edges
* Extract updated path
* Mecanum controller follows the new trajectory
* Repeat until goal is reached

### Why RRT + D* Lite?

* **RRT** efficiently handles large, unknown spaces
* **D* Lite** enables fast replanning without recomputing the entire graph
* Ideal for **dynamic and partially known environments**

---

## 🚧 Obstacle Avoidance Algorithm

### Threshold‑Based Reactive Avoidance

1. Activate ultrasonic sensor
2. Continuously measure distance ( d(t) )
3. Collect multiple samples
4. Apply noise filtering (mean + standard deviation)
5. Remove outliers
6. Compare filtered distance with threshold ( T )
7. If ( d(t) \le T ): trigger avoidance (stop / turn)
8. If ( d(t) > T ): move forward
9. Update wheel velocities using Mecanum kinematics
10. Repeat sensing–decision–action loop

This reactive layer runs **in parallel** with the navigation stack, acting as a safety override.

---

## 📷 Vision‑Based Perception

* Camera mounted on servo for dynamic tracking
* Image processing pipeline:

  * Frame acquisition
  * Object detection / color tracking
  * Feature extraction
  * Bounding box & centroid estimation
* Vision data used for:

  * Obstacle awareness
  * Object following
  * User feedback overlay

---

## ⚙️ Mecanum Wheel Kinematics

The robot uses **holonomic motion equations** to independently control each wheel:

* Allows translation in X, Y, and rotation simultaneously
* Wheel velocities computed from:

  * Linear velocity (vx, vy)
  * Angular velocity (ω)

This enables precise navigation in narrow and cluttered environments.

---

## 🧩 ROS 2 Node Structure

```
ros2_ws/
├── src/
│   ├── perception_pkg/
│   │   ├── camera_node.py
│   │   ├── ultrasonic_node.cpp
│   │   └── sensor_fusion_node.py
│   ├── planning_pkg/
│   │   ├── rrt_planner.cpp
│   │   ├── dstar_lite.cpp
│   │   └── path_manager.cpp
│   ├── control_pkg/
│   │   ├── mecanum_controller.cpp
│   │   └── motor_driver_node.cpp
│   ├── decision_pkg/
│   │   └── behavior_engine.cpp
│   └── bringup_pkg/
│       └── launch/
│           └── robot_launch.py
```

---

## 🔄 Command Fusion & Priority Resolver

Command priority order:

1. Emergency stop
2. Obstacle avoidance override
3. Manual control
4. Autonomous navigation

This ensures **safe and predictable robot behavior** under all conditions.

---

## 🛠️ Hardware Components

* Mecanum wheel chassis (4‑wheel)
* DC geared motors
* Motor driver (high‑current)
* Ultrasonic sensor
* Camera module
* Servo motor (camera pan)
* Embedded controller (Raspberry Pi / MCU)
* Battery & power management

---

## 💻 Software Stack

* **ROS 2 (Humble / Foxy)**
* Nav2 Navigation Stack
* OpenCV
* Python & C++
* SLAM Toolbox
* RViz2
* Linux (Ubuntu)

---

## ▶️ How to Run

```bash
# Build workspace
colcon build

# Source setup
source install/setup.bash

# Launch robot system
ros2 launch bringup_pkg robot_launch.py
```

---

## 📊 Results & Performance

* Accurate omnidirectional motion
* Reliable obstacle avoidance in dynamic environments
* Fast replanning with minimal latency
* Stable navigation with continuous costmap updates

---

## 🚀 Future Improvements

* LiDAR‑based perception
* 3D SLAM integration
* Deep‑learning object detection
* Multi‑robot coordination
* Autonomous docking & charging

---


## 📜 License

This project is released under the **MIT License**.

---

⭐ *If you find this project useful, please star the repository!*
