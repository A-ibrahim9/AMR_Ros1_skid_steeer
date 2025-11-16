
# 🚗 AMR Ros1 Skid-Steer Robot — Simulation & Control

This repository contains a complete **ROS Noetic** package for simulating and controlling a **Skid-Steer Autonomous Mobile Robot (AMR)** using **Gazebo** and **URDF/Xacro**.  
The project includes robot modeling, Gazebo plugins, teleoperation, and environment setup.

---

## 📌 Table of Contents
- [GIF](#GIF)
- [Project Overview](#project-overview)
- [Repository Structure](#repository-structure)
- [Requirements](#requirements)
- [How to Run](#how-to-run)
- [Launch Files](#launch-files)
- [TF Tree](#tf_tree)
- [Topics](#topics)
- [Demo & Result](#Demo_&_Result)
- [Future Improvements](#future-improvements)
- [Author](#author)

---
## GIF

![GIF](https://github.com/user-attachments/assets/3f3a9e9c-25ad-4b8b-9171-b522d0370a67)

---

## 📘 Project Overview

This project implements a 4-wheel skid-steer mobile robot using ROS1 Noetic.
The robot includes:

- 360° LiDAR
- Kinect RGB-D Camera
- Two Ultrasonic Sensors
- 4-Wheel Skid-Steer Drive System
- Gazebo Simulation
- SLAM using gmapping

The goal is to build a modular simulation-ready platform for mapping, perception, and future navigation.

---

## 📂 Repository Structure
```
AMR_Ros1_skid_steer/
│
├── src/
│
├── skid_steering_pkgs/                # Main robot control package
│   ├── skid_steer_description/        # URDF/Xacro files for robot model
│   │   ├── config/
│   │   ├── meshes/
│   │   ├── launch/
│   │   └── urdf/
│   │       ├── main_skid.xacro        # Base link, box above base, front support for ultrasonic
│   │       ├── plugins.gazebo         # Plugins for skid steer, LiDAR, Kinect, ultrasonic
│   │       ├── sensors.xacro          # LiDAR, Kinect, ultrasonic sensors
│   │       └── wheels.xacro           # 4 wheels: f_right, f_left, b_right, b_left
│   │
│   ├── skid_steer_gazebo/             
│   │   ├── launch/                     # spawn_skid_steer --> run robot in Gazebo & RViz
│   │   └── world/                      # Custom Gazebo world
│   │
│   ├── skid_steer_slam/                
│   │   ├── launch/
│   │   ├── maps/
│   │   └── config/
│   │
│   └── teleop/                         # Keyboard teleoperation
│
├── CMakeLists.txt
├── package.xml
├── photos/                             # Robot images, world snapshots, output results
├── demo_gazebo/                        # Demo simulations
└── README.md
```
---

## 🖥️ Requirements

| Component | Version |
|----------|---------|
| Ubuntu   | 20.04 |
| ROS      | Noetic |
| Gazebo   | 11 |
| Python   | ≥ 3.8 |

Install teleop package:

```bash
sudo apt install ros-noetic-teleop-twist-keyboard
```
---

## 🚀 How to Run the Project

**1️⃣ Clone the repository**
```bash
cd ~/catkin_ws/src
git clone https://github.com/A-ibrahim9/AMR_Ros1_skid_steeer.git
cd ..
catkin_make
source devel/setup.bash
```
**2️⃣ Run Gazebo simulation**
```bash
roslaunch skid_steer_gazebo spwan_skid_steer.launch
```
**3️⃣ Run SLAM (Mapping)**
```bash
roslaunch skid_steer_slam skid_gmapping.launch
```
**4️⃣ Control the robot (Teleop)**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
5️⃣ Save generated map
```bash
rosrun map_server map_saver -f my_map
```
---

## 📂 Launch Files

### spawn_skid_steer.launch
- Launches Gazebo + custom world.
- Spawns the robot.
- Opens RViz.

### robot_description.launch
- Send urdf to param server
- Send robot states to tf

### skid_gmapping.launch
- Runs Gmapping SLAM.
- Uses LiDAR for mapping.
- Opens RViz for live map visualization.

---

## Build Process

### 1️⃣ Robot Modeling (skid_steer_description)
- Started by building the main robot model using **URDF/Xacro**.
- Added the base link, chassis, wheels, and mounting frame.
- Integrated sensors:
  - **LiDAR** → for mapping & obstacle detection  
  - **Kinect/Depth Camera** → 3D perception  
  - **Ultrasonic Sensors** → short-range proximity  
- Added **Gazebo plugins**:
  - Skid-steer drive plugin  
  - LiDAR plugin  
  - Depth camera plugin  
  - Ultrasonic sensor plugin  

### 2️⃣ Simulation Environment (skid_steer_gazebo)
- Created a custom Gazebo world.
- Added launch files to:
  - Load URDF
  - Spawn the robot
  - Start Gazebo + RViz

### 3️⃣ Mapping / SLAM (skid_steer_slam)
- Configured mapping nodes (Gmapping).
- Tuned LiDAR settings for better map quality.
- Added map saving/loading functionality.

### 4️⃣ Teleoperation
- Added keyboard teleop node for manual control during testing.

---

## 🔀 TF Tree

<img width="1842" height="569" alt="image" src="https://github.com/user-attachments/assets/eb4169a4-6d8d-4bfd-9133-937f8437b7f8" />

---

## 📡 ROS Topics

- **Published Topics**

| Topic                     | Message Type                   | Description                                      |
|---------------------------|---------------------------------|--------------------------------------------------|
| /odom                     | nav_msgs/Odometry              | Odometry from Gazebo skid-steer plugin           |
| /joint_states             | sensor_msgs/JointState         | Wheel joint state feedback                       |
| /scan                     | sensor_msgs/LaserScan          | 360° LiDAR scan data                             |
| /camera/color/image_raw   | sensor_msgs/Image              | RGB image from Kinect camera                     |
| /camera/depth/image_raw   | sensor_msgs/Image              | Depth image from Kinect                          |
| /camera/color/camera_info | sensor_msgs/CameraInfo         | Camera intrinsics                                |
| /ultrasonic_front         | sensor_msgs/Range              | Front ultrasonic distance                        |
| /ultrasonic_rear          | sensor_msgs/Range              | Rear ultrasonic distance                         |

- **Subscribed Topics**

| Topic        | Message Type          | Description                         |
|--------------|------------------------|-------------------------------------|
| /cmd_vel     | geometry_msgs/Twist    | Velocity commands (teleop or nodes) |

---

## Demo & Result

### Demo

### Result

- **Robot in Gazebo**

<img width="1183" height="1010" alt="R_gazebo" src="https://github.com/user-attachments/assets/6e2b2d62-08fe-42d8-af11-8ce2b3446678" />

- **Robot in Rviz**

<img width="1131" height="923" alt="R_rviz" src="https://github.com/user-attachments/assets/81c059f3-b4ff-4097-b886-09a7c7842e73" />

- **My World**

<img width="1345" height="737" alt="myWorld" src="https://github.com/user-attachments/assets/ba0a9084-9f3c-4e64-aad4-6567df58b2d0" />

- **Data of LiDAR**

<img width="1914" height="861" alt="data_of_lidar" src="https://github.com/user-attachments/assets/1b24865d-592d-4529-b28f-244510881614" />

- **Data of Camera Kinect**

<img width="939" height="465" alt="data_of_camera_kinect" src="https://github.com/user-attachments/assets/1bcd089b-3577-42c4-819d-41028b4ba3ab" />

- **Data_of ultrasonic**

<img width="1914" height="781" alt="data_of_ultrasonic" src="https://github.com/user-attachments/assets/733b470e-2164-49be-a81c-5d5c1bfd1a97" />

---
## 🔮 Future Work

The following modules are planned and will be added later:
- AMCL Localization
- Full Navigation Stack (Global + Local Planners)
- Sensor Fusion (IMU, wheel encoders)
