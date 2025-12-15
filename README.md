# 🚀 ROS 2 TurtleBot3 Mapping Project with Robotic Arm

This project demonstrates **real-time SLAM (Simultaneous Localization and Mapping)** using **TurtleBot3** in a **Gazebo simulation environment** with **ROS 2**, with an **attached robotic arm**. The robot is manually controlled using a keyboard to explore the environment and generate a 2D occupancy map in real time using **RViz2**, while also simulating a robotic arm mounted on the TurtleBot3.

This repository contains the live mapping screen recording, generated map output, and setup for the robotic arm integration.

---

## 🎯 Objective of the Project

* To simulate a mobile robot (**TurtleBot3**) with a **robotic arm** in a virtual environment
* To perform **real-time environment mapping** using SLAM
* To visualize **sensor data, robot movement, robotic arm movement, and map generation** in RViz2
* To save and export the **final generated map**

---

## 📂 Repository Contents

```
ros2-turtlebot3-mapping/
├── ros2_turtlebot3_mapping_demo.webm   → Screen recording of live SLAM mapping
├── map_XXXXXXXX.pgm                    → Generated occupancy grid map
└── robotic_arm_simulation/             → Launch and control scripts for the robotic arm
```

---

## 🧠 Technologies & Tools Used

* **ROS 2 (Humble)**
* **Gazebo Simulator**
* **RViz2 Visualization Tool**
* **TurtleBot3 Packages**
* **Robotic Arm Packages (simulated on TurtleBot3)**
* **Teleop Keyboard Control**
* **Cartographer SLAM**

---

## ⚙️ What Was Implemented

* Launched **TurtleBot3 simulation in Gazebo** with a mounted **robotic arm**
* Started **SLAM mapping** using Cartographer
* Controlled robot motion using **keyboard teleoperation**
* Simulated **robotic arm movement** using pre-defined commands
* Visualized in RViz2:

  * Laser scan data
  * Robot movement
  * Robotic arm position and motion
  * Live map generation
* Successfully generated and saved the **final 2D map**
* Recorded the full mapping process as a **demo video**

---

## 🎥 Demo Output

✅ **Live Mapping Video (.webm)**
✅ **Generated Map File (.pgm)**
✅ **Robotic Arm Simulation**

These demonstrate:

* Real-time map generation
* Robot navigation
* Robotic arm integration
* Sensor data handling
* SLAM functionality in action

---

## ✅ Learning Outcomes

Through this project, I gained hands-on experience with:

* ROS 2 workspace setup
* Robot simulation in Gazebo
* SLAM & environment mapping
* RViz visualization tools
* Robot teleoperation
* Real-time sensor data handling
* Simulating a **robotic arm mounted on a mobile robot**
