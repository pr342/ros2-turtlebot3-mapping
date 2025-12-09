# 🚀 ROS 2 TurtleBot3 Mapping Project

This project demonstrates **real-time SLAM (Simultaneous Localization and Mapping)** using **TurtleBot3 in a Gazebo simulation environment with ROS 2**.
The robot is manually controlled using a keyboard to explore the environment and generate a 2D occupancy map in real time using **RViz2**.

This repository contains the **live mapping screen recording and the generated map output**.

---

## 🎯 Objective of the Project

* To simulate a **mobile robot (TurtleBot3)** in a virtual environment
* To perform **real-time environment mapping using SLAM**
* To visualize sensor data, robot movement, and map generation in **RViz2**
* To save and export the final generated map

---

## 📂 Repository Contents

```
ros2-turtlebot3-mapping/
├── ros2_turtlebot3_mapping_demo.webm   → Screen recording of live SLAM mapping
└── map_XXXXXXXX.pgm                    → Generated occupancy grid map
```

---

## 🧠 Technologies & Tools Used

* **ROS 2 (Humble)**
* **Gazebo Simulator**
* **RViz2 Visualization Tool**
* **TurtleBot3 Packages**
* **Teleop Keyboard Control**
* **Cartographer SLAM**

---

## ⚙️ What Was Implemented

* Launched **TurtleBot3 simulation in Gazebo**
* Started **SLAM mapping using Cartographer**
* Controlled robot motion using **keyboard teleoperation**
* Visualized:

  * Laser scan data
  * Robot movement
  * Live map generation
* Successfully **generated and saved the final map**
* Recorded the full mapping process as a **demo video**

---

## 🎥 Demo Output

* ✅ **Live Mapping Video** (`.webm`)
* ✅ **Generated Map File** (`.pgm`)

These demonstrate:

* Real-time map generation
* Robot navigation
* Sensor data integration.
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
