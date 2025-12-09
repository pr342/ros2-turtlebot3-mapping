# Software Assessment - ROS 2 Robotics Workspace

This repository contains a ROS 2 workspace with multiple packages for robotics simulation and control, including hospital world environments, Doosan robot integration, and TurtleBot3 simulation.

## Repository Structure

```
assessment_ws/
├── src/
│   ├── aws-robomaker-hospital-world/    # Hospital environment simulation
│   ├── my_doosan_pkg/                    # Doosan robot package
│   └── turtlebot3_sim_bringup/          # TurtleBot3 simulation bringup
```

## Packages Overview

### 1. AWS RoboMaker Hospital World

A detailed hospital environment for Gazebo simulation with realistic 3D models including elevators, curtains, medical equipment, and patient areas.

**Features:**
- Multi-floor hospital environment
- Interactive elements (elevators, curtains)
- 40+ medical equipment and furniture models from Ignition Fuel

**Requirements:**
- Python 3 and pip3
- Gazebo

**Launch Files:**
- `hospital.launch.py` - Launch the hospital world in Gazebo
- `view_hospital.launch.py` - View the hospital world

### 2. My Doosan Package

Custom package for Doosan robot simulation and control forked from dvalenciar's repository.

**Launch Files:**
- `my_doosan_gazebo.launch.py` - Launch Doosan robot in Gazebo
- `my_doosan_rviz.launch.py` - Launch RViz visualization
- `my_doosan_controller.launch.py` - Launch robot controllers
- `my_doosan_gazebo_controller.launch.py` - Launch Gazebo with controllers

**Configuration:**
- Controller configuration: `config/simple_controller.yaml`
- Custom world: `worlds/my_empty_world.world`

### 3. TurtleBot3 Simulation Bringup

ROS 2 package for TurtleBot3 robot simulation with differential drive control.

**Features:**
- Gazebo simulation integration
- RViz2 visualization
- Differential drive control via `cmd_vel` topics
- Pre-configured RViz layouts

**Launch Files:**
- `bringup.launch.py` - Launch both Gazebo and RViz
- `gazebo.launch.py` - Launch TurtleBot3 in Gazebo
- `rviz.launch.py` - Launch RViz2 visualization

## Prerequisites

### System Requirements
- Ubuntu 20.04/22.04
- ROS 2 (Humble/Foxy/Iron)
- Gazebo
- Python 3

### ROS 2 Dependencies

```bash
sudo apt update
sudo apt install \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-turtlebot3-description \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2-control \
    python3-pip
```

### Python Dependencies

For the hospital world package:
```bash
cd assessment_ws/src/aws-robomaker-hospital-world
pip3 install -r requirements.txt
```

## Building the Workspace

1. **Clone the repository:**
```bash
git clone https://github.com/sakar-robotics/software-assesment.git
cd software-assesment
```

2. **Navigate to the workspace:**
```bash
cd assessment_ws
```

3. **Install dependencies:**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. **Build the workspace:**
```bash
colcon build
```

5. **Source the workspace:**
```bash
source install/setup.bash
```

## Usage Examples

### Launch Hospital World

```bash
source install/setup.bash
ros2 launch aws_robomaker_hospital_world hospital.launch.py
```

### Launch TurtleBot3 Simulation

1. Set the TurtleBot3 model:
```bash
export TURTLEBOT3_MODEL=burger
```

2. Launch the simulation:
```bash
source install/setup.bash
ros2 launch turtlebot3_sim_bringup bringup.launch.py
```

3. Control the robot (in a new terminal):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Launch Doosan Robot

Launch Gazebo with controllers:
```bash
source install/setup.bash
ros2 launch my_doosan_pkg my_doosan_gazebo_controller.launch.py
```

Or launch components separately:
```bash
# Terminal 1 - Gazebo
ros2 launch my_doosan_pkg my_doosan_gazebo.launch.py

# Terminal 2 - RViz
ros2 launch my_doosan_pkg my_doosan_rviz.launch.py

# Terminal 3 - Controllers
ros2 launch my_doosan_pkg my_doosan_controller.launch.py
```

## Troubleshooting

### Gazebo Model Loading Issues
If models don't load properly in the hospital world:
```bash
cd assessment_ws/src/aws-robomaker-hospital-world
./setup.sh
```

### TurtleBot3 Model Not Set
If you get errors about TURTLEBOT3_MODEL:
```bash
export TURTLEBOT3_MODEL=burger
# Add to ~/.bashrc to make permanent
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

### Build Errors
Clean and rebuild:
```bash
cd assessment_ws
rm -rf build install log
colcon build
```

## License

Please refer to individual package licenses:
- AWS RoboMaker Hospital World: See `src/aws-robomaker-hospital-world/LICENSE`
- Other packages: Refer to respective package.xml files

## Contributing

This is an assessment workspace. For contributions to individual packages, please refer to their respective repositories.

## Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Documentation](http://gazebosim.org/)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [AWS RoboMaker](https://aws.amazon.com/robomaker/)