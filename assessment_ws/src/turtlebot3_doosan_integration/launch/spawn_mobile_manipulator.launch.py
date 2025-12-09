from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_path = "/home/protima/ros2_internship_project/software-assignment-pr342-main/assessment_ws/src/turtlebot3_doosan_integration"

    xacro_file = os.path.join(pkg_path, "urdf", "turtlebot3_doosan.urdf")

    robot_description = os.popen(f"xacro {xacro_file}").read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                "gazebo", "--verbose",
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so"
            ],
            output="screen"
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "mobile_manipulator",
                "-topic", "robot_description"
            ],
            output="screen"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        )
    ])
