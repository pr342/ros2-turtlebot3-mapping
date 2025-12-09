from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(
                '/home/protima/ros2_internship_project/software-assignment-pr342-main/assessment_ws/src/turtlebot3_doosan_integration/urdf/turtlebot3_doosan.urdf'
            ).read()}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'mobile_manipulator',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])
