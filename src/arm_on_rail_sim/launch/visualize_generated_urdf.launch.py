from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # IMPORTANT: You must update this path when using it to debug other configurations!
    urdf_file = Path("~/user_ws/src/arm_on_rail_sim/xacro_generated_urdf_for_testing.urdf").expanduser()

    # Node to publish the joint states
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # Node to run RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_gui_node,
    ])