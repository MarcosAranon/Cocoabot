import os
import launch
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define paths
    pkgPath = FindPackageShare('cocoabot_description').find('cocoabot_description')
    urdfModelPath = os.path.join(pkgPath, 'urdf', 'cocoabot_description.urdf.xacro')
    rvizConfigPath = os.path.join(pkgPath, 'rviz2', 'cocoabot_config.rviz')

    # Ensure files exist
    if not os.path.exists(urdfModelPath):
        raise FileNotFoundError(f"URDF file not found: {urdfModelPath}")
    if not os.path.exists(rvizConfigPath):
        raise FileNotFoundError(f"RViz config file not found: {rvizConfigPath}")

    # Use xacro to process the URDF
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdfModelPath
        ]),
        value_type=str
    )

    # Parameters
    params = {"robot_description": robot_description_content}

    # Define nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

