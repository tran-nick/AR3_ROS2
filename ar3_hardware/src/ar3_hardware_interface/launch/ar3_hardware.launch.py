import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Load universal robot description format (URDF)
        # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("ar3_description"),
            "urdf",
            "ar3.urdf.xacro",
        )
    )

    robot_description = {"robot_description": robot_description_config.toxml()}

    ros2_controllers_path = os.path.join(
        get_package_share_directory("ar3_moveit_config"),
        "config",
        "controller_manager.yaml",
    )

    # Start controllers / hardware interface
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
             ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    return LaunchDescription(
        [
            ros2_control_node,
        ]
    )