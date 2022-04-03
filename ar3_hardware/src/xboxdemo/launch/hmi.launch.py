
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch_ros.actions import Node 



def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

        # Message multiplexer node
    twist_mux_config = load_yaml(
        "twist_cmd_mux" , "config/topics.yaml"
    )

    twist_mux_node = Node(
        package="twist_cmd_mux",
        executable="twist_cmd_mux_node",
        name="servo_server_mux",
        parameters=[twist_mux_config],
        output="screen",
    )

    mouse_parameters_file = os.path.join(
        get_package_share_directory('mouse_teleop'),
        'config', 'mouse_teleop.yaml'
    )

    mouse_teleop = Node(
            package='mouse_teleop', 
            executable='mouse_teleop',
            parameters=[mouse_parameters_file],
            output="screen"
    )

    return LaunchDescription(
    [
        twist_mux_node,
        mouse_teleop
    ]

)