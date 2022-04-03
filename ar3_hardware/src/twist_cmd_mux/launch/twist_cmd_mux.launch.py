import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    mux_config = os.path.join(
        get_package_share_directory('twist_cmd_mux'),
        'config', 'topics.yaml'
    )

    twist_cmd_mux = launch_ros.actions.Node(
        package='twist_cmd_mux', 
        executable='twist_cmd_mux_node',
        name="twist_cmd_mux",
        parameters=[mux_config],
        output="screen"    
    )

    return LaunchDescription(
        [twist_cmd_mux]
    )