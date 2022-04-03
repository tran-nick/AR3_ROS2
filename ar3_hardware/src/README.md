# AR3 ROS2 Packages

Contains the ROS2 packages for the AR3 robot arm.

## Build

    $ git clone https://gitlab.rim.net/medical-robot-project/medical-robot.git
    $ cd ~/medical-robot/ar3_ros2_control_hardware
    $ source /opt/ros/foxy/setup.bash
    $ colcon build --symlink-install

## Run Hardware Interface and Controller setup

    $ source ./install/setup.bash
    $ ros2 launch ar3_hardware_interface ar3_hardware_bringup_launch.py

## Run RViz Visualization and MoveIt motion planning
In a new terminal window

    $ ros2 launch ar3_moveit_config ar3_moveit_bringup_launch.py
