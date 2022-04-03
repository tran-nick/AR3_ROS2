# AR3 ROS2 Packages

Contains the ROS2 packages for the AR3 robot arm, simulation of robotic control in RViz only. ros2_control.xacro configured to load fake components to simulate hardware. 

## Build

    $ git clone https://github.com/tran-nick/AR3_ROS2.git
    $ cd ~/AR3_ROS2/ar3_simulation
    $ source /opt/ros/foxy/setup.bash
    $ colcon build --symlink-install

## Run Simple Demo

    $ source ./install/setup.bash
    $ ros2 launch ar3_moveit_config demo2.launch.py
       
