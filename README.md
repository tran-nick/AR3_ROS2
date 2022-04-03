# AR3_ROS2 
ROS2 packages for Annin Robotics AR3 robot arm. AR3 arm used for development had been upgraded to AR4 platform by retrofitting Teensy 4.1 microcontroller with 600 MHz clock speed to reduce latency between processing ROS commands. 
The observed latency was a result from hybridizing teensy driver code from ar3_core to utilize differential stepping logic from annin robotics driver code in order to maintain constant end-effector Z-height when moving along X/Y cartesian path.

#### Specs:
- Ubuntu 20.04
- ROS2 Foxy

#### What was done:
- Ported [ar3_core](https://github.com/ongdexter/ar3_core) packages from ROS1 to ROS2
- Extended functionality to utilize moveit_servo for xbox controller and wacom tablet input for real-time jogging


#### REFERENCE(s):

- https://github.com/ongdexter/ar3_core

## Packages

1.  `ar3_hardware`

    Workspace containing packages to control physical hardware and recieve input from xbox / wacom controllers. 

    ### Build
    ```
    git clone https://github.com/tran-nick/AR3_ROS2.git
    cd ~/AR3_ROS2/ar3_hardware
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install
    ```

    ### Launch Hardware
    ```
    source install/setup.bash   # source workspace environment
    ros2 launch ar3_hardware_interface ar3_hardware.launch.py
    ```

    ### Launch Rviz + MoveIt
    ```
    ros2 launch xboxdemo moveit_servo.launch.py
    ```

    ### Launch Input Multiplexing Node
    ```
    ros2 launch xboxdemo hmi.launch.py
    ```

2. `ar3_simulation`

    Workspace with fake hardware loaded. Used for testing new node developments and visualization. 

    ### Build
    ```
    git clone https://github.com/tran-nick/AR3_ROS2.git   # Don't run if you've already cloned repo
    cd ~/AR3_ROS2/ar3_simulation
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install
    ```

    ### Launch
    ```
    ros2 launch ar3_moveit_config demo2.launch.py
    ```

3.  teensyduino_sketches

    Teensduino drivers for taking string commands from either [ARCS.py program from Annin Robotics](https://www.anninrobotics.com/downloads) or ROS2 application. Code is extension of driver from [ar3_core](https://github.com/ongdexter/ar3_core)

    * `ROS_ARCS_hybrid_indie.ino` -- Since we upgraded AR3 with AR4 parts, we needed driver that would work with both Teensy 3.5 and Teensy 4.1 board. This gets the board information from Arduino IDE and conditionally compiles driver based on Teensy you are uploading code to. 
    