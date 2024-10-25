# RB1 ROS2 Integration

# Introduction
This package makes integrating controlling the RB1 robot easier by using ROS2 Control Framework. It includes Differential drive and elevator usage.

# Installation

Download the git repository
cd ~ros2_ws/src

git clone https://github.com/Hamz115/rb1_rb2_description.git

cd ~ros2_ws

colcon build

source install/setup.bash

Now we are ready to integrate it

# Launch
Launch the simulation using this command

ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py

Give it 2 to 3 minutes

Next we activate the controllers

# Controller Activation
ros2 control list_controllers --controller-manager /rb1/controller_manager

# List hardware interfaces
ros2 control list_hardware_interfaces --controller-manager /rb1/controller_manager

# To move the robot
ros2 topic pub --rate 10 /rb1/diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0, z: 0.0}, angular: {x: 0.3, y: 0.0, z: 0.0}}"

# To lift the elevator
ros2 service call /apply_joint_effort gazebo_msgs/srv/ApplyJointEffort '{joint_name: "robot_elevator_platform_joint", effort: 1.0, start_time: {sec: 0, nanosec: 0}, duration: {sec: -1, nanosec: 0} }'

# To bring down the elevator
ros2 service call /clear_joint_efforts 'gazebo_msgs/srv/JointRequest' '{joint_name: "robot_elevator_platform_joint"}'