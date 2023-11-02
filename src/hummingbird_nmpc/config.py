#!/usr/bin/env python3

# Parameters
CONTROL_RATE = 20

# Gazebo
UNPAUSE_SERVICE = "/gazebo/unpause_physics"     # std_srvs/Empty

# Commnand
SPEED_COMMAND_TOPIC = "command/motor_speed"     # mav_msgs/Actuators
GOAL_TOPIC = "command/goal"                     # geometry_msgs/PoseStamped
PATH_TOPIC = "command/path"                     # nav_msgs/Path

# Quadrotor
ODOMETRY_TOPIC = "odometry_sensor1/odometry"    # nav_msgs/Odometry