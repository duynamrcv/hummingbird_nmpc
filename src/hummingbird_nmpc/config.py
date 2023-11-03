#!/usr/bin/env python3
'''
Filename: hummingbird_nmpc/src/hummingbird_nmpc/config.py
Created Date: Thursday, November 2nd 2023
Author: Duy Nam Bui

Copyright (c) 2023 Duy Nam Bui
'''

# Parameters
MAX_THRUST = 10.0   # Quadrotor max thrust
N = 20              # NMPC horizontal length
CONTROL_RATE = 20   # Control rate
EPSILON = 0.05      # Error epsilon
HEIGHT = 1.0        # Take off height
LAND = 0.0          # Landing height
DS = 0.02           # Distance of each interpolated points


# Gazebo
UNPAUSE_SERVICE = "/gazebo/unpause_physics"     # std_srvs/Empty
PAUSE_SERVICE = "/gazebo/pause_physics"         # std_srvs/Empty

# Commnand
STATE_TOPIC = "state"                           # std_msgs.msg/UInt8
SPEED_COMMAND_TOPIC = "command/motor_speed"     # mav_msgs/Actuators
GOAL_TOPIC = "command/goal"                     # geometry_msgs/PoseStamped
PATH_TOPIC = "command/path"                     # nav_msgs/Path

# Quadrotor
ODOMETRY_TOPIC = "odometry_sensor1/odometry"    # nav_msgs/Odometry