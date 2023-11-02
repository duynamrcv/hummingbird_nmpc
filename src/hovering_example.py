 #!/usr/bin/env python3

'''
Filename: /hummingbird_nmpc/src/hovering_example.py
Created Date: Saturday, October 28th 2023
Author: Duy-Nam Bui

Copyright (c) 2023 Duy-Nam Bui
'''


import rospy
from std_srvs.srv import Empty
from mav_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np
from hummingbird_nmpc.nmpc_controller import Controller
from hummingbird_nmpc.config import *

class Hummingbird:
    def __init__(self):
        rospy.init_node("hovering_example")

        # Trying to unpause physics in Gazebo
        try:
            rospy.wait_for_service(UNPAUSE_SERVICE, timeout=5.0)
            rospy.ServiceProxy(UNPAUSE_SERVICE,Empty)()
            rospy.loginfo("Unpause physics Gazebo")
        except:
            rospy.logwarn("{} not available".format(UNPAUSE_SERVICE))

        # Publisher
        self.ac_pub = rospy.Publisher(rospy.get_namespace()+SPEED_COMMAND_TOPIC, Actuators, queue_size=CONTROL_RATE)

        # Subscriber
        rospy.Subscriber(rospy.get_namespace()+ODOMETRY_TOPIC, Odometry, self.odomCallback)
        rospy.Subscriber(rospy.get_namespace()+GOAL_TOPIC, PoseStamped, self.goalCallback)
        
        self.odom = Odometry()
        self.goal = PoseStamped()
        
        # Create NMPC controller
        N = 20
        self.controller = Controller(t_horizon=3*N/CONTROL_RATE,n_nodes=N)

        self.rate = rospy.Rate(CONTROL_RATE)
        
    def odomCallback(self, data:Odometry):
        self.odom = data

    def goalCallback(self, data:PoseStamped):
        if data != PoseStamped():
            self.goal = data
            if data.header.frame_id == "":
                self.goal.header.frame_id == "world"
            

    def execute(self):
        while not rospy.is_shutdown():

            # Current state
            p = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
            q = np.array([self.odom.pose.pose.orientation.w, self.odom.pose.pose.orientation.x, 
                            self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z])
            v = np.array([self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z])
            w = np.array([self.odom.twist.twist.angular.x, self.odom.twist.twist.angular.y, self.odom.twist.twist.angular.z])
            current = np.concatenate([p, q, v, w])
            
            # Goal state
            target = np.array([0,0,1, 1,0,0,0, 0,0,0, 0,0,0])
            if self.goal != PoseStamped():
                pg = np.array([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z])
                qg = np.array([self.goal.pose.orientation.w, self.goal.pose.orientation.x, 
                                self.goal.pose.orientation.y, self.goal.pose.orientation.z])
                vg = np.array([0,0,0])
                wg = np.array([0,0,0])
                target = np.concatenate([pg, qg, vg, wg])

            # NMPC Solve
            angular_velocity = self.controller.run_optimization(initial_state=current, goal=target)
            
            # Send command
            ac_msg = Actuators()
            ac_msg.angular_velocities = angular_velocity
            print(angular_velocity)
            self.ac_pub.publish(ac_msg)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        hummingbird = Hummingbird()
        hummingbird.execute()
    except rospy.ROSInterruptException:
        pass