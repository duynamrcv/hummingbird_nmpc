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
from std_msgs.msg import UInt8

import numpy as np
from hummingbird_nmpc.nmpc_controller import Controller
from hummingbird_nmpc.state import State
from hummingbird_nmpc.config import *

class Hummingbird:
    def __init__(self):
        rospy.init_node("hovering_example")

        # Trying to unpause physics in Gazebo
        try:
            rospy.wait_for_service(UNPAUSE_SERVICE, timeout=5.0)
            rospy.ServiceProxy(UNPAUSE_SERVICE,Empty)()
            rospy.loginfo("Unpause physics Gazebo")
            self.state = State.TAKE_OFF
        except:
            rospy.logwarn("{} not available".format(UNPAUSE_SERVICE))

        # State
        self.state = State.IDLE

        # Publisher
        self.pub_ac = rospy.Publisher(rospy.get_namespace()+SPEED_COMMAND_TOPIC, Actuators, queue_size=CONTROL_RATE)
        self.pub_state = rospy.Publisher(rospy.get_namespace()+STATE_TOPIC, UInt8, queue_size=CONTROL_RATE)

        # Subscriber
        rospy.Subscriber(rospy.get_namespace()+ODOMETRY_TOPIC, Odometry, self.odomCallback)
        rospy.Subscriber(rospy.get_namespace()+GOAL_TOPIC, PoseStamped, self.goalCallback)
        
        self.odom = Odometry()
        self.goal = PoseStamped()
        self.angular_velocity = np.zeros(4)
        
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
    
    def publishState(self):
        msg_state = UInt8()
        msg_state.data = self.state.value
        self.pub_state.publish(msg_state)

    def publishActuators(self):
        msg_ac = Actuators()
        msg_ac.angular_velocities = self.angular_velocity
        self.pub_ac.publish(msg_ac)

    def takeOff(self):
        # Prepare goal
        pg = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 1.0])
        qg = np.array([1,0,0,0])
        vg = np.array([0,0,0])
        wg = np.array([0,0,0])
        
        # Goal state
        target = np.concatenate([pg, qg, vg, wg])
        mode = 'pose'

        # Current state
        current = self.getCurrentState()

        # NMPC Solve
        self.angular_velocity = self.controller.run_optimization(initial_state=current, goal=target, mode=mode)

    def execute(self):
        while not rospy.is_shutdown():
            if self.state == State.IDLE:
                self.state = State.TAKE_OFF
            elif self.state == State.TAKE_OFF:
                self.takeOff()
            elif self.state == State.HOVERING:
                pass
            elif self.state == State.TRACKING:
                pass
            elif self.state == State.LANDING:
                pass

            # Publish topics
            self.publishActuators()
            self.publishState()

            # Sleep
            self.rate.sleep()

    def getCurrentState(self):
        p = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
        q = np.array([self.odom.pose.pose.orientation.w, self.odom.pose.pose.orientation.x, 
                        self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z])
        v = np.array([self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z])
        w = np.array([self.odom.twist.twist.angular.x, self.odom.twist.twist.angular.y, self.odom.twist.twist.angular.z])
        current = np.concatenate([p, q, v, w])
        return current

if __name__ == "__main__":
    try:
        hummingbird = Hummingbird()
        hummingbird.execute()
    except rospy.ROSInterruptException:
        pass