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
from hummingbird_nmpc.trajectory import CubicSpline3D
from hummingbird_nmpc.state import State
from hummingbird_nmpc.config import *
from hummingbird_nmpc.utils import euler_to_quaternion

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
        self.hover_state = None
        
        # Create NMPC controller
        self.controller = Controller(t_horizon=3*N/CONTROL_RATE,n_nodes=N)

        self.rate = rospy.Rate(CONTROL_RATE)
        
    def odomCallback(self, data:Odometry):
        self.odom = data

    def goalCallback(self, data:PoseStamped):
        if data != PoseStamped():
            if self.state != State.HOVERING:
                rospy.logwarn("Cannot execute the goal")
            else:
                self.state = State.TRACKING
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
        if np.abs(self.odom.pose.pose.position.z - HEIGHT) < EPSILON:
            self.state = State.HOVERING
            self.hover_state = self.getCurrentState()
        else:
            # Prepare goal
            pg = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, HEIGHT])
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

    def hovering(self):
        # Current state
        current = self.getCurrentState()
        mode = 'pose'

        # NMPC Solve
        self.angular_velocity = self.controller.run_optimization(initial_state=current, goal=self.hover_state, mode=mode)

    def tracking(self):
        cp = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
        cg = np.array([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z])

        if np.linalg.norm(cg-cp) < EPSILON:
            self.state = State.HOVERING
            self.hover_state = self.getCurrentState()
        else:
            # x = [self.odom.pose.pose.position.x, self.goal.pose.position.x]
            # y = [self.odom.pose.pose.position.y, self.goal.pose.position.y]
            # z = [self.odom.pose.pose.position.z, self.goal.pose.position.z]
            # target = self.trajectoryGeneration(x, y, z)
            # mode = 'traj'
            target = np.array([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z])
            mode = 'pose'

            # Current state
            current = self.getCurrentState()

            # NMPC Solve
            self.angular_velocity = self.controller.run_optimization(initial_state=current, goal=target, mode=mode)

    def landing(self):
        if np.abs(self.odom.pose.pose.position.z - 0.0) < EPSILON:
            # Trying to unpause physics in Gazebo
            try:
                rospy.wait_for_service(PAUSE_SERVICE, timeout=5.0)
                rospy.ServiceProxy(PAUSE_SERVICE,Empty)()
                rospy.loginfo("Unpause physics Gazebo")
                self.state = State.IDLE
            except:
                rospy.logwarn("{} not available".format(PAUSE_SERVICE))
        else:
            # Prepare goal
            pg = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, LAND])
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
                self.hovering()
            elif self.state == State.TRACKING:
                self.tracking()
            elif self.state == State.LANDING:
                self.landing()

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

    def trajectoryGeneration(self, x, y, z):
        # Trajectory generation
        sp = CubicSpline3D(x, y, z)
        s = np.arange(0, sp.s[-1], DS)
        length = len(s)
        traj = []
        for i in range(N+1):
            if i < length:
                ix, iy, iz = sp.calc_position(s[i])
                p = np.array([ix, iy, iz])
                q = euler_to_quaternion(0,0,sp.calc_yaw(s[i]))
                v = [0,0,0]
                w = [0,0,0]
                traj.append(np.concatenate([p, q, v, w]))
            else:
                traj.append(traj[-1])
        return np.array(traj)

if __name__ == "__main__":
    try:
        hummingbird = Hummingbird()
        hummingbird.execute()
    except rospy.ROSInterruptException:
        pass