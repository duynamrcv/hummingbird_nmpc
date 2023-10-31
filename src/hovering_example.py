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
# from 
from nav_msgs.msg import Odometry

from hummingbird_nmpc.nmpc_controller import Controller
import numpy as np

SPEED_COMMAND_TOPIC = "command/motor_speed"
ODOMETRY_TOPIC = "ground_truth/odometry"

UNPAUSE_SERVICE = "/gazebo/unpause_physics"

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
    
        control_rate = 20
        self.ac_pub = rospy.Publisher(rospy.get_namespace()+SPEED_COMMAND_TOPIC, Actuators, queue_size=control_rate)
        rospy.Subscriber(rospy.get_namespace()+ODOMETRY_TOPIC,Odometry,self.odomCallback)
        
        self.odom = Odometry()
        
        # Create NMPC controller
        N = 20
        self.controller = Controller(t_horizon=3*N/control_rate,n_nodes=N)

        self.rate = rospy.Rate(control_rate)
        
    def odomCallback(self,data:Odometry):
        self.odom = data

    def hovering(self,hover_position = np.array([0,0,1])):
        while not rospy.is_shutdown():
            p = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
            q = np.array([self.odom.pose.pose.orientation.w, self.odom.pose.pose.orientation.x, 
                            self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z])
            v = np.array([self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z])
            w = np.array([self.odom.twist.twist.angular.x, self.odom.twist.twist.angular.y, self.odom.twist.twist.angular.z])
            
            current = np.concatenate([p, q, v, w])
            angular_velocity = self.controller.run_optimization(initial_state=current, goal=hover_position)
            # print(self.odom)
            # print(angular_velocity)
            ac_msg = Actuators()
            # ac_msg.angular_velocities = [470, 470, 470, 470]
            ac_msg.angular_velocities = angular_velocity
            print(angular_velocity)
            self.ac_pub.publish(ac_msg)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        hummingbird = Hummingbird()
        hummingbird.hovering()
    except rospy.ROSInterruptException:
        pass