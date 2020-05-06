#!/usr/bin/env python
# A simple ROS publisher node in Python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import numpy as np
import math

wheel_base_diameter_mm = 287.0
wheel_diameter_mm = 66.0
wheel_ratio = wheel_diameter_mm /  wheel_base_diameter_mm
n_headings = 4
class BioOdometryNav:
    def calculate_heading(self,left_w_pos,right_w_pos):
        wheel_pos_dif_deg = math.degrees(right_w_pos-left_w_pos)/2
        angle = wheel_pos_dif_deg*wheel_ratio
        return angle
    
    def update_headings(self,joint_state):
        #Sanity check
        if self.last_left_w_pos is 0 and self.last_right_w_pos is 0:
            self.last_right_w_pos = joint_state.position[0]
            self.last_left_w_pos = joint_state.position[1]
            return

        right_w_pos = joint_state.position[0]
        left_w_pos = joint_state.position[1]
        #Calculate current robot angle
        self.angle = self.calculate_heading(left_w_pos,right_w_pos)
        #Calculate difference in wheel positions since last update
        left_w_diff = left_w_pos - self.last_left_w_pos
        right_w_diff = right_w_pos - self.last_right_w_pos
        displacement_m = (left_w_diff + right_w_diff)
        #Update each of the heading neurons
        for heading in self.pi_headings:
            angle_diff = math.cos(math.radians(self.angle-heading))
            #Biological neurons cannot be negative, so min of 0
            angle_diff = max(0,angle_diff)
            #Rounded to ensure small differences don't build
            self.position[heading] += round(angle_diff*displacement_m,6)
        self.last_right_w_pos = right_w_pos
        self.last_left_w_pos = left_w_pos


    def callback(self, joint_state):
        self.update_headings(joint_state)
        x,y = 0,0
        for heading in self.pi_headings:
            x -= math.cos(math.radians(heading)) * self.position[heading]
            y -= math.sin(math.radians(heading)) * self.position[heading]
        rospy.loginfo("------===")
        rospy.loginfo(x)
        rospy.loginfo(y)
        rospy.loginfo("==----===")
        self.position_callback(x,y,self.angle)
                

    def __init__(self, position_callback):
        self.position_callback = position_callback
        rospy.init_node('odom_subscriber', anonymous=True)
        #Calculate the angles of each of the directional headings
        pi_headings = [heading * 360 /n_headings for heading in range(1,n_headings+1)]
        self.subscriber_object = rospy.Subscriber("joint_states", JointState, self.callback)
        self.angle = 180
        self.last_right_w_pos = self.last_left_w_pos =  0
        self.pi_headings = [heading * 360 /n_headings for heading in range(1,n_headings+1)]
        self.position = dict.fromkeys(self.pi_headings,0)
        rospy.loginfo("subscriber node is active...")

    def main_loop(self):
        rospy.spin()