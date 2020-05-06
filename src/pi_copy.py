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
import random as rnd
wheel_base_diameter_mm = 287.0
wheel_diameter_mm = 66.0
wheel_ratio = wheel_diameter_mm /  wheel_base_diameter_mm
n_headings = 4
#Something exploding from RAM usage, problem with self.position I think, generally algorithm is busted somewhere
#Can maybe work out direction to best reduce  headings
class Pi:
    def calculate_heading(self,left_w_pos,right_w_pos):
        wheel_pos_dif_deg = math.degrees(right_w_pos-left_w_pos)/2
        angle = wheel_pos_dif_deg*wheel_ratio
        return angle
    
    def update_headings(self,joint_state):
        if self.last_left_w_pos is 0 and self.last_right_w_pos is 0:
            self.last_right_w_pos = joint_state.position[0]
            self.last_left_w_pos = joint_state.position[1]
            return
        right_w_pos = joint_state.position[0]
        left_w_pos = joint_state.position[1]
        self.angle = self.calculate_heading(left_w_pos,right_w_pos)
        left_w_diff = left_w_pos - self.last_left_w_pos
        right_w_diff = right_w_pos - self.last_right_w_pos
        displacement_m = (left_w_diff + right_w_diff)/2 * 1/360 * wheel_diameter_mm * math.pi 
        self.y -= math.sin(math.radians(self.angle)) * displacement_m
        self.x -= math.cos(math.radians(self.angle)) * displacement_m
        '''for heading in self.pi_headings:
            #Considering that for biological systems, neurons cannot be negative
            angle_diff = max(0,math.cos(math.radians(self.angle-heading)))
            rospy.loginfo("Heres you thing bro: {}".format(round(angle_diff*displacement_m,4)))
            self.position[heading] += round(angle_diff*displacement_m,4)
        rospy.loginfo("Position...")
        rospy.loginfo(self.position)
        #Update position'''
        self.last_right_w_pos = right_w_pos
        self.last_left_w_pos = left_w_pos

    def wander(self):
        rospy.loginfo("Random wandering")
        vel_msg = Twist()
        vel_msg.linear.x = 0.6
        vel_msg.angular.z = (rnd.random()-0.5)/2
        self.publisher_object.publish(vel_msg)

    def callback(self, joint_state):
        self.update_headings(joint_state)
        if self.wandering:
            self.wander()
        else:
            '''x,y = 0,0
            for heading in self.pi_headings:
                y -= math.cos(math.radians(heading)) * self.position[heading]
                x -= math.sin(math.radians(heading)) * self.position[heading]
            rospy.loginfo('-------')
            rospy.loginfo("predicted x is {}".format(x))
            rospy.loginfo("predicted y is {}".format(y))'''
            self.return_home(self.x,self.y,self.angle)
        self.wandering = self.wander_count < 1000 if self.wandering else self.wandering
        self.wander_count += 1
                
        
            

    def __init__(self):
        rospy.init_node('odom_subscriber', anonymous=True)
        #Calculate the angles of each of the directional headings
        pi_headings = [heading * 360 /n_headings for heading in range(1,n_headings+1)]
        self.publisher_object = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subscriber_object = rospy.Subscriber("joint_states", JointState, self.callback)

        self.last_right_w_pos = self.last_left_w_pos =  0
        self.x,self.y = 0,0
        self.pi_headings = [heading * 360 /n_headings for heading in range(1,n_headings+1)]
        self.position = dict.fromkeys(self.pi_headings,0)
        self.wandering = True
        rospy.loginfo("subscriber node is active...")
        self.wander_count = 0

    def return_home(self,x,y,yaw):
        target_yaw = (math.degrees(math.atan2(y,x)) - math.degrees(math.atan2(0,0)))
        target_yaw = target_yaw % 360
        yaw = yaw %360
        rospy.loginfo(target_yaw)
        rospy.loginfo(yaw)
        #angular = 1 /(1 + math.exp(-(target_yaw-yaw)/360))
        angular = ((target_yaw-yaw))/360
        x = min(0.25,math.sqrt(math.pow(x,2)+math.pow(y,2))) * min(1,abs(1/(angular*180)))
        rospy.loginfo(angular)
        rospy.loginfo(x)
        rospy.loginfo("------")
        vel_msg = Twist()
        vel_msg.linear.x = x
        vel_msg.angular.z = angular
        self.publisher_object.publish(vel_msg)


    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Pi()
    subscriber_instance.main_loop()