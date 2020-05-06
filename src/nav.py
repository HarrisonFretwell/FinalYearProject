#!/usr/bin/env python
# A simple ROS publisher node in Python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import numpy as np
import math
import sys
from RecordPosition import RecordPosition
from pi import Pi
from OdometeryNav import OdometeryNav
from BioOdometryNav import BioOdometryNav
import random as rnd
rnd.seed(2)
print(rnd.random())
#Measured in 10ths of a second
MAX_WANDER_LOOPS = 500
class Nav:
    def callback(self, odom):
        position = odom.pose.pose.position
        x = position.x
        y = position.y
        orientation = odom.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                       orientation.y, orientation.z,orientation.w], 
                       'sxyz')
        self.RecordPosition.update_position(x,y)


    def __init__(self,nav_type):
        rospy.init_node('odom_subscriber', anonymous=True)
        self.publisher_object = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subscriber_object = rospy.Subscriber("odom", Odometry, self.callback)
        self.wander_loops = 0
        self.angular_v = 0
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.publisher_object.publish(vel_msg)
        self.RecordPosition = RecordPosition()
        if(nav_type == "bio"):
            navigation = BioOdometryNav(self.position_callback)
        else:
            navigation = OdometeryNav(self.position_callback)
        self.wander(None)
        navigation.main_loop()
        rospy.on_shutdown(self.RecordPosition.export)
        rospy.loginfo("subscriber node is active...")

        

    def main_loop(self):
        rospy.spin()
    
    def position_callback(self,x,y,yaw):
        if not (self.wander_loops < MAX_WANDER_LOOPS):
            self.return_home(x,y,yaw)

    def wander(self,event):
        if self.wander_loops < MAX_WANDER_LOOPS:
            rospy.loginfo("Random wandering")
            vel_msg = Twist()
            vel_msg.linear.x = 0.4
            if self.wander_loops > 10:
                self.angular_v += min(0.5,max(-0.5,(rnd.random()-0.5) /15))
            else:
                self.angular_v = 0
            vel_msg.angular.z = self.angular_v
            self.publisher_object.publish(vel_msg)
            #Wander again in 0.1 seconds
            self.wander_loops += 1
            rospy.loginfo(self.wander_loops)
            rospy.Timer(rospy.Duration(0.1), self.wander, oneshot=True)

    def return_home(self,x,y,yaw):
        def hyp(x,y):
            return math.sqrt(math.pow(x,2) + math.pow(y,2))
        if hyp(x,y) > 0.1:
            target_yaw = (math.degrees(math.atan2(y,x)) - math.degrees(math.atan2(0,0)))
        else:
            target_yaw = 360
        target_yaw = target_yaw % 360
        yaw = yaw %360
        angular = (target_yaw-yaw)/360 * 2
        rospy.loginfo("==============")
        rospy.loginfo(yaw)
        rospy.loginfo(target_yaw)
        rospy.loginfo(angular)
        
        x = min(0.5,hyp(x,y)) * min(1,abs(1/(angular*90)))
        rospy.loginfo(x)
        vel_msg = Twist()
        vel_msg.linear.x = x if hyp(x,y) > 0.1 else 0
        vel_msg.angular.z = angular if abs(target_yaw-yaw) > 0.2 else 0
        self.publisher_object.publish(vel_msg)

if __name__ == '__main__':
    nav_type = ""
    if (len(sys.argv) - 1) < 1:
        print("No command line argument provided, include '-b' or '-o' to specify odometry or bio-odometry")
        exit()
    else:
        if sys.argv[1] ==  '-b':
            nav_type = "bio"
        elif sys.argv[1] == '-o':
            nav_type = "odo"
        else:
            print("Invalid command line argument")
            print("provided: "+sys.argv[1])
            print("Please provide either '-b' or '-o'")
            exit()
        if len(sys.argv) - 1 == 2 :
            rnd.seed(sys.argv[2])
    subscriber_instance = Nav(nav_type)
    subscriber_instance.main_loop()