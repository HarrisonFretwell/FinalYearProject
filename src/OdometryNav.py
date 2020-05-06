#!/usr/bin/env python
# A simple ROS publisher node in Python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import numpy as np
import math
from RecordPosition import RecordPosition
class OdometryNav:
    
    def callback(self, odom):
        position = odom.pose.pose.position
        x = position.x
        y = position.y
        orientation = odom.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                       orientation.y, orientation.z,orientation.w], 
                       'sxyz')
        self.position_callback(x,y,yaw)

    def __init__(self,position_callback):
        rospy.init_node('odom_subscriber', anonymous=True)
        self.position_callback = position_callback
        self.subscriber_object = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.on_shutdown(self.RecordPosition.export)
        rospy.loginfo("subscriber node is active...")