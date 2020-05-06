#!/usr/bin/env python
# A simple ROS publisher node in Python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import numpy as np
import math
import csv

class RecordPosition():
    def __init__(self):
        self.position_history = []

    def update_position(self,x,y):
        self.position_history.append([x,y])
    
    def export(self):
        print("exporting")
        wtr = csv.writer(open ('./catkin_ws/src/nav_exercise/src/out.csv', 'w'), delimiter=',')
        wtr.writerow(["x","y"])
        wtr.writerows(self.position_history)
   
   