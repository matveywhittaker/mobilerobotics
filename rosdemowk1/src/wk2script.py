# -*- coding: utf-8 -*-
"""
Created on Fri Feb  1 11:46:28 2019

@author: student
"""



import rospy as r
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

r.init_node("Lincoln")
speed = 0.5
rot = 0;
t = Twist()

def callback(data):
    if data.ranges[len(data.ranges) / 2] < 0.6:
        global speed
        speed = 0
        
        global rot
        rot = 0.25
     

pub = r.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 10)
sub_laser = r.Subscriber("/scan", LaserScan, callback)

while not r.is_shutdown():
    print(speed)
    t.linear.x = speed
    t.angular.z = rot
    pub.publish(t)
