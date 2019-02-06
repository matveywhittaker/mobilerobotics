import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

import math

rospy.init_node("lincoln")

# Our movement variables
speed = 0.5
rotation = 0

t = Twist()

def laser_scan_callback(laser_scan_data):
     
    global speed
    global rotation
    global close_to_object

    laser_scan_center = laser_scan_data.ranges[len(laser_scan_data.ranges) / 2]
    print laser_scan_center
    
    # Are we close to an object?
    if laser_scan_center < 0.8:     
        
        # Rotate the robot #
        rotation = 0.8
        speed = 0
     
        
    elif laser_scan_center > 0.5 or math.isnan(laser_scan_center):
        rotation = 0
   
velocity_publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 10)
laser_scan_subscriber = rospy.Subscriber("/scan", LaserScan, laser_scan_callback)

# Main Robot loop #
while not rospy.is_shutdown():

    # Move our robot forward #
    t.linear.x = speed
    
    # Rotate our robot #
    t.angular.z = rotation
    
    # Actually move the robot #
    velocity_publisher.publish(t)
