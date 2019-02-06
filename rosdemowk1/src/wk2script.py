import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

rospy.init_node("lincoln")

# Our movement variables
speed = 0.5
rotation = 0
t = Twist()

def laser_scan_callback(laser_scan_data):
     
    global speed
    global rotation
    
    # Are we close to an object?
    if laser_scan_data.ranges[len(laser_scan_data.ranges) / 2] < 0.6:
      
        # Stop moving the robot #
        speed = 0
        
        # Rotate the robot #
        rotation = 0.25
     
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
