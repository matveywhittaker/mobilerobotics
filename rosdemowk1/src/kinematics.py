import rospy
import cv2
import numpy
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

from geometry_msgs.msg import Twist

wheel_radius = .1
robot_radius = 1
class Test():
    def __init__(self):
        self.sub = rospy.Subscriber('/wheel_vel_left', Float32, self.callback)
        self.twist_sub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
# computing the forward kinematics for a differential drive
    def forward_kinematics(self,w_l, w_r):
        c_l = wheel_radius * w_l
        c_r = wheel_radius * w_r
        v = (c_l + c_r) / 2
        a = (c_r - c_l) / (2 * robot_radius)
        return (v, a)
    
    
    # computing the inverse kinematics for a differential drive
    def inverse_kinematics(self,v, a):
        c_l = v - (robot_radius * a)
        c_r = v + (robot_radius * a)
        w_l = c_l / wheel_radius
        w_r = c_r / wheel_radius
        return (w_l, w_r)
    
    
    
    # inverse kinematics from a Twist message (This is what a ROS robot has to do)
    def inverse_kinematics_from_twist(self,t):
        return inverse_kinematics(t.linear.x, t.angular.z)
        
    def callback(self, data):
        
        v,a = self.forward_kinematics(data.data, data.data)
    
        print v
        print a
        t = Twist()
    #
        t.linear.x = v
        t.angular.z = a
    #
        self.twist_sub.publish(t)

def main():
    
    test = Test()
    rospy.init_node('gkjusgf')
    rospy.spin()
    print 1
    

if __name__ == "__main__":
    main()
     
      

#    (w_l, w_r) = inverse_kinematics(0.0, 1.0)
#    print "w_l = %f,\tw_r = %f" % (w_l, w_r)
#
#    (v, a) = forward_kinematics(w_l, w_r)
#    print "v = %f,\ta = %f" % (v, a)
#
#    from geometry_msgs.msg import Twist
#    t = Twist()
#
#    t.linear.x = 0.3
#    t.angular.z = 0.8
#
#    (w_l, w_r) = inverse_kinematics_from_twist(t)
#    print "w_l = %f,\tw_r = %f" % (w_l, w_r)
