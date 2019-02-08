
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
from std_msgs.msg import String

from geometry_msgs.msg import Twist

class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        self.sub = rospy.Publisher('/result_topic', String)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()
        
    def callback(self, data):
        
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("BOOT", 2)
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = numpy.array([40, 40, 130])
        upper_red = numpy.array([70, 70, 180])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        h, w, d = cv_image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            ##self.twist.linear.x = 0.1
            self.twist.angular.z = -float(err) / 100
            print self.twist.angular.z

            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)


image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
