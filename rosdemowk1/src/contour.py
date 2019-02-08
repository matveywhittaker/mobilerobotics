
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


class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        self.sub = rospy.Publisher('/result_topic', String)

    def callback(self, data):
        
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("BOOT", 2)
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((30, 30, 60)),
                                 numpy.array((100, 100, 170)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((30, 30, 60)),
                                 numpy.array((100, 100, 170)))

        self.sub.publish(str(numpy.mean(hsv_img[:, :, 0])))
        self.sub.publish(str(numpy.mean(hsv_img[:, :, 1])))
        self.sub.publish(str(numpy.mean(hsv_img[:, :, 2])))

        _, bgr_contours, hierachy = cv2.findContours(
            bgr_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        _, hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        for c in bgr_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                
                
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))
                
        
        thresh = cv_image
        thresh[:,:,2] = bgr_thresh * cv_image[:,:, 2]
        thresh[:,:,1] = bgr_thresh * cv_image[:,:, 1]         
        thresh[:,:,0] = bgr_thresh * cv_image[:,:, 0]
        
        
        
    
        
        print '===='
        cv2.imshow("Image window", cv_image)
        cv2.imshow("BOOT", thresh)
        cv2.waitKey(1)

image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
