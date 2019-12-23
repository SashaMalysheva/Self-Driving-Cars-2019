#!/usr/bin/env python
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage, Image
import rospy
import numpy as np
import math
import sys
sys.path.append(sys.path.pop(1))
sys.path.append(sys.path.pop(1))
import cv2

def findIntersection(l1, l2, height):
    [[x1,y1],[x2,y2]] = l1
    [[x3,y3],[x4,y4]] = l2
    d = ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))
    if d == 0: return 0, height
    px = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / d
    py = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / d
    return px, py

def find_angle(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    height, width, _ = image.shape
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    canny_image = cv2.Canny(gray_image, 100, 200)
    lines = cv2.HoughLinesP(canny_image[int(height/3):int(2*height/3),:],
                            rho=6,
                            theta=np.pi/180,
                            threshold=160,
                            lines=np.array([]),
                            minLineLength=100,
                            maxLineGap=20)
    #array of (x_1, y_1, x_2, y_2)
    lines = [[[l[0][0], l[0][1] + int(height/3), l[0][2], l[0][3] + int(height/3)]] for l in lines]
    lines = [[l[0][:2], l[0][2:]] for l in lines] #array of (x_1, y_1, x_2, y_2)
    half_sorted_lines = [sorted(l, key = lambda x: x[1]) for l in lines] #sort poins in line: from top to bottom
    sorted_lines = sorted(half_sorted_lines, key = lambda x: x[1][0]) #sort poins in lines: from left to right
    l1, l2 = sorted_lines[0], sorted_lines[-1]
    [[x1,y1],[x2,y2]] = l1
    [[x3,y3],[x4,y4]] = l2
    l = [[[x1,y1,x2,y2]], [[x3,y3,x4,y4]]]
    image_with_lines = drow_the_lines(image, l)
    x, y = findIntersection(sorted_lines[0], sorted_lines[-1], height)
    return math.atanh((width/2 - x)/(height - y))

def img_to_cv2(image_msg):
    """
        Convert the image message into a cv2 image (numpy.ndarray)
        to be able to do OpenCV operations in it.
        :param Image or CompressedImage image_msg: the message to transform
    """
    rospy.loginfo("image is of type: " + str(type(image_msg)))
    type_as_str = str(type(image_msg))
    if type_as_str.find('sensor_msgs.msg._CompressedImage.CompressedImage') >= 0:
        # Image to numpy array
        np_arr = np.fromstring(image_msg.data, np.uint8)
        # Decode to cv2 image and store
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    elif type_as_str.find('sensor_msgs.msg._Image.Image') >= 0:
        # Use CvBridge to transform
        try:
            return self.bridge.imgmsg_to_cv2(image_msg, image_msg.encoding)  # "bgr8"
        except CvBridgeError as e:
            rospy.logerr("Error when converting image: " + str(e))
            return None
        else:
            rospy.logerr("We don't know how to transform image of type " + str(type(image_msg)) + " to cv2 format.")
            return None

class DemoNode(object):
    def __init__(self):
        self.node_name = "LineDetectorNode"
        self.sub_image = rospy.Subscriber("/None/corrected_image/compressed", CompressedImage, self.cbImage, queue_size=1)
        self.pub_cmd = rospy.Publisher("/None/car_cmd", Twist2DStamped, queue_size=1)



    def cbImage(self, image_msg):
        msg = Twist2DStamped()
        msg.v = 0.1
        bridge = cv_bridge.CvBridge()
        np_arr = np.fromstring(image_msg.data, np.uint8)
#        img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img = img_to_cv2(image_msg)
        msg.omega = find_angle(img)
        self.pub_cmd.publish(msg)

if __name__ == '__main__': 
    rospy.init_node('demo',anonymous=False)
    demo_node = DemoNode()
    rospy.spin()


