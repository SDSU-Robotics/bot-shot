#!/usr/bin/env python

import rospy
import sys
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float64
import numpy as np
import constant

class webcam:


    def __init__(self):
        global dist_pub
        global cursorAdjustment
        print("Initializing")
        self.bridge = CvBridge()
        cursorAdjustment = 0
        rospy.Subscriber("cv_camera/image_raw", Image, self.callback)
        rospy.Subscriber("cursor_adjustmant", Float64, self.cursor_callback)
        dist_pub = rospy.Publisher("calculated_distance", Float64, queue_size = 1000)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            imageProcessing(image)
        except CvBridgeError as e:
            print(e)

    def cursor_callback(self, msg):
        global cursorAdjustment 
        cursorAdjustment += int(msg.data)
        #print(cusorAdjustment)

def imageProcessing(image):

        background = cv2.imread('/home/robotics/catkin_ws/src/bot-shot/assets/backdrop.jpg')   
        
        global cursorAdjustment

        cv_image = rotateImage(image, 90)

        height, width = cv_image.shape[:2]

        #print('Height: ', height, 'Width: ', width)

        cursorHeight = (height/2) + cursorAdjustment

        getDistance(cursorHeight, height)

        cv2.line(cv_image, ((width/2) + constant.ADJUST_CENTER_LINE, 0), ((width/2) + constant.ADJUST_CENTER_LINE, height), (0, 0, 0), 3)    
        cv2.line(cv_image, (0, cursorHeight), (width, cursorHeight), (0, 0, 0), 3)    

        dim = (2000, height)
        background = cv2.resize(background, dim, interpolation = cv2.INTER_AREA)
        height, width = background.shape[:2]
        #print('Height: ', height, 'Width: ', width)

        x_offset = 0
        y_offset = 0
        background[y_offset:y_offset+cv_image.shape[0], x_offset:x_offset+cv_image.shape[1]] = cv_image

        cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image window", width, height)

        cv2.imshow("Image window", background)
        cv2.waitKey(1)

def rotateImage(image, angle):

        height, width = image.shape[:2]
        image_center = (width/2, height/2)
        rotation_image = cv2.getRotationMatrix2D(image_center, angle, 1)
        abs_cos = abs(rotation_image[0,0])
        abs_sin = abs(rotation_image[0,1])

        bound_w = int(height*abs_sin + width * abs_cos)
        bound_h = int(height*abs_cos + width + abs_sin)

        rotation_image[0, 2] += bound_w/2 - image_center[0]
        rotation_image[1, 2] += bound_h/2 - image_center[1]

        rotated_image = cv2.warpAffine(image, rotation_image, (bound_w, bound_h))

        return rotated_image

def getDistance(cursorHeight, height):
    global dist_pub
    #print("Calculating Distance")
    theta = ((cursorHeight / height) * 69) + constant.THETA_MOUNT
    distance = (constant.HEIGHT_OF_HOOP) / math.tan(math.radians(theta))
    #print(distance)
    dist_pub.publish(distance)
    return distance

def main(args):
    webcam()
    rospy.init_node('webcam', anonymous=True)
    try:
        rospy.spin()        
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

  


