#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np

class webcam:
    def __init__(self):
        print("Initializing")
        #self.image_pub = rospy.Publisher("crosshair_image", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("cv_camera/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #scale_percent = 100
        #width = int(cv_image.shape[1] * scale_percent / 100)
        #height = int(cv_image.shape[0] * scale_percent / 100)
        #dim = (width, height)

        cv_image = rotateImage(cv_image, 90)

        height, width = cv_image.shape[:2]

        print('Height: ', height, 'Width: ', width)

        cv2.line(cv_image, (width/2,0), (width/2, height), (0, 0, 0), 3)    

        #cv_image = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)

        cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image window", width, height)
        #dim = (int(abs(windowWidth)), int(abs(windowHeight)))
        #resized = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)
        #windowDetails = cv2.getWindowProperty("Image window")
        #print(windowDetails)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def rotateImage(image, angle):
        #image_center = tuple(np.array(image.shape[1::-1]) / 2)
        #rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        #result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        #rows,cols = image.shape
        #M = cv2.getRotationMatrix2D((cols/2, rows/2), angle, 1)
        #dst = cv2.warpAffine(image,M,(cols,rows))

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

  


