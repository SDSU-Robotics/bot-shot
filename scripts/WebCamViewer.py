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

class Webcam:
    distance = 0
    setRPM = 1000
    actualBotRPM = 1000
    actualTopRPM = 1000
    cursorAdjustment = 676

    def __init__(self):

        print("Initializing Computer Vision")
        self.bridge = CvBridge()

        rospy.Subscriber("cv_camera/image_raw", Image, self.callback)
        rospy.Subscriber("cursor_adjustment", Float64, self.cursor_callback)
        rospy.Subscriber("set_RPM", Float64, self.set_rpm_callback)
        rospy.Subscriber("top_RPM_reading", Float64, self.set_top_rpm_callback)
        rospy.Subscriber("bot_RPM_reading", Float64, self.set_bot_rpm_callback)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            imageProcessing(image)
        except CvBridgeError as e:
            print(e)

    def cursor_callback(self, msg):
        Webcam.cursorAdjustment += msg.data / 10.0

    def set_rpm_callback(self, msg):
        Webcam.setRPM = msg.data
            
    def set_top_rpm_callback(self, msg):
        Webcam.actualTopRPM = msg.data
    
    def set_bot_rpm_callback(self, msg):
        Webcam.actualBotRPM = msg.data
        

def imageProcessing(image):
     
        background = cv2.imread('/home/robotics/catkin_ws/src/bot-shot/assets/backdrop.jpg')
        
        scale = 200
        w = int(image.shape[1] * scale / 100)
        h = int(image.shape[0] * scale / 100)
        dim = (w, h)
        image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)

        cv_image = rotateImage(image, 90)

        height, width = cv_image.shape[:2]
        #print height

        #print('Height: ', height, 'Width: ', width)
 
        cursorHeight = (height) + int(Webcam.cursorAdjustment)

        calcDistance(cursorHeight, height)

        createGUI(background)
        
        cv2.line(cv_image, ((width/2) + constant.ADJUST_CENTER_LINE, 0), ((width/2) + constant.ADJUST_CENTER_LINE, height), (0, 0, 0), 3)    
        cv2.line(cv_image, (0, height - cursorHeight), (width, height - cursorHeight), (0, 0, 0), 3)    

        dim = (2000, height)
        background = cv2.resize(background, dim, interpolation = cv2.INTER_AREA)
        height, width =  background.shape[:2]

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

def createGUI(background):

        distanceToPrint = Webcam.distance * 3.28084
        distanceToPrint = round(distanceToPrint, 3)
        
        printData(background, "Distance to hoop", str(distanceToPrint) + " feet", constant.DISTANCE_LOCATION) 
        #printData(background, "")
        """
        text = "Distance to hoop:  "
        currentTextLocation = 200
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = str(distanceToPrint) + " feet"
        cv2.putText(background, text, (constant.DATA_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Desired RPM: "
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = str(Webcam.setRPM) + " RPM"
        cv2.putText(background, text, (constant.DATA_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)
               
        text = "Actual Top RPM: " 
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = str(Webcam.actualTopRPM) + " RPM"
        cv2.putText(background, text, (constant.DATA_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Actual Bottom RPM: " 
        currentTextLocation += 100
        cv2.putText(background, text, (900, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = str(Webcam.actualBotRPM) + " RPM"
        cv2.putText(background, text, (constant.DATA_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Desired Angle:  " 
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Set Angle: "
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Actual RPM: "
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Desired Angle:  "
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Set Angle: "
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)
        """
def printData(background, text, data, textLocation):        
        cv2.putText(background, text, (constant.TEXT_LOCATION, textLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)
        cv2.putText(background, data, (constant.DATA_LOCATION, textLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

def calcDistance(cursorHeight, height):
    dist_pub = rospy.Publisher("distance", Float64, queue_size = 1000)

    global dist_pub
    #print("Calculating Distance")

    falseTheta = ((float(cursorHeight) / float(height)) * constant.THETA_FOV) 
    theta = falseTheta + (constant.THETA_MOUNT - (.5*constant.THETA_FOV))
    distance = (constant.HOOP_HEIGHT - constant.CAMERA_HEIGHT ) / math.tan(math.radians(theta))
    #print("CursorHeight:", cursorHeight, "Height:", height, "FalseTheta:", falseTheta, "Theta:", theta, "Distance:", distance)

    #print(distance)
    dist_pub.publish(distance)
    Webcam.distance = distance

def main(args):
    Webcam()
    rospy.init_node('webcam', anonymous=True)
    try:
        rospy.spin()        
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

  


