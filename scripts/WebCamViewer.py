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
        global setRPM
        global actualTopRPM
        global actualBotRPM


        print("Initializing")
        self.bridge = CvBridge()
        cursorAdjustment = 0
        setRPM = 1000
        actualBotRPM = 1000
        actualTopRPM = 1000

        rospy.Subscriber("cv_camera/image_raw", Image, self.callback)
        rospy.Subscriber("cursor_adjustment", Float64, self.cursor_callback)
        rospy.Subscriber("set_RPM", Float64, self.set_rpm_callback)
        rospy.Subscriber("top_RPM_reading", Float64, self.set_top_rpm_callback)
        rospy.Subscriber("bot_RPM_reading", Float64, self.set_bot_rpm_callback)

        dist_pub = rospy.Publisher("distance", Float64, queue_size = 1000)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            imageProcessing(image)
        except CvBridgeError as e:
            print(e)

    def cursor_callback(self, msg):
        global cursorAdjustment 
        cursorAdjustment += msg.data / 10.0
        #print(cusorAdjustment)

    def set_rpm_callback(self, msg):
        global setRPM
        setRPM = msg.data
    
    def set_top_rpm_callback(self, msg):
        global actualTopRPM
        setRPM = msg.data
    
    def set_bot_rpm_callback(self, msg):
        global actualBotRPM
        setRPM = msg.data
        

def imageProcessing(image):

        background = cv2.imread('/home/robotics/catkin_ws/src/bot-shot/assets/backdrop.jpg')   
        
        global cursorAdjustment
        global setRPM
        global actualTopRPM
        global actualBotRPM

        cv_image = rotateImage(image, 90)

        height, width = cv_image.shape[:2]
        #print height

        #print('Height: ', height, 'Width: ', width)
 
        cursorHeight = (height) + int(cursorAdjustment)

        #print(cursorHeight)

        distanceToPrint = getDistance(cursorHeight, height) 

        

        distanceToPrint = distanceToPrint * 3.28084
        distanceToPrint = round(distanceToPrint, 3)
        createGUI(background, distanceToPrint)
        
        cv2.line(cv_image, ((width/2) + constant.ADJUST_CENTER_LINE, 0), ((width/2) + constant.ADJUST_CENTER_LINE, height), (0, 0, 0), 3)    
        cv2.line(cv_image, (0, 1393 - cursorHeight), (width, 1393 - cursorHeight), (0, 0, 0), 3)    

        dim = (2000, height)
        background = cv2.resize(background, dim, interpolation = cv2.INTER_AREA)

        height, width = background.shape[:2]

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

def createGUI(background, distanceToPrint):
        global setRPM
        global actualTopRPM
        global actualBotRPM

        
        text = "Distance to hoop:  "
        currentTextLocation = 200
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = str(distanceToPrint) + " feet"
        cv2.putText(background, text, (constant.DATA_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Desired RPM: "
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = str(setRPM) + " RPM"
        cv2.putText(background, text, (constant.DATA_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)
               
        text = "Actual Top RPM: " 
        currentTextLocation += 100
        cv2.putText(background, text, (constant.TEXT_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = str(actualTopRPM) + " RPM"
        cv2.putText(background, text, (constant.DATA_LOCATION, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = "Actual Bottom RPM: " 
        currentTextLocation += 100
        cv2.putText(background, text, (900, currentTextLocation), cv2.FONT_HERSHEY_DUPLEX, constant.TEXT_SIZE, constant.TEXT_COLOR, constant.TEXT_WEIGHT)

        text = str(actualBotRPM) + " RPM"
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



def getDistance(cursorHeight, height):
    global dist_pub
    #print("Calculating Distance")
    falseTheta = ((float(cursorHeight) / float(height)) * constant.THETA_FOV) 
    theta = falseTheta + (constant.THETA_MOUNT - (.5*constant.THETA_FOV))
    distance = (constant.HOOP_HEIGHT - constant.LAUNCH_HEIGHT ) / math.tan(math.radians(theta))
    #print("CursorHeight:", cursorHeight, "Height:", height, "FalseTheta:", falseTheta, "Theta:", theta, "Distance:", distance)

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

  


