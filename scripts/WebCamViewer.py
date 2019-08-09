#!/usr/bin/env python

import rospy
import sys
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64
import numpy as np
import constant

class Webcam:
    distance = 0
    setRPM = 0.0
    actualBotRPM = 0.0
    actualTopRPM = 0.0
    cursorAdjustment = -676
    setAngle = 0
    actualAngle = 0
    calculatedRPM = 0
    calculatedAngle=0
    def __init__(self):

        print("Initializing Computer Vision")
        self.bridge = CvBridge()

        rospy.Subscriber("cv_camera/image_raw", Image, self.callback)
        rospy.Subscriber("cursor_adjustment", Float64, self.cursor_callback)
        rospy.Subscriber("set_RPM", Float64, self.set_rpm_callback)
        rospy.Subscriber("bot_RPM_reading", Float64, self.set_top_rpm_callback)
        rospy.Subscriber("top_RPM_reading", Float64, self.set_bot_rpm_callback)
        rospy.Subscriber("set_angle", Int64, self.set_angle_callback)
        rospy.Subscriber("angle_pos", Float64, self.set_actual_angle_callback)
        rospy.Subscriber("calculated_rpm", Float64, self.set_calculated_rpm_callback)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            imageProcessing(image)
        except CvBridgeError as e:
            print(e)

    def cursor_callback(self, msg):
        Webcam.cursorAdjustment += msg.data / 20.0

    def set_rpm_callback(self, msg):
        Webcam.setRPM = msg.data
            
    def set_top_rpm_callback(self, msg):
        Webcam.actualTopRPM = msg.data
    
    def set_bot_rpm_callback(self, msg):
        Webcam.actualBotRPM = msg.data
    
    def set_angle_callback(self, msg):
        Webcam.setAngle = msg.data

    def set_actual_angle_callback(self, msg):
        Webcam.actualAngle = msg.data
    
    def set_calculated_rpm_callback(self, msg):
        Webcam.calculatedRPM = msg.data
            


def imageProcessing(image):
     
        # Get background image and width/height ratio
        background = cv2.imread('/home/robotics/catkin_ws/src/bot-shot/assets/backdrop.jpg')
        backgroundWidth, backgroundHeight = background.shape[:2]
        ratio = float(backgroundWidth) / float(backgroundHeight)

        # Scale video, and rotate
        scale = 200
        w = int(image.shape[1] * scale / 100)
        h = int(image.shape[0] * scale / 100)
        dim = (w, h)
        image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        cv_image = rotateImage(image, 90)
        height, width = cv_image.shape[:2]

        # Calc height
        cursorHeight = (height) + int(Webcam.cursorAdjustment)
        calcDistance(cursorHeight, height)       

        # Create the GUI
        createGUI(background)
        
        # Create the crosshairs
        cv2.line(cv_image, ((width/2) + constant.ADJUST_CENTER_LINE, 0), ((width/2) + constant.ADJUST_CENTER_LINE, height), (255, 0, 0), constant.CROSSHAIR_WEIGHT)    
        cv2.line(cv_image, (0, height - cursorHeight), (width, height - cursorHeight), (255, 0, 0), constant.CROSSHAIR_WEIGHT)    

        # Adjust the size of the background
        newWidth = int(round(height / ratio))
        dim = (newWidth, height)
        background = cv2.resize(background, dim, interpolation = cv2.INTER_AREA)
        height, width =  background.shape[:2]

        # Overlay the image
        x_offset = 0
        y_offset = 0
        background[y_offset:y_offset+cv_image.shape[0], x_offset:x_offset+cv_image.shape[1]] = cv_image

        cv2.namedWindow("Bot Shot Champions", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Bot Shot Champions", width, height)

        cv2.imshow("Bot Shot Champions", background)
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

        if Webcam.setRPM == 0:
            targetTop = 0
            targetBottom = 0
        else:
            targetTop = Webcam.setRPM - constant.RPM_DIFFERENCE / 2
            targetBottom = Webcam.setRPM + constant.RPM_DIFFERENCE / 2

            
        printData(background, "Distance to Hoop: ", str(distanceToPrint) + " feet", constant.DISTANCE_LOCATION) 
        if Webcam.calculatedRPM <= -1:
            printData(background, "Calculated RPM: ", "invalid", constant.CALCULATED_RPM_LOCATION)
        else:
            printData(background, "Calculated RPM: ", str(int(Webcam.calculatedRPM)) + " RPM", constant.CALCULATED_RPM_LOCATION)
        printData(background, "Target Average RPM: ", str(int(Webcam.setRPM)) + " RPM", constant.DESIRED_AVG_RPM_LOCATION)
        printData(background, "Target Top RPM: ", str(int(targetTop)) + " RPM", constant.DESIRED_TOP_RPM_LOCATION)
        printData(background, "Target Bottom RPM: ", str(int(targetBottom)) + " RPM", constant.DESIRED_BOTTOM_RPM_LOCATION)
        printData(background, "Actual Top RPM: ", str(int(Webcam.actualTopRPM)) + " RPM", constant.ACTUAL_TOP_RPM_LOCATION)
        printData(background, "Actual Bottom RPM: ", str(int(Webcam.actualBotRPM)) + " RPM", constant.ACTUAL_BOTTOM_RPM_LOCATION)
        printData(background, "Target Angle: ", str(round((Webcam.setAngle / 4096.0 / 100.0 / 85.0 * 42.0 * 360.0 + 34.5), 2)) + " deg", constant.DESIRED_ANGLE_LOCATION)
        printData(background, "Actual Angle: ", str(round(Webcam.actualAngle, 1)) + " deg", constant.ACTUAL_ANGLE_LOCATION)
        

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
    #except rospy.ROSInterruptException: pass


  


