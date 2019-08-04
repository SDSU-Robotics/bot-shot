#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float64
import math
import constant

class Listener:
    _distance = 5
    _angle = math.radians(34.5)
    def distanceCallback(self, msg):
        _distance = msg.data

    def angleCallback(self, msg):
        _angle = math.radians(msg.data)

    def calculateRPM(self):
        num = 4.9 * self._distance * self._distance
        den = math.cos(self._angle) * math.cos(self._angle) * self._distance * math.tan(self._angle) + constant.LAUNCH_HEIGHT - constant.HOOP_HEIGHT
        V = math.sqrt(num / den)
        keb = 0.5 * constant.BASKETBALL_MASS * V * V
        kew = 0.5 * keb / constant.LAUNCH_ENERGY_TRANSFER
        w = math.sqrt(2 * kew / constant.WHEEL_INERTIA)
        rpm = w / 0.104719755
        return rpm


def physicsModel():
    rospy.init_node('physics_model', anonymous=True)
    rate = rospy.Rate(10) # 10 hz
    listener = Listener()

    rpm_pub = rospy.Publisher('calculated_rpm', Float64, queue_size=1000)
    rospy.Subscriber('distance', Float64, listener.distanceCallback)
    rospy.Subscriber('angle', Float64, listener.angleCallback)

    while (1):
		rpm_pub.publish(listener.calculateRPM())
		rate.sleep()

if __name__ == '__main__':
    try: physicsModel()
    except rospy.ROSInterruptException: pass