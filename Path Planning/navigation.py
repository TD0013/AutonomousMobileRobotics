#!/usr/bin/env python2

from os import linesep
from numpy.core.numeric import Inf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Main():
    def __init__(self):
        self.subPos = rospy.Subscriber("/gps/fix", NavSatFix, self.PosCallback)
        self.subGoal = rospy.Subscriber("/gpsGoal/fix", NavSatFix, self.GoalCallback)
        sub = rospy.Subscriber ('/custom2/odom', Odometry, self.get_rotation)
        self.cmd_vel_pub = rospy.Publisher('/custom2/cmd_vel', Twist, queue_size=1)

        rospy.sleep(0.5)

        self.Loop()

    def get_rotation (self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        
        
    def PosCallback(self, msg):
        self.currentLatitude = msg.latitude
        self.currentLongitude = msg.longitude


    def GoalCallback(self, msg):
        self.goalLatitude = msg.latitude
        self.goalLongitude = msg.longitude

    def heading(self):
        self.x = self.goalLatitude - self.currentLatitude
        self.y = self.goalLongitude - self.currentLongitude

        self.angular = math.atan2(self.y,self.x)

        self.linear = math.sqrt(self.x**2 + self.y**2)



    def Loop(self):
        while not rospy.is_shutdown():
            self.heading()

            print(self.angular+self.yaw, self.linear)

            self.motion()

            rospy.sleep(0.1)

    def motion(self):

        self.cmd_vel_pub.publish(Twist())

        self.move = Twist()
        if self.linear*10000 > 0.4:
            self.move.linear.x = self.linear*10000
            self.move.angular.z = -(self.angular+self.yaw)/3.14*5

        elif self.linear*10000 > 0.1:
            self.move.linear.x = 0.3
            self.move.angular.z = -(self.angular+self.yaw)/3.14*5

        else:
            pass

        print(self.move)

        self.cmd_vel_pub.publish(self.move)



if __name__ == '__main__':
    print(" start!!")
    rospy.init_node("Avoid")
    main = Main()
    print(" Done!!")