#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge, numpy
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.i = int()
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("image_window",1)
        
        print("initiating frames")
        self.frames = 0
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)

        rospy.sleep(0.5)
        
                
        #self.find_line()


        self.Loop()
        #rospy.sleep(0.5)


        #self.Loop()
        #self.image_process()
        #cv2.imshow("mask_window", self.mask)
        
    
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frames +=1
        print("frames ",self.frames)
       
        #cv2.imshow("image_window", self.image)
        #cv2.waitKey(1)
        #print(self.image - self.image1)
        print("data updated")


        

    def Loop(self):
            while not rospy.is_shutdown():
                print(self.i) 
                self.i += 1
                #print(self.image)
                cv2.imshow("image_window", self.image)
                cv2.waitKey(30)                 
 
                print("resetting frames")
                self.frames = 0
                rate = rospy.Rate(30)
                rate.sleep() 


    def image_process(self):
        self.hsv = cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV)
        self.lower_yellow = numpy.array([20, 0, 140])
        self.upper_yellow = numpy.array([ 45, 255, 255])
        self.mask = cv2.inRange(self.hsv, self.lower_yellow, self.upper_yellow)
        self.masked = cv2.bitwise_and(self.image, self.image, mask=self.mask)
        cv2.namedWindow("mask_window",1)
        

    def find_line(self):
        self.h, self.w, self.d = self.image.shape
        self.search_top = 3*self.h/4
        self.search_bot = self.search_top + 20
        self.mask[0:self.search_top, 0:self.w] = 0
        self.mask[self.search_bot:self.h, 0:self.w] = 0
        self.M = cv2.moments(self.mask)
        #if self.M['m00'] > 0:
        #    self.cx = int(self.M['m10']/self.M['m00'])
        #    self.cy = int(self.M['m01']/self.M['m00'])
        #cv2.circle(self.image, (self.cx, self.cy), 20, (0,0,255), -1)
        self.masked = cv2.bitwise_and(self.image, self.image, mask=self.mask)
        cv2.imshow("mask_window_2", self.masked)



if __name__=="__main__":
    rospy.init_node('follower')
    follower = Follower()
    rospy.spin()
