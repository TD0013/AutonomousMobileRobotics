#!/usr/bin/env python

import rospy
import os
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
import matplotlib as plt
import matplotlib.image as mpimg
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        print("__init__")

        self.i = 0
        self.twist = Twist()


        self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)

        rospy.sleep(1)

        self.Loop()

        print("Print before Endong")


        

        
        #cv2.imwrite('/home/td0013/general_ws/src/beginner_tutorials/Images/savedImage.jpg', self.image)

    def Loop(self):
        while not rospy.is_shutdown():
            self.Function(self.image)

            self.Move()
            rate = rospy.Rate(20)
            rate.sleep()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
         
        #print(self.i)
        #self.i += 1


        #cv2.imshow("image_window", self.image)
        #cv2.waitKey(1)        

        #print("data updated")

    def image_process(self,img):
        self.hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        self.lower_yellow = np.array([20, 0, 140])
        self.upper_yellow = np.array([ 45, 255, 255])
        self.mask = cv2.inRange(self.hsv, self.lower_yellow, self.upper_yellow)
        self.masked = cv2.bitwise_and(img, img, mask=self.mask)

        #cv2.imshow("mask_window", self.mask)
        #cv2.waitKey(1)

        self.Edges = self.mask
        
    def region_of_interest(self, img, vertices):
        print("region of interest")

        self.mask = np.zeros_like(img) # Define a blank matrix that matches the image height/width.
        self.channel_count = img.shape[2] # Retrieve the number of color channels of the image.
        self.match_mask_color = (255,) * self.channel_count # Create a match color with the same color channel counts.
        cv2.fillPoly(self.mask, vertices, self.match_mask_color) # Fill inside the polygon
        self.masked_image = cv2.bitwise_and(img, self.mask) # Returning the image only where mask pixels match

        cv2.imshow("RoI", self.masked_image)
        cv2.waitKey(30)
 
        return self.masked_image
    
    def Function(self, img):
        print("Function")
        print(int)

        self.height = img.shape[0]
        self.width = img.shape[1]
        

        self.region_of_interest_vertices = [
            (0, self.height),
            (self.width / 2, self.height / 2),
            (self.width, self.height),
        ]

        #print(self.region_of_interest_vertices)

        self.cropped_image = self.region_of_interest(img, np.array([self.region_of_interest_vertices], np.int32))
        print(self.cropped_image.shape)

        #plt.imshow(self.masked_image)
        #plt.show()
          

        #self.Canny(self.masked_image)
        self.image_process(self.masked_image)

    def Canny(self, img):
        self.Edges = cv2.Canny(img, 100, 200)
        cv2.imshow("mask_window", self.Edges)

    def Move(self):
        print("Move")
        self.M = cv2.moments(self.Edges)
        
        print(self.M['m00'])
        if self.M['m00'] > 0:
            self.cx = int(self.M['m10']/self.M['m00'])
            self.cy = int(self.M['m01']/self.M['m00'])
            cv2.circle(self.image, (self.cx, self.cy), 20, (0,0,255), -1)
            print("Should Stop")
            cv2.imshow("Move", self.image)
            cv2.waitKey(30)
            self.follow_line()

            
        else:    
            rospy.sleep(0.1)
            
            self.twist.linear.x = 0

            if self.twist.angular.z == 0:

                self.twist.angular.z = 0.1

            
            self.cmd_vel_pub.publish(self.twist)   

            #self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)

            #cv2.imshow("image_window", self.image)
            


            
            
            
        #print("loop skip?")
               

    def follow_line(self):
        print("follow_line")
        self.err = self.cx - self.width/2
        print(self.err)
        
        self.twist.linear.x = 0.1
        self.twist.angular.z = -float(self.err) / 100
        self.cmd_vel_pub.publish(self.twist)    

    def image_save(self):
        self.directory = r'/home/td0013/general_ws/src/beginner_tutorials/Images'
        #filename = 'savedImage.jpg'
        cv2.imwrite('/home/td0013/general_ws/src/beginner_tutorials/Images/savedImage.jpg', self.image)
        print(os.listdir(self.directory))
        


if __name__=="__main__":
    rospy.init_node('Line_follower')
    follower = Follower()
    rospy.spin()
