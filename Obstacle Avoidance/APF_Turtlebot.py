#!/usr/bin/env python2

from numpy.core.numeric import Inf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import math
import numpy as np


class Main():
    def __init__(self):

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.sleep(0.5) #wait for the scan to have values

        self.i = 1
        self.initGraph()

        self.Loop()

    def scan_callback(self,msg):
        
        n = 61 #number of entries
        self.Range = [0]*n
        for i in range(-(n-1)/2, (n+1)/2):
            #print(i)
            if i > 0 or i == 0:
                self.Range[i] = msg.ranges[i]
            else:
                self.Range[i] = msg.ranges[360+i]
        

    def PlotData(self):
    
    
        self.x = [0]*len(self.Range)
        self.y = [0]*len(self.Range)
        
        for i in range(-(len(self.Range)-1)/2, (len(self.Range)+1)/2):
            self.x[i] = -self.Range[i]*np.sin(i*np.pi/180)
            self.y[i] = self.Range[i]*np.cos(i*np.pi/180)

            
        self.x[0] = 0 
        

    def initGraph(self):

        self.PlotData()
        self.Forces()

        # You probably won't need this if you're embedding things in a tkinter plot...
        plt.ion()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)
    
        self.ax.set_ylim(30,-30)
        self.ax.set_xlim(-30,30)

        

        
        plt.show()
        
        self.scat1 = self.ax.scatter(self.x, self.y) # Returns a tuple of line objects, thus the comma
        
        self.plotForce = self.ax2.arrow(0, 0, (300 - (self.netFy))/100, self.netFx, width = 0.5)

    def Forces(self):
        self.fx = [0]*len(self.Range)
        self.fy = [0]*len(self.Range)
        self.netFx = 0
        self.netFy = 0
        self.size = [0]*len(self.Range)

        for i in range(-(len(self.Range)-1)/2, (len(self.Range)+1)/2):
            
            if self.x[i] != 0:
                #print("true", i)
                self.fx[i] = 1/(self.x[i])
            else:
                #print("Else", self.x[i],"  ", i)
                self.fx[i] = 0

            if self.y[i] != 0:
                self.fy[i] = 1/(self.y[i])
            else:
                self.fy[i] = 0
        
        for l in range(0, len(self.fx)):
            #print(l, ":", self.x[l], "\n")
            self.netFx += self.fx[l]

        for l in range(0, len(self.fy)):
        
            self.netFy += self.fy[l]

        for i in range(-(len(self.Range)-1)/2, (len(self.Range)+1)/2):
            self.size[i] = (self.fx[i]**2 + self.fy[i]**2)**0.5
            

    def Motion(self):
        stp = Twist()
        self.cmd_vel_pub.publish(stp)

        self.move = Twist()
        self.move.linear.x = (100 - (self.netFy))/100
        self.move.angular.z = self.netFx/300

        
        return self.move

        
            


    def Loop(self):
        while not rospy.is_shutdown():

            self.PlotData()
            self.Forces()

            #update x and y
            self.ax.cla()
            self.ax.set_ylim(-1,3)
            self.ax.set_xlim(-1,1)
            self.scat1 = self.ax.scatter(self.x, self.y)
            
            self.ax2.cla()
            self.ax2.set_ylim(300,-300)
            self.ax2.set_xlim(-300,300)
            self.plotForce = self.ax2.arrow(0, 0, -self.netFx, ( (self.netFy)), width = 5)
            #print (self.i)
            #plot x and y
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            #self.i += 1

            

            self.velocities = self.Motion()

            self.cmd_vel_pub.publish(self.velocities)


            print ("Fx",-self.netFx,"\n" , "Fy", -self.netFy)

            rospy.sleep(0.5)



if __name__ == '__main__':
    print(" start!!")
    rospy.init_node("Avoid")
    main = Main()
    print(" Done!!")