#!/usr/bin/env python2

from numpy.core.numeric import Inf, Infinity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

from sys import maxsize

import matplotlib.pyplot as plt

class Main():
    def __init__(self):
        self.i = 1
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.subPos = rospy.Subscriber("/gps/fix", NavSatFix, self.PosCallback)
        self.subGoal = rospy.Subscriber("/gpsGoal/fix", NavSatFix, self.GoalCallback)
        sub = rospy.Subscriber ('/custom2/odom', Odometry, self.get_rotation)
        self.cmd_vel_pub = rospy.Publisher('/custom2/cmd_vel', Twist, queue_size=1)

        rospy.sleep(2) #wait for the scan to have values

        
        self.initGraph()
        
        self.Loop()

    def scan_callback(self,msg):
        
        n = 181 #number of entries
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

        

        # You probably won't need this if you're embedding things in a tkinter plot...
        plt.ion()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)
 
        plt.show()
        
        #self.scat1 = self.ax.matshow(self.obstacle_map, cmap=plt.cm.Blues)
        #self.scat2 = self.ax.matshow(self.position, cmap=plt.cm.Reds) 
        

   
    def ObstacleMap(self, height,width, start, current, goal):
        
        self.obstacle_map = [[0 for z in range(0, width)] for z in range(0, height)]
        
        print(start[0], start[1])
        if goal[0]<0 or goal[1]<0:
            self.Loop()
        self.obstacle_map[start[0]][start[1]] = 80
        self.obstacle_map[current[0]][current[1]] =90
        self.obstacle_map[goal[0]][goal[1]] = 100
                
        for i in range(-(len(self.Range)-1)/2, (len(self.Range)+1)/2):
            
            #print(i)
            if self.x[i] != float('inf') and self.y[i] != float('inf'):
                if int(self.y[i]*-10+current[0]) < 0 or int(self.x[i]*10+current[1])<0:
                    continue
                print("current",current[0], current[1])
                #print(self.x[i],self.y[i])
                self.obstacle_map[int(self.y[i]*-10+current[0])][int(self.x[i]*10+int(current[1]))] = 60
                
                print("obstacle map",int(self.y[i]*-10),int(height*0.75-int(self.Posx*1000000)))

                #self.map[self.x[i]][self.y[i]].set_state("#")

                if self.obstacle_map[int(self.y[i]*-10+current[0]+1)][int(self.x[i]*10+current[1])] != 60:
                    self.obstacle_map[int(self.y[i]*-10+current[0]+1)][int(self.x[i]*10+current[1])] = 50

                if self.obstacle_map[int(self.y[i]*-10+current[0]-1)][int(self.x[i]*10+current[1])] != 60:
                    self.obstacle_map[int(self.y[i]*-10+current[0]-1)][int(self.x[i]*10+current[1])] = 50

                if self.obstacle_map[int(self.y[i]*-10+current[0])][int(self.x[i]*10+current[1]+1)] != 60:
                    self.obstacle_map[int(self.y[i]*-10+current[0])][int(self.x[i]*10+current[1]+1)] = 50
                
                if self.obstacle_map[int(self.y[i]*-10+current[0])][int(self.x[i]*10+current[1]-1)] != 60:
                    self.obstacle_map[int(self.y[i]*-10+current[0])][int(self.x[i]*10+current[1]-1)] = 50
            

            #self.c1 = [[self.obstacle_map for z in range(-height, height)] for z in range(-width, width)]
            if rospy.is_shutdown():
                print("break")
                break
        print()
            

    def get_rotation (self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        
        
    def PosCallback(self, msg):
        self.currentLatitude = msg.latitude
        self.currentLongitude = msg.longitude
        if self.i ==1:
            self.startLatitude = msg.latitude
            self.startLongitude = msg.longitude
            self.i =0


    def GoalCallback(self, msg):
        self.goalLatitude = msg.latitude
        self.goalLongitude = msg.longitude

    def heading(self):
        self.Goalx = self.goalLatitude - self.startLatitude
        self.Goaly = self.goalLongitude - self.startLongitude

        self.Posx = self.currentLatitude - self.startLatitude
        self.Posy = self.currentLongitude - self.startLongitude

        self.angular = math.atan2(self.Goaly,self.Goalx)

        self.linear = math.sqrt(self.Goalx**2 + self.Goaly**2)

        print(self.Goalx, self.Goaly)
    def Loop(self):
        while not rospy.is_shutdown():
            print("entering loop")

            self.PlotData()
           
            self.heading()


            height = max(30,4*abs(int((self.Goalx-self.Posx)*1000000)))
            width = max(30,4*abs(int((self.Goaly-self.Posy)*750000)))          

            print(height, width)
            
            print("defining start and end")
            start = [int(height*0.75), int(width/2)]



            current = [int(height*0.75-int(self.Posx*1000000)), int(width/2+int(self.Posy*750000))]

            
            goal = [int(height*0.75-int(self.Goalx*1000000)),int(width/2+int(self.Goaly*750000))]
            print("start and goal:",start, goal)
            #print("compare", int(current[0]), int(height*0.75-int(self.Posx*1000000)))
            
            self.ObstacleMap(height, width, start,current, goal)
            
            print("Publishing Map")

            

            self.ax2.cla()
            self.ax2.sety = height
            self.ax2.setx = width 
            
               
            self.ax2.plot(0, 0, "og")
            self.ax2.plot(current[1]-start[1], current[0]-start[0], "or")
            self.ax2.plot(goal[1]-start[1], -goal[0]+start[0], "xb")
           
            self.scat1 = self.ax.matshow(self.obstacle_map)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
            #update x and y
            self.ax.cla()

            del self.obstacle_map[:]
          
            rospy.sleep(1)



if __name__ == '__main__':
    print(" start!!")
    rospy.init_node("Avoid")
    main = Main()
    print(" Done!!")