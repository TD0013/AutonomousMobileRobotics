#!/usr/bin/env python2

from numpy.core.defchararray import array
from numpy.core.numeric import Inf, Infinity
import rospy
from geometry_msgs.msg import Twist
from rospy.names import resolve_name
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from nav_msgs.msg import OccupancyGrid

from sys import maxsize

import matplotlib.pyplot as plt




class Map:
    def __init__(self):
        print("Map init")
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.sleep(1)

    def scan_callback(self,msg):
        
        n = 360 #number of entries
        self.Range = [0]*n
        for i in range(-(n-1)/2, (n+1)/2):
            #print(i)
            
            if i > 0 or i == 0:
                if msg.ranges[i] == float('inf'):
                    self.Range[i] = 30
                else: 
                    self.Range[i] = msg.ranges[i]
            else:
                if msg.ranges[i] == float('inf'):
                    self.Range[i] = 30  
                else: 
                    self.Range[i] = msg.ranges[360+i]
            


    def PlotData(self): 
    
    
        self.x = [0]*len(self.Range)
        self.y = [0]*len(self.Range)
        
        for i in range(-(len(self.Range)-1)/2, (len(self.Range)+1)/2):
            self.x[i] = -(self.Range[i]*np.sin(i*np.pi/180))
    

            self.y[i] = -(self.Range[i]*np.cos(i*np.pi/180))


            
            
            #print (i, self.y[i], self.x[i])

            
            
        self.x[0] = 0 


    def ObstacleMap(self, res, graph):


        self.PlotData()

        cells = 1/res
        height = int((abs(int(max(self.y)) * 2)+10) * cells)
        width = int((abs(int(max(self.x)) * 2) +10) * cells)
        current = [height/2, width/2]

        #print(height, width, current)
        
        self.obstacle_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]
       
        

        for i in range(-(len(self.Range)-1)/2, (len(self.Range)+1)/2):
            #print(i, int(self.y[i]+current[0]),int(self.x[i]+int(current[1])))
        

            #obstacle cell
            self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))] = 100
            
            
            #nearby cells
            self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))] = max(self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))+1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))+1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))+1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))+1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))+1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))+1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))] = max(self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))-1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))-1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))-1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))-1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))-1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))-1], 50)


            #print(int(self.y[i]*cells+current[0]), int(self.x[i]*cells+int(current[1])))

            #print(i, int(self.x[i]*cells+current[1]),self.y[i]*cells+current[0])
            #assign unoccupied grid value
            #m = (self.y[i]- current[0])/()
            for x in range(min(int(current[1]), int(self.x[i]*cells+current[1])), max(int(current[1]), int(self.x[i]*cells+current[1]))):
                y = (((self.y[i])*x)+(self.x[i])*current[0]-self.y[i]*current[1])/(self.x[i])

                #print(x,y)

                self.obstacle_map[int(y)][int(x)] = max(self.obstacle_map[int(y)][int(x)], 0)

            for y in range(min(int(current[0]), int(self.y[i]*cells+current[0])), max(int(current[0]), int(self.y[i]*cells+current[0]))):
                x = (((self.x[i])*y)+(-self.x[i])*current[0]+self.y[i]*current[1])/(self.y[i])

                #print(x,y)

                self.obstacle_map[int(y)][int(x)] = max(self.obstacle_map[int(y)][int(x)], 0)



            if rospy.is_shutdown():
                print("break")
                break
        self.matrixTOarray(height,width)
        #graph.updateMap(self.obstacle_map)
        return self.local_map, height, width, current 

    def matrixTOarray(self, height, width):
        self.local_map = [-1]*(width*height)
        #print(len(self.local_map))
        i = width*height
        
        for a in range(0, width) :
            for b in range(0, height) :
                i-=1
                self.local_map[i] = int(self.obstacle_map[b][a])
                

class GPS:
    def __init__(self):
        print("GPS init")
        self.i=1
        self.j=1
        rospy.Subscriber ('/odom', Odometry, self.get_rotation)


        rospy.sleep(1)

   
    def get_rotation (self,msg):
        self.odometry_p = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        if self.j ==1:
            self.startx = msg.pose.pose.orientation.x
            self.starty = msg.pose.pose.orientation.y
            self.j =0

class Graph:
    def __init__(self):
        plt.ion()
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)
        
        
        plt.show()
        
 
    def updateObstacleMap(self, obstacle_map):
        self.scat1 = self.ax.matshow(obstacle_map, cmap= 'gray_r')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def updateMap(self, map):
        self.scat1 = self.ax2.matshow(map)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def loop():
        gps = GPS
        localMap = OccupancyGrid()
        graph = Graph()
        map = Map()
        while not rospy.is_shutdown():
            
            
            resolution = 0.05
            #Goalx,Goaly, Posx, Posy = gps.heading()
            
            

            l_map, height, width, current = map.ObstacleMap(resolution, graph)

            localMap.header.frame_id = "base_link"
            localMap.info.height = width
            localMap.info.width = height 
            localMap.info.resolution = resolution
            localMap.data = l_map
            localMap.info.origin.position.x = -current[0]*resolution
            localMap.info.origin.position.y = -current[1]*resolution
            
            
            test = rospy.Publisher("/localMap", OccupancyGrid, queue_size=1)
            test.publish(localMap)
            
            #print("published Map")

            rospy.sleep(0.001)

if __name__ == '__main__':
    rospy.init_node("localMap")
    loop()