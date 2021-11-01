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
        
        self.temp_h =0
	self.temp_w =0
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.sleep(1)

    def scan_callback(self,msg):

	#print("angle_min",msg.angle_min)
	#print("angle_max",msg.angle_max)
	#print("angle_increment",msg.angle_increment)
	#print("time_increment",msg.time_increment)
	#print("scan_time",msg.scan_time)
	#print("range_min",msg.range_min)
	#print("range_max",msg.range_max)
	#print("range_max",len(msg.ranges))

	self.ANGLE_MIN = int(round(msg.angle_min*180/math.pi))
	self.ANGLE_MAX = int(round(msg.angle_max*180/math.pi))
	self.ANGLE_TOT = self.ANGLE_MAX-self.ANGLE_MIN
	self.ANGLE_INC = msg.angle_increment
	N = len(msg.ranges)

	self.N_MIN = int(N*(float(self.ANGLE_MIN)/float(self.ANGLE_TOT)))
	self.N_MAX = int(N*(float(self.ANGLE_MAX)/float(self.ANGLE_TOT)))
	
        #print(self.ANGLE_MIN ,self.ANGLE_MAX, self.ANGLE_TOT, self.N_MIN, self.N_MAX)
         #number of entries
	self.Range =[0]*N 
		
	for i in range(0,N):
		if(np.isnan(msg.ranges[i])):
			self.Range[i+self.N_MIN] = msg.range_max
			#print(i+self.N_MIN, self.Range[i+self.N_MIN])
		else:
			self.Range[i+self.N_MIN] = msg.ranges[i]
		
		

			
            


    def PlotData(self): 
	
    
    
        self.x = [0]*len(self.Range)
        self.y = [0]*len(self.Range)
        
        for i in range(self.N_MIN, self.N_MAX):
            self.x[i] = -(self.Range[i]*np.sin(i*self.ANGLE_INC))
    

            self.y[i] = -(self.Range[i]*np.cos(i*self.ANGLE_INC))


            
        return self.x,self.y
            #print (i, self.y[i], self.x[i])

            
            
        #self.x[0] = 0 


    def ObstacleMap(self, res):

	RANGE = self.Range
        self.PlotData()

        cells = 1/res
        self.temp_h = max(abs(max(self.y)), abs(min(self.y)), self.temp_h)
        height = int(self.temp_h*2*cells) +10
        self.temp_w = max(abs(max(self.x)), abs(min(self.x)), self.temp_w)
        width = int(self.temp_w*2*cells) +10

        current = [height/2, width/2]

        #print(min(self.y), height, width, current)
        
        self.obstacle_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]
       
        

        for i in range(self.N_MIN, self.N_MAX):
            #print(i, int(self.y[i]+current[0]),int(self.x[i]+int(current[1])))
        

            #obstacle cell
            self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))] = 100
            
            self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))] = max(self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))+1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))+1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))+1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))+1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))+1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))+1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))] = max(self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))-1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])-1][int(self.x[i]*cells+(current[1]))-1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))-1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])][int(self.x[i]*cells+(current[1]))-1], 50)
            self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))-1] = max(self.obstacle_map[int(self.y[i]*cells+current[0])+1][int(self.x[i]*cells+(current[1]))-1], 50)
            
            for r in range(0, int(round(cells*RANGE[i]))):
            	#print(i, r)
            	tempx = -(r*np.sin(i*self.ANGLE_INC))
            	tempy = -(r*np.cos(i*self.ANGLE_INC))
            	self.obstacle_map[int(tempy+current[0])][int(tempx+current[1])] = max(self.obstacle_map[int(tempy+current[0])][int(tempx+current[1])], 0)

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
    def plotObs(self, x,y):
    	self.scat1 = self.ax.scatter(x, y)
    	self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def loop():
        
        localMap = OccupancyGrid()
        #graph = Graph()
        map = Map()
        while not rospy.is_shutdown():
            
            
            resolution = 0.05
            #Goalx,Goaly, Posx, Posy = gps.heading()
            
            #(x,y) = map.PlotData()
            #graph.plotObs(x, y)

            l_map, height, width, current = map.ObstacleMap(resolution)

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
