#!/usr/bin/env python2

import matplotlib.pyplot as plt
import rospy
import math
from sensor_msgs.msg import NavSatFix

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler



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


class GPS:
    def __init__(self):
        print("GPS init")
        self.i=1
        self.j=1
        #rospy.Subscriber("/gps/fix", NavSatFix, self.PosCallback)
        rospy.Subscriber ('/ros0xrobot/odom', Odometry, self.get_rotation)


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
        
        
        
    def PosCallback(self, msg):
        self.currentLatitude = msg.latitude
        self.currentLongitude = msg.longitude
        if self.i ==1:
            self.startLatitude = msg.latitude
            self.startLongitude = msg.longitude
            self.i =0

    def heading(self):

        #self.Posx = self.currentLatitude - self.startLatitude
        #self.Posy = self.currentLongitude - self.startLongitude

        self.Posx = self.odometry_p.x - self.startx
        self.Posy = self.odometry_p.y - self.starty

class Map:

    def __init__(self, height, width):
        print("Map init")
        rospy.Subscriber('/localMap', OccupancyGrid, self.map_callback)
        self.old_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]
        self.count = [[1 for z in range(0, width+1)] for z in range(0, height+1)]
        rospy.sleep(2)


    def map_callback(self, msg):

        self.localMap = msg.data
        self.res = msg.info.resolution
        self.localHeight = msg.info.width
        self.localWidth = msg.info.height


    def obstacleMap(self, height, width, current, gps):

        cells = int(1/self.res)
        localHeight =  self.localHeight
        localWidth = self.localWidth
	local_map = self.local_map

	THETA = 0;
	TRANSFORMATION_J = -int(gps.odometry_p.x*cells)
	TRANSFORMATION_I = -int(gps.odometry_p.y*cells)

	self.obstacle_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]


        for j in range(0, localHeight):
            for i in range (0, localWidth):
                J = int(i*math.sin(THETA)+j*math.cos(THETA))+current[0]-localHeight/2 + TRANSFORMATION_J
                I = int(i*math.cos(THETA)-j*math.sin(THETA))+current[1]-localWidth/2 +TRANSFORMATION_I


                #print(j,i, J, I)

                

                if local_map[j][i] == -1:
                    self.obstacle_map[J][I] = self.old_map[J][I]
            
                else:
                    
                    self.obstacle_map[J][I] = int((local_map[j][i] + max(0,(self.count[J][I]-1)*self.old_map[J][I]))/(self.count[J][I]))
                
                    self.count[J][I] += 1

        
        self.old_map = self.obstacle_map

        self.matrixTOarray(height, width)

        return self.global_map

    def arrayTOmatrix(self):
        self.local_map = [[-1 for z in range(0, self.localWidth+1)] for z in range(0, self.localHeight+1)]
        i = self.localHeight*self.localWidth
        
        for a in range(0, self.localWidth):
            for b in range(0, self.localHeight):
                i-=1

                self.local_map[b][a] = self.localMap[i]

        #graph.updateObstacleMap(self.local_map)



    def matrixTOarray(self, height, width):
        self.global_map = [-1]*(width*height)
        #print(len(self.local_map))
        i = width*height
        
        for a in range(0, width) :
            for b in range(0, height) :
                i-=1
                self.global_map[i] = int(self.obstacle_map[b][a])
           
        

        

        
def loop():
    #graph=Graph()
    globalMap = OccupancyGrid()
    gps = GPS()
    
    HEIGHT = 400
    WIDTH = 400

    map = Map(HEIGHT, WIDTH)

    while not rospy.is_shutdown():
        


        
        
        RESOLUTION = map.res
        #HEIGHT = int(max(HEIGHT/2 + map.localHeight+(gps.odometry_p.x)/map.res, HEIGHT))
        #print("Height:", HEIGHT, map.localHeight, int(gps.odometry_p.x/map.res))
        #WIDTH =  int(max(WIDTH/2 +map.localWidth+(gps.odometry_p.y)/map.res, WIDTH))
        #print("WIDTH:", WIDTH, map.localWidth, gps.odometry_p.y/map.res)
        CENTER = [HEIGHT/2, WIDTH/2]


        
            #Goalx,Goaly, Posx, Posy = gps.heading()
        map.arrayTOmatrix()

        global_map = map.obstacleMap(HEIGHT, WIDTH, CENTER, gps)
        

        globalMap.header.frame_id = "odom"
        globalMap.info.height = WIDTH
        globalMap.info.width = HEIGHT
        globalMap.info.resolution = RESOLUTION
        globalMap.data = global_map

        globalMap.info.origin.position.x = -CENTER[0]*RESOLUTION
        globalMap.info.origin.position.y = -CENTER[1]*RESOLUTION 
        
        
        #globalMap.info.origin.orientation = gps.orientation_q

        gps.heading()
       # print(gps.odometry_p)
        
        

        
            
            
        test = rospy.Publisher("/globalMap", OccupancyGrid, queue_size=1)
        test.publish(globalMap)
            
        #print("published Map")
                
        #rospy.sleep(0.001)

        #print(local_map)

if __name__ == '__main__':
    rospy.init_node("globalMap")
    loop()


