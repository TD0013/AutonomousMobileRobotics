# AutonomousMobileRobotics
This repository contains the work I have done in the field of Autonomous Mobile Robotics.

## Table Of Contents
 * [Custom Models](#custom-models)
 * [SLAM](#slam)
 * [Path Planning](path-planning)
 * [Obstacle Avoidance](#obstacle-avoidance)
 * [Computer Vision](#computer-vision)

## Custom Models
Custom Models contains the models I made for testing out various algorithms using different sensor data.
##### Custom1.urdf
Custom1.urdf is a basic robot with four wheels and a lidar sensor plugin that I initially created to get familiar with urdf and gazebo models.
 
   <img src ="https://user-images.githubusercontent.com/86218311/131243211-8ee8b566-aa4f-4d89-99d7-6a2558d989bc.png"
   width="320"  style="horizontal-align:middle">

##### Custom2.urdf
Custom2.urdf is a modified [Pioneer3at](https://github.com/MobileRobots/amr-ros-config), equipped with wheel actuators, a GPS sensor plugin, an imu sensor plugin, a 2-D Lidar plugin and a Kinect sensor plugin.

  <img src ="https://user-images.githubusercontent.com/86218311/131243394-9ef630c6-9d6a-475b-bb88-c2cd41af42b6.png"
   width="320"  style="horizontal-align:middle">

##### Custom3.launch
Custom3.launch uses the Custom2.urdf and goalpole.urdf to launch the rover model and a goal pole, both having their GPS sensors. This is so that we can try and implement path planning algorithms as we have both the rover's coordinates(/GPS/fix) and the goal coordinates(/gpsGoal/fix).
  
  <img src ="https://user-images.githubusercontent.com/86218311/131243519-266f2711-2ce6-4ef0-a9b3-4fa4bd1f469c.png"
   width="320"  style="horizontal-align:middle">
  <img src ="https://user-images.githubusercontent.com/86218311/131243582-1937702a-0d40-4807-8715-6921181fdad1.png"
   width="250"  style="horizontal-align:middle">
  

## SLAM
SLAM contains the codes I made for mapping or SLAM

##### globalMap.py
globalMap.py is a code for that uses the data from localMap.py to build and store a map of its surroundings.
* It takes in localMap and odom data to plot the data from the localMap in a global frame.
* It constructs a blank map of dimensions 1000x1000 with resolution same as the localMap.
* Then according to the localMap, it populates the map, converts the map into ROS recognisable nav_msgs/OccupancyGrid and publishes the map and stores htis map as old_map.
* When the next localMap scan comes in, the loop takes the data from the old_map and localMap and takes the weighted average to get the value(0-100) of a cell.
* Then the map is again canverted to OccupancyGrid, published and stored as old_map.
* This data can be vizualized using Rviz.

Drawbacks:
* Due to the inaccuracy in odom data, the map can have some inaccuracy.

 Example: On the left is a gazbo simulation of the custom2 model in a gazebo worls and on the right is the vizualization of the Lidar, pointCloud data and the Occupancy Grid map.
 
 ![globalMap](https://user-images.githubusercontent.com/86218311/132984227-25d49ade-228b-4569-8321-0af28076d747.gif)


##### localMap.py
localMap.py is a code for building a realtime, constantly updating occupancy grid map of the robot in its surroundings and vizualizing it on rviz.
* It takes in 2-D Lidar data and depending on the distance from the farthest obstacle and the given resoultion, defines a matrix of relevant leangth and given resolution.
* Using the Lidar data, it marks obstacles on the matrix having the value '100' and all 8 adjacent cells having value '50'.
* Then it takes the assigns a value of '0' to all the cells between the rover and the detected obstacle.
* When all obstacles have been marked, it converts the matrix in the form of ROS recognisable nav_msgs/OccupancyGrd format and publishes the map data.
* This data can be vizualized using Rviz.

Drawbacks:
* The algorithm that assigns a value of '0' to free cells publishes some garbage values and has some gaps if the obstacles are far away.

 Example: On the left is a gazbo simulation of the custom2 model in a gazebo worls and on the right is the vizualization of the Occupancy Grid map.
 
 ![localMap](https://user-images.githubusercontent.com/86218311/132983900-f8e347ef-754c-4387-b25f-d64422b8a8b0.gif)

 
##### SLAM1.py
SLAM1.py is a code that uses live 2-D Lidar and initial, current and goal GPS data to output an occupancy grid map.
 * Depending on the GPS coordinates of the rover and the goal, it constructs a matrix that is four times the distance between them. This method makes the matrix size dynamic and reduces unnecessary computational complexity.
 * The code then uses the rover's initial(start) GPS coordinates to set it on the centre of the matrix. 
 * Then, according to the current and goal GPS coordinates, calculates their position wrt to the start coordinates and places them in corresponding matrix cells having the same distance from the centre cell.
 * It also takes live LIDAR data, changes its frame from the current GPS frame to the start GPS frame, and fills the corresponding cell to plot obstacles.
 * The code then plots this matrix onto a matplotlib matshow map. Since it uses live data, it updates the map every second to provide a real-time map of its surroundings.  
 
 Example: On the Left is a gazebo simulation of an environment and on the right is the occupancy grid map constructed using SLAM1.py. The green dot(near 80, 60) is the start position, the yellow-green dot (near 60, 45) is the current position, and the yellow dot(near 25, 25) is the Goal. (x,y: reference x in the top scale, y in the left scale) 
 
 
   <img src ="https://user-images.githubusercontent.com/86218311/131244690-8e1d3e86-62f1-4296-b86b-7a82f648fc23.png"
   width="350"  >
   <img src ="https://user-images.githubusercontent.com/86218311/131244703-6ef760c5-3cf6-4e61-a7b2-06609ec68dde.png"
   width="400"  >


## Path Planning
##### navigation.py
* Navigation.py is a code that takes the current and goal GPS coordinates and tf2 and outputs a heading and a velocity based on the difference between the two coordinates.
* It takes the frame transformation data (custom2/tf2) into account so that the heading calculated by the GPS coordinates, and is hence in the global frame, is converted to appropriate custom2 frame and output as twist velocities. 

 
   ![navigation py gif](https://user-images.githubusercontent.com/86218311/131245455-02829c1a-c334-4739-b288-1bfc501b7f0d.gif)


## Obstacle Avoidance 
##### APF_Turtlebot.py
* APF_Turtlebot is a code that implements a basic Artificial Potential Field Algorithm for avoiding obstacles that I applied on a [turtlebot3 waffle pi](https://github.com/ROBOTIS-GIT/turtlebot3).
* The Turtlebot has a 2-D lidar mounted on it and uses this Lidar to scan its surroundings and returns an array with 360 distance values. 
* The code splits the lidar scan distance as the x and y coordinates for each point in the region 60 to -60 degrees and stores these values.
* Based on this x and y distance from each point, a force inversely proportional to the x/y distance is calculated. 
* These forces are summed up to give a net force in the x and y direction and published as twist velocities

Example: The following video shows the code APF_turtlebot.py controlling a turtlebot3 waffle pi in a gazebo house environment.
 The model can be seen changing its direction to avoid walls and obstacles and even reversing when it sees no way forward.
  
  ![APF_Turtlebot](https://user-images.githubusercontent.com/86218311/131246478-46ef914e-8886-4fd1-995b-e45561897446.gif)

##### ObstacleAvoidance.py
* An initial implementation of a bug-type obstacle avoidance algorithm.
* It uses lidar data to scan its front for obstacles.
* If it is about to collide with an obstacle, it stops and turns left or right(whichever is a free space) and then resumes its motion.
   
   ![ObstacleAvoidance](https://user-images.githubusercontent.com/86218311/131246763-08dc0e6a-b22a-4d0f-b7b2-ce724d19590e.gif)


## Computer Vision
##### LineFollower.py
* LineFollower.py contains the code for a [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) burger that enables it to follow a line.
* The code takes input form a comera mounted on the turtlebot and uses OpenCV to find and follow a line.
* The video from the camera is first masked so that it only looks for a line in the region of interest.
* Once it has a region of interest, it thenn processes using thresholds for finding a yellow line.
* Then it marks the found yellow line in the video and using momnets, finds the center of the line.
* Finally, it publishes twist velocities based on the centre of the moments to follow the line
  Example:
  
  ![LineFollower](https://user-images.githubusercontent.com/86218311/131248464-7c60d5ba-b2f4-4edd-9da6-4907676e4d7b.gif)

##### alternateLineFollower.py
* Alternate Line Follower works the same as the LineFollower.py but instead of using thresholds for detecting a yellow line, uses canny edge detection for detecting general lines.

Example:

  <img src ="https://user-images.githubusercontent.com/86218311/131248622-0113570f-a3d6-40a2-b9f1-b53f9e47b813.png"
   width="300" >




