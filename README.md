# AutonomousMobileRobotics
This repository contains the work I have done in the field of Autonomous Mobile Robotics

## Table Of Contents
 * [Custom Models](#custom-models)
 * [SLAM](#slam)
 * [Path Planning](path-planning)
 * [Obstacle Avoidance](#obstacle-avoidance)
 * [Computer Vision](#computer-vision)

## Custom Models
Custom Models contains the models I made for the 
* Custom1.urdf
Custom1.urdf is a basic robot with 4 wheels and a lidat sensor plugin that I initially created to get familiar with urdf and gazebo models.
 
   <img src ="https://user-images.githubusercontent.com/86218311/131243211-8ee8b566-aa4f-4d89-99d7-6a2558d989bc.png"
   width="320" alt="4-DOF Robot" style="horizontal-align:middle">

* Custom2.urdf
Custom2.urdf is a modified [Pioneer3at](https://github.com/MobileRobots/amr-ros-config), what is equvipped with wheel actuators, a gps sensor plugin, an imu sensor plugin, a 2-D Lidar plugin and a kinect sensor plugin.

  <img src ="https://user-images.githubusercontent.com/86218311/131243394-9ef630c6-9d6a-475b-bb88-c2cd41af42b6.png"
   width="320" alt="4-DOF Robot" style="horizontal-align:middle">

* Custom3.launch
Custom3.launch uses the Custom2.urdf and goalpole.urdf to launch the rover model and a goal pole, both having their own gps sensors. This is so that we can try and implement path planning algorithms as we have both the rover's coordinates(/gps/fix) and the goal coordinates(/gpsGoal/fix).
  
  <img src ="https://user-images.githubusercontent.com/86218311/131243519-266f2711-2ce6-4ef0-a9b3-4fa4bd1f469c.png"
   width="320" alt="4-DOF Robot" style="horizontal-align:middle">
  <img src ="https://user-images.githubusercontent.com/86218311/131243582-1937702a-0d40-4807-8715-6921181fdad1.png"
   width="250" alt="4-DOF Robot" style="horizontal-align:middle">
  

## SLAM
SLAM contains the codes I made for mapping or SLAM

* SLAM1.py
SLAM1.py is a code that uses live 2-D Lidar and initial, current and goal gps data to output an occupancy grid map.
* Depending on the gps coordinates of the rover and the goal, it constructs a matrix that is 4 times the distance between them. This medhod makes the matrix size dynamic and reduces unnecessary computational complexity.
* The code then uses the rover's initial(start) gps coordinates of the rover sets it on the center of the matrix. 
* Then according to the current and goal gps coordinates, calculates their position wrt to the the start coordinates and places them in corresponding matrix cells having the same distance from the center cell.
* It also takes live LIDAR data, changes its frame from current gps frame to start gps frame and plots fills the corresponding cell to plot obstacles.
* The code then plots this matrix onto a matplotlib matshow map. Since it uses live data, it updates the map every second to provide us with a realtime map of its surroundings.  
 Example: On the Left is a gazebo simulation of an environment and on the right is the occupancy grid map constructed using SLAM1.py. The green dot(near 80, 60) is the start positiion, the yello-green dot (near 60, 45) is the current position and the yellow dot(near 25, 25) is the Goal. (x,y: reference x in the top scale, y in the left scale) 
 
 
 <img src ="https://user-images.githubusercontent.com/86218311/131244690-8e1d3e86-62f1-4296-b86b-7a82f648fc23.png"
   width="300" alt="4-DOF Robot" style="horizontal-align:middle">
 <img src ="https://user-images.githubusercontent.com/86218311/131244703-6ef760c5-3cf6-4e61-a7b2-06609ec68dde.png"
   width="400" alt="4-DOF Robot" style="horizontal-align:middle">

* TeleOp.py
TeleOp.py is a code for Teleoperation of a rover. It captures keyboard inputs and accociates:
 w --> twist.linear.x += 0.1 (increases forward velocity by 0.1)
 s--> twist.linear.x -= 0.1 (decreases forward velocity by 0.1)
 a--> twist.angular.z += 0.1 (increases the leftward turn by 0.1)
 d--> twist.angular.z -= 0.1 (increases the rightward turn by 0.1)
 " " --> twist() (Stops the robot)
## Path Planning
* navigation.py

## Obstacle Avoidance 
* APF_Turtlebot.py
* ObstacleAvoidance.py

## Computer Vision
* canny.py
* LineFollower.py
* AlternateLineFollower.py

