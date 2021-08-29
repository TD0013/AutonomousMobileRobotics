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
   width="200" alt="4-DOF Robot" style="horizontal-align:middle">
  

## SLAM
* TeleOp.py
* SLAM1.py

## Path Planning
* navigation.py

## Obstacle Avoidance 
* APF_Turtlebot.py
* ObstacleAvoidance.py

## Computer Vision
* canny.py
* LineFollower.py
* AlternateLineFollower.py

