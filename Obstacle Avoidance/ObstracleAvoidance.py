#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


rospy.init_node('avoid')
print('init node')
Range = []

print('Def')


print('Def2')
def scan_callback(msg):
    global Range
    Range = [0,0,0,0,0,0,0,0,0,0,0]
    n = -5
    while n < 6:
        #print([(len(msg.ranges)/2)+(n*3)])
        Range[n] = (msg.ranges[(len(msg.ranges)/2)+(n*6)])
        n +=1
    #print(Range)
    n = 0


print('Def3')

def TurnRight():

    turn = Twist()
    turn.angular.z = -0.1
    cmd_vel_pub.publish(turn)
    rospy.sleep(0.5)
    move()

def TurnLeft():

    turn = Twist()
    turn.angular.z = 0.1
    cmd_vel_pub.publish(turn)
    rospy.sleep(0.5)
    move()

def Reverse():

    back = Twist()
    back.linear.x = 0.1
    cmd_vel_pub.publish(back)
    rospy.sleep(0.5)
    move()

def Forward():

    fwd = Twist()
    fwd.linear.x= -0.1
    cmd_vel_pub.publish(fwd)
    rospy.sleep(0.5)
    move()

Threshold = 0.3

def Rangetest():

    global GoRight
    global GoLeft
    global GoRev

    GoRight = False
    GoLeft = False
    GoRev =False


    i = 0
    RightMin = Range[i]
    while i < 6:
        if(Range[i]<RightMin):
            RightMin = Range[i]
        i+=1


    i = -5
    LeftMin = Range[i]
    while i < 0:
        if(Range[i]<RightMin):
            LeftMin = Range[i]
        i+=1

    print(RightMin,LeftMin)

    if (RightMin < LeftMin and RightMin < Threshold):
        GoRight = True

    if (LeftMin < RightMin and LeftMin < Threshold):
        GoLeft = True




    if (Range[0] < 0.2):
        GoRev = True

def move():
    print("move init")
    stp = Twist()
    cmd_vel_pub.publish(stp)
    Rangetest()

    print(GoRev,GoRight,GoLeft)

    if(GoRev == True):
        print("loop 1-1")
        Reverse()

    elif(GoRight == True):
        print("loop 1-2")
        TurnRight()

    elif(GoLeft == True):
        print("loop 1-3")
        TurnLeft()

    else:
        print('2')
        Forward()


    print('3')

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

print(Range)

while Range == []:
    print("waiting...")
    rospy.sleep(2)


move()

rospy.spin()
