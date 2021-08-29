#!/usr/bin/env python2
import sys, select, tty, termios
import rospy
from rospy.client import init_node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Teleop:

    def __init__(self):
        self.key_pub = rospy.Publisher('keys', String, queue_size=1)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        
        self.keyRead()
        self.key_to_teleop()
        #self.keys_cb()
        self.Loop()

        
    def keyRead(self):
        print('1')
        #self.key_pub = rospy.Publisher('keys', String, queue_size=1)

        self.old_attr = termios.tcgetattr(sys.stdin)


        tty.setcbreak(sys.stdin.fileno())
        print("Publishing keystrokes.")
        self.t = Twist()


    def key_to_teleop(self):

        self.key_mapping = { 'w': [ 0, 0.1], 's': [0, -0.1],
                        'd': [-0.1, 0], 'a': [0.1, 0],
                        ' ': [ -self.t.angular.z, -self.t.linear.x] }


    def Loop(self):
        print("enter Loop")
        while not rospy.is_shutdown():
            self.EnteredKey = String()

            print(self.EnteredKey.data)
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                
                
                self.EnteredKey.data= sys.stdin.read(1)
                self.key_pub.publish(self.EnteredKey)
                
                print(self.EnteredKey.data)
               

                

            if len(self.EnteredKey.data) == 0 or not self.key_mapping.has_key(self.EnteredKey.data[0]):
                print('unknown key')
                 # unknown key


            else:
                self.key_to_teleop()
                self.vels = self.key_mapping[self.EnteredKey.data[0]]
                print("Printing Velocity")
                print(self.vels)
                
                self.t.angular.z += self.vels[0]
                self.t.linear.x += self.vels[1]
                print(self.t)
                self.twist_pub.publish(self.t)

            rospy.sleep(0.1)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)


if __name__ == '__main__':
    rospy.init_node('TeleOP')
    teleop = Teleop()
    rospy.spin()
    sys.exit(0)