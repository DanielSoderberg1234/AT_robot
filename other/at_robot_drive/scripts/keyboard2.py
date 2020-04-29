#!/usr/bin/env python 


# This file reades keyboard commands
# to drive a differential drive robot,
# the BOE shield bot
# ---------------------------------------------
#       w           [       foward           ]
#   a   s   d       [left   backward    right]
# ---------------------------------------------


import tty, sys, termios
import rospy
import math
from geometry_msgs.msg import Twist


class Keyboard_reader:
    def __init__(self):
        # Init the node and publisher
        rospy.init_node("Keyboard_reader",anonymous=False)
        self.cmd_pub = rospy.Publisher("velocity_inputs",Twist, queue_size=2)
        # Init the message, to variables to keep track of linear and angular velocity
        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        # Define how we should interpret key presses, and max values
        self.key_interpretation = {'w':(15.32/3,0),'s':(-15.32/3,0),'a':(0,2.83/3),'d':(0,-2.83/3),}
        self.max = 5
        # Call the function to read continously 
        self.read_cont()

    def read_cont(self): 
        while True: 
            pressed_key = self.get_pressed_key()
            if pressed_key in self.key_interpretation.keys(): 
                self.check_pub_cmd(pressed_key)
            elif pressed_key == '\x03': 
                break

            

    def check_pub_cmd(self,key): 
        # Check if velocity request is greater than max or min
        if key == 'w': 
            if math.fabs(self.velocity.angular.z) == self.max: 
                self.velocity.angular.z = 0 

            elif self.velocity.linear.x != self.max:
                self.velocity.linear.x += 1
                self.velocity.angular.z = 0

        elif key == "s":
            if math.fabs(self.velocity.angular.z) == self.max: 
                self.velocity.angular.z = 0 

            elif self.velocity.linear.x != -self.max:
                self.velocity.linear.x -= 1
                self.velocity.angular.z = 0
            

        elif key == "a": 
            if (math.fabs(self.velocity.linear.x) + math.fabs(self.velocity.angular.z)) == self.max and self.velocity.angular.z != self.max and self.velocity.linear.x > 0: 
                self.velocity.angular.z += 1
                self.velocity.linear.x -= 1
            elif (math.fabs(self.velocity.linear.x) + math.fabs(self.velocity.angular.z)) == self.max and self.velocity.angular.z != self.max and self.velocity.linear.x < 0: 
                self.velocity.angular.z += 1
                self.velocity.linear.x += 1
            elif self.velocity.angular.z != self.max:
                self.velocity.angular.z += 1

        elif key == 'd': 
            if (math.fabs(self.velocity.linear.x) + math.fabs(self.velocity.angular.z)) == self.max and self.velocity.angular.z != -self.max and self.velocity.linear.x > 0: 
                self.velocity.angular.z -= 1
                self.velocity.linear.x -= 1
            elif (math.fabs(self.velocity.linear.x) + math.fabs(self.velocity.angular.z)) == self.max and self.velocity.angular.z != -self.max and self.velocity.linear.x < 0: 
                self.velocity.angular.z -= 1
                self.velocity.linear.x += 1
            elif self.velocity.angular.z != -self.max:
                self.velocity.angular.z -= 1


        cmd_send = Twist()
        cmd_send.linear.x = self.velocity.linear.x *(15.32/self.max)
        cmd_send.angular.z = -self.velocity.angular.z *(2.83/self.max)
        rospy.loginfo("Veolocities:\nV = %f\nW=%f",cmd_send.linear.x,cmd_send.angular.z)
        
        self.cmd_pub.publish(cmd_send)


    def get_pressed_key(self): 
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try: 
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return ch
        
if __name__ == "__main__":
    K = Keyboard_reader()