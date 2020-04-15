#!/usr/bin/env python 

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class InterfaceKeyboard: 
    def __init__(self): 
        rospy.init_node("Interface_keyboard_node", anonymous=False)
        self.sub = rospy.Subscriber("velocity_inputs",Twist, self.velocity_cb)
        self.left = rospy.Publisher("/simpleRobot/left_wheel_controller/command", Float64, queue_size=5)
        self.right = rospy.Publisher("/simpleRobot/right_wheel_controller/command", Float64, queue_size=5)

    def velocity_cb(self,data): 
        right_velocity = Float64()
        left_velocity = Float64()
        right_velocity = data.linear.x + 0.15*data.angular.z
        left_velocity = data.linear.x - 0.15*data.angular.z

        rospy.loginfo("Left speed %f\n Right speed %f\n", left_velocity, right_velocity)
        self.left.publish(left_velocity)
        self.right.publish(right_velocity)

if __name__ == "__main__":
    I = InterfaceKeyboard()
    rospy.spin()
