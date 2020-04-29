#!/usr/bin/env python 

# Import ros libraries and msgs
import rospy
from std_msgs.msg import Float64

# Import python libraries
import math


class TableController: 
    def __init__(self): 
        rospy.init_node("table_controller_node",anonymous=False)

        # Publisher and subscribers to get the table moving as one unit
        self.postion_sub = rospy.Subscriber("/robot1/push_link_controller/command", Float64, self.get_pos_cb)
        self.right_back_leg_pub = rospy.Publisher("/robot1/right_back_leg_controller/command", Float64, queue_size=1)
        self.left_back_leg_pub = rospy.Publisher("/robot1/left_back_leg_controller/command", Float64, queue_size=1)
        self.right_front_leg_pub = rospy.Publisher("/robot1/right_front_leg_controller/command", Float64, queue_size=1)
        self.left_front_leg_pub = rospy.Publisher("/robot1/left_front_leg_controller/command", Float64, queue_size=1)
        
        # The bottom height of the table and the length of a support leg
        self.bottom_height = 25.48
        self.support_leg_length = 81.0
        self.max_angle = 0.52
        self.min_angle = 0.32


    def get_pos_cb(self,data): 
        current_height = self.bottom_height + data.data
        angle = math.asin(current_height/self.support_leg_length) 

        if angle > self.max_angle: 
            angle = self.max_angle
        elif angle < self.min_angle:
            angle = self.min_angle

        front_angle = Float64()
        back_angle = Float64()

        front_angle = angle
        back_angle = -angle

        rospy.loginfo("Angle: %f", angle)

        self.right_back_leg_pub.publish(back_angle)
        self.left_back_leg_pub.publish(back_angle)

        self.right_front_leg_pub.publish(front_angle)
        self.left_front_leg_pub.publish(front_angle)
        


if __name__ == "__main__":
    TableController = TableController()
    rospy.spin()