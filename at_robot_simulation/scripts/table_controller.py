#!/usr/bin/env python 

# Import ros libraries and msgs
import rospy
from std_msgs.msg import Float64
from atr_msgs.srv import table, tableRequest, tableResponse
from atr_msgs.srv import changestate, changestateRequest, changestateResponse

# Import python libraries
import math


class TableController: 
    def __init__(self, regulator): 
        

        # Publisher and subscribers to get the table moving as one unit

        self.raise_service = rospy.Service("/robot1/raise_table", table, self.raise_table )
        self.lower_service = rospy.Service("/robot1/lower_table", table, self.lower_table )

        self.postion_pub = rospy.Publisher("/robot1/push_link_controller/command", Float64, queue_size=1)
        self.right_back_leg_pub = rospy.Publisher("/robot1/right_back_leg_controller/command", Float64, queue_size=1)
        self.left_back_leg_pub = rospy.Publisher("/robot1/left_back_leg_controller/command", Float64, queue_size=1)
        self.right_front_leg_pub = rospy.Publisher("/robot1/right_front_leg_controller/command", Float64, queue_size=1)
        self.left_front_leg_pub = rospy.Publisher("/robot1/left_front_leg_controller/command", Float64, queue_size=1)
        
        # The bottom height of the table and the length of a support leg
        self.bottom_height = 25.48
        self.support_leg_length = 81.0
        self.max_angle = 0.52
        self.min_angle = 0.32

        # Connect to the regulator
        self.regulator = regulator


    def raise_table(self, height):   
        # If the robot is not active or if it hasn't detected an obstacle go into loading phase
        rospy.loginfo("State is:   "+self.regulator.states[self.regulator.state] )

        if self.regulator.state == 1 or self.regulator.state == 4: 
            rospy.loginfo("I skipped raising the table")

        else: 
            res = False
            try: 
                change_regulator_state = rospy.ServiceProxy("table_state_changer", changestate)
                res = change_regulator_state("Loading")
                
            except rospy.ServiceException: 
                rospy.loginfo("Service failed")

            if res: 
                h = height.distance 
                self.change_table_height(h)


        return tableResponse(0.0)
        

    def lower_table(self,request): 
        rospy.loginfo("Trying to lower table")
        self.change_table_height(0.0)
        return tableResponse(0.0)

    def change_table_height(self,height):
        current_height = self.bottom_height + height
        angle = math.asin(current_height/self.support_leg_length) 

        if angle > self.max_angle: 
            angle = self.max_angle
        elif angle < self.min_angle:
             angle = self.min_angle

        front_angle = Float64()
        back_angle = Float64()

        front_angle = angle
        back_angle = -angle

        self.postion_pub.publish(height)
        self.right_back_leg_pub.publish(back_angle)
        self.left_back_leg_pub.publish(back_angle)
        self.right_front_leg_pub.publish(front_angle)
        self.left_front_leg_pub.publish(front_angle)



