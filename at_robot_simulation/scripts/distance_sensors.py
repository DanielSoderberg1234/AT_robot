#!/usr/bin/env python 

# Ros libraries
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class DistanceSensors: 
    def __init__(self, regulator): 
        self.regulator = regulator
        self.front_sensors = rospy.Subscriber("/robot1/sensor/front", Range, self.front_sensors_cb)
        self.back_sensors = rospy.Subscriber("/robot1/sensor/back", Range, self.back_sensors_cb)
        self.time_semicritical = rospy.Time.now().to_sec()
        self.time_critical = rospy.Time.now().to_sec()


    def front_sensors_cb(self, range): 
        if self.regulator.cmd_vel.linear.x > 0: 

            if range.range < 0.3: 
                self.time_critical = rospy.Time.now().to_sec()
                self.regulator.brake_robot()
            
            elif range.range < 1.0:
                self.time_semicritical = rospy.Time.now().to_sec()
                self.regulator.slow_down_fact = 0.5

            if self.regulator.slow_down_fact < 1.0 and (rospy.Time.now().to_sec() -  self.time_semicritical) > 2.0:
                self.regulator.slow_down_fact = 1.0 
        
        elif self.regulator.cmd_vel.linear.x == 0 and self.regulator.cmd_vel.angular.z != 0: 
            if range.range < 0.3: 
                self.time_critical = rospy.Time.now().to_sec()
                self.regulator.brake_robot()



    def back_sensors_cb(self,range):
        if self.regulator.cmd_vel.linear.x < 0: 

            if range.range < 0.3: 
                self.time_critical = rospy.Time.now().to_sec()
                self.regulator.brake_robot()
            
            elif range.range < 1.0:
                self.time_semicritical = rospy.Time.now().to_sec()
                self.regulator.slow_down_fact = 0.5

            if self.regulator.slow_down_fact < 1.0 and (rospy.Time.now().to_sec() -  self.time_semicritical) > 2.0:
                self.regulator.slow_down_fact = 1.0 

        elif self.regulator.cmd_vel.linear.x == 0 and self.regulator.cmd_vel.angular.z != 0: 
            if range.range < 0.3: 
                self.time_critical = rospy.Time.now().to_sec()
                self.regulator.brake_robot()