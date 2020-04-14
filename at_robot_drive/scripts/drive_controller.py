#!/usr/bin/env python 

import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Range

class DriveController: 
    def __init__(self): 
        # Initiate node
        rospy.init_node("drive_controller_node", anonymous=False)
        #Initate velocity subscribers and publishers
        self.sub_vel = rospy.Subscriber("velocity_inputs", Twist, self.vel_cb)
        self.pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=2)
        # Initiate subscribers for ultrasounds
        self.sub_ultra_front = rospy.Subscriber("ultrasound_front", Range, self.front_cb)
        self.sub_ultra_back = rospy.Subscriber("ultrasound_back", Range, self.back_cb)
        self.flag = True
        self.time_obstacle_detection = rospy.Time.now().to_sec()
        # Message to set specifik velocity
        self.break_msg = Twist()

    def vel_cb(self,data): 
        if self.flag: 
            self.pub_vel.publish(data)
            rospy.loginfo("Publishing velocity to robot")

    def front_cb(self,data): 

        if data.range < 15:
            self.pub_vel.publish(self.break_msg)
            self.time_obstacle_detection = rospy.Time.now().to_sec()
            self.flag = False
        elif rospy.Time.now().to_sec() - self.time_obstacle_detection > 4: 
            self.flag = True
        
        rospy.loginfo("Stop "+str(self.flag)+"\nFront messages")

    def back_cb(self,data): 

        if data.range < 30:
            self.pub_vel.publish(self.break_msg)
            self.time_obstacle_detection = rospy.Time.now().to_sec()
            self.flag = False
        elif rospy.Time.now().to_sec() - self.time_obstacle_detection > 4:
            self.flag = True

        rospy.loginfo("Stop "+str(self.flag)+"\nBack messages")
    
if __name__ == "__main__":
    D = DriveController()
    rospy.spin()
