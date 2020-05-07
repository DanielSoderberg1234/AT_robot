#!/usr/bin/env python 

import rospy
from purepursuit import PurePursuit
from table_controller import TableController
from distance_sensors import DistanceSensors

def main(): 
    rospy.init_node("controller", anonymous=False)
    regulator = PurePursuit()
    TableController(regulator)
    DistanceSensors(regulator)
    rospy.spin()


if __name__ == "__main__":
    main()