#!/usr/bin/env python 

# Ros libraries
import rospy
from std_msgs.msg import String

import matplotlib.pyplot as plt 
import matplotlib.image as mp

def plotstuff(): 
    img = mp.imread('/home/daniel/enviroment.png')
    imgplot = plt.imshow(img)
    plt.show()

if __name__ == "__main__":
    plotstuff()
    rospy.init_node("test", anonymous=False)
    rospy.spin()