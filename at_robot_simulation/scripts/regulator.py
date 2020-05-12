#!/usr/bin/env python 

import rospy 
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String


import numpy as np
from math import sqrt,cos, sin, fabs, pi
import matplotlib.pyplot as plt 

class Regulator: 
    def __init__(self): 
        rospy.init_node("regulator", anonymous=False)

        # Odom callbacks
        self.odom_sub = rospy.Subscriber("/robot1/odom", Odometry,self.odom_cb)
        self.time_odom = rospy.Time.now().to_sec()

        # Regulator specific 
        self.cmd_pub = rospy.Publisher("/robot1/mobile_base_controller/cmd_vel", Twist, queue_size=1)
        self.dampning_fac = 0.6
        self.gain = 0.8

        # Trajectory related
        self.traj_sub = rospy.Subscriber("/robot1/trajectory", String, self.get_traj_cb)
        self.receieved_trajectory = False
        self.trajectory = []
        self.index = 0

        # Save stuff
        self.save_sub = rospy.Subscriber("/save", String, self.save_values)
        self.x_traj = []
        self.y_traj = []
        self.x_rob = []
        self.y_rob = []
        self.goal = []

        # Brake variables, slow down is set by ultrasound
        self.slow_down_fact = 1.0
        self.brake = False
        self.emergency_brake = False

    
    def get_traj_cb(self, traj): 

        with open('/home/daniel/traj.txt','r') as f:
            for line in f:
                array = []
                array = line.split()
                self.trajectory.append(array)
                self.x_traj.append(float(array[0]))
                self.y_traj.append(float(array[1]))

        self.goal.append(float(self.trajectory[len(self.trajectory)-1][0]))
        self.goal.append(float(self.trajectory[len(self.trajectory)-1][1]))

        self.receieved_trajectory = True
        print(self.trajectory)
                

    def odom_cb(self,odom): 
        if (rospy.Time.now().to_sec() - self.time_odom) >= 1.0: 
            self.time_odom = rospy.Time.now().to_sec()
            x_state = np.array([0.0,0.0,0.0])
            x_state[0] = odom.pose.pose.position.x
            x_state[1] = odom.pose.pose.position.y
            x_state[2] = euler_from_quaternion((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w ), axes='sxyz')[2]

            self.compute_input_cmd(x_state)

            

    def compute_input_cmd(self,x):
        x_state =np.array([x[0],x[1],x[2]])
        x_ref = np.array([0.0,0.0,0.0])
        u_ref = np.array([0.0,0.0])

        if not self.brake and self.receieved_trajectory: 

            # Find reference value
            d = sqrt( (x_state[0]-float(self.trajectory[self.index][0]))**2 + (x_state[1]-float(self.trajectory[self.index][1]))**2) 
            start = self.index
            goal = len(self.trajectory)
            while start<goal: 
                next_dist = sqrt( (x_state[0]-float(self.trajectory[start][0]))**2 + (x_state[1]-float(self.trajectory[start][1]))**2) 
                if next_dist < d:
                    d = next_dist
                    self.index = start
                start +=1

            self.x_rob.append(x_state[0])
            self.y_rob.append(x_state[1])
            

            x_ref[0] = float(self.trajectory[self.index][0])
            x_ref[1] = float(self.trajectory[self.index][1])
            x_ref[2] = float(self.trajectory[self.index][2])
            u_ref[0] = float(self.trajectory[self.index][3])
            u_ref[1] = float(self.trajectory[self.index][4])

            # Get the error
            delta_err = -(x_ref - x_state)
            delta_err[2] = (delta_err[2] + pi) % (2 *pi) - pi
            rotation_maxtrix = np.array( ([cos(x_state[2]),sin(x_state[2]),0], [-sin(x_state[2]),cos(x_state[2]),0], [0.0,0.0,1.0]))
            error = rotation_maxtrix.dot(delta_err)

            # Feedforward
            u_f = np.array([u_ref[0]*cos(error[2]), u_ref[1]])

            # Feedback 
            omegan = sqrt(u_ref[1]**2 + self.gain * u_ref[0]**2)
            k1 = 2*omegan*self.dampning_fac
            k2 = self.gain * fabs(u_ref[0])
            k3 = k1 

            k_matrix = np.array(([-k1, 0.0, 0.0], [0, -np.sign(u_ref[0])*k2, -k3 ]))

            u_b = k_matrix.dot(error)

            # Total u

            u =  u_f + u_b

            v = u[0]
            w = u[1]

            if v > 0.4: 
                v = 0.4
            elif v< -0.4:
                v = -0.4

            if w > 1: 
                w = 1
            elif w< -1:
                w = -1

            print("######################################\n")
            print("x_ref: "+str(x_ref))
            print("x_state: "+str(x_state))
            print("delta_err "+str(delta_err))
            print("error "+str(error))
            print("k matrix " +str(k_matrix) )
            print("u_f: "+str(u_f))
            print("u_b: "+str(u_b))
            print("v: "+str(v) +"\n"+"W: "+str(w))
            print("######################################\n")

            velocity_cmd = Twist()
            velocity_cmd.linear.x = u[0]
            velocity_cmd.angular.z = u[1]

            self.cmd_pub.publish(velocity_cmd)

            if v == 0:
                self.plot_tracking()



    def save_values(self,data): 
        with open("/home/daniel/robot2.txt","w") as file1: 
            for i in range(len(self.x_rob)):
                file1.write(str(self.x_rob[i]) + " " + str(self.y_rob[i]) +"\n"   )

        print("Done")

    def plot_tracking(self): 
        plt.plot(self.x_traj,self.y_traj, label="Path")
        plt.plot(self.x_rob,self.y_rob, label="Robot")
        plt.legend()
        plt.xlabel('X')
        plt.xlabel('Y')
        plt.title('Regulator med hastighetsreferenser')
        plt.show()


                



if __name__ == "__main__":
    regulator = Regulator()
    rospy.spin()