#!/usr/bin/env python 

# Ros libraries
import rospy 
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from atr_msgs.srv import changestate, changestateRequest, changestateResponse
from atr_msgs.srv import table, tableResponse, tableRequest

# Python libraries
import numpy as np
from math import sqrt,cos, sin, fabs, pi, atan, tan
import matplotlib.pyplot as plt 

class PurePursuit: 
    def __init__(self): 
        # Odom callbacks
        self.odom_sub = rospy.Subscriber("/robot1/odom", Odometry,self.odom_cb)
        self.time_odom = rospy.Time.now().to_sec()

        # Regulator specific 
        self.cmd_pub = rospy.Publisher("/robot1/mobile_base_controller/cmd_vel", Twist, queue_size=1)
        self.lookahead = 1
        self.v_ref = 0.4 
        self.goal = np.array([0.0,0.0])
        self.cmd_vel = Twist()
        self.states = {1:"Active", 2:"Waiting", 3:"Loading", 4:"CollisionDetection"}
        self.state = 2
        self.table_state_changer = rospy.Service("table_state_changer", changestate, self.change_regulator_state_table)
        self.coordinator = rospy.Service("coordintor", changestate, self.coordinator_service)
        self.forward = True
       
        
        # Trajectory related
        self.traj_sub = rospy.Subscriber("/robot1/trajectory", String, self.get_traj_cb)
        self.receieved_trajectory = False
        self.trajectory = []
        self.index = 0

        # Brake variables, slow down is set by ultrasound
        self.slow_down_fact = 1
        self.brake = False
        self.emergency_brake = False

        # Plot stuff
        self.x_traj = []
        self.y_traj = []
        self.x_rob = []
        self.y_rob = []
        self.plot = 0

    def brake_robot(self): 
        self.brake = True
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_pub.publish(self.cmd_vel)

    def change_regulator_state_table(self, request): 
         self.state = 3
         return changestateResponse(True)
        

    def coordinator_service(self, request): 
        print(self.states[self.state])
        if request.new_state == "Active": 
            try: 
                lower_table = rospy.ServiceProxy("/robot1/lower_table", table)
                req = tableRequest()
                req.distance = 0.0
                lower_table(req)
                self.state = 1
                self.brake = False
            except rospy.ServiceException: 
                rospy.loginfo("Failed to lower table")

            return changestateResponse(True)

        elif request.new_state == "Waiting": 
            self.state = 2
            return changestateResponse(True)
        elif request.new_state == "Loading": 
            self.state = 3
            return changestateResponse(True)
        elif request.new_state == "CollisionDetection": 
            self.state = 4
            return changestateResponse(True)
        
        return changestateResponse(False)

    def get_traj_cb(self, traj): 
        split_input = traj.data.split()
        filename = "{0}".format(split_input[0])
        self.trajectory = []
        self.index = 0
        with open('/home/daniel/{0}'.format(filename),'r') as f:
            for line in f:
                array = []
                array = line.split()
                self.trajectory.append(array)
                self.x_traj.append(float(array[0]))
                self.y_traj.append(float(array[1]))


        if split_input[1] == "forward": 
            self.forward = True
        else: 
            self.forward = False

        self.receieved_trajectory = True
        self.goal[0] = float(self.trajectory[len(self.trajectory)-1][0])
        self.goal[1] = float(self.trajectory[len(self.trajectory)-1][1])
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
        x_ref = np.array([0.0,0.0])
        self.x_rob.append(x_state[0])
        self.y_rob.append(x_state[1])


        if sqrt((self.goal[0]-x_state[0])**2 + (self.goal[1] - x_state[1])**2) < 0.35 and self.states[self.state] == "Active": 
            self.final()
            self.state = 2

        elif self.receieved_trajectory and not self.brake and self.states[self.state] == "Active": 

            for i in range(self.index, len(self.trajectory)): 
                dx = x_state[0]-float(self.trajectory[i][0])
                dy = x_state[1]-float(self.trajectory[i][1])
                l_cur = sqrt(dx**2 + dy**2)

                if fabs(l_cur-self.lookahead) < 0.25: 
                    self.index = i
                    # l_d = fabs(l_cur-self.lookahead)
                    break

            x_ref[0] = float(self.trajectory[self.index][0])
            x_ref[1] = float(self.trajectory[self.index][1])
            
            x_ref[0] = x_ref[0] - x_state[0]
            x_ref[1] = x_ref[1] - x_state[1]

            if not self.forward: 
                x_state[2] = x_state[2] - pi

            rotation_maxtrix = np.array( ([cos(x_state[2]),sin(x_state[2])], 
                                        [-sin(x_state[2]),cos(x_state[2])]))

            x_ref = rotation_maxtrix.dot(x_ref)
     
            e = x_ref[1]
            kappa = (2*e) / (self.lookahead**2)
            omega = self.v_ref / 0.8 * kappa*0.8

            print("###################################################") 
            print("Robot view of trajectory:  " + str(x_ref))
            print("e:" + str(e))
            print("x_state\n" + str(x_state))
            print("lookahead point\n" + self.trajectory[self.index][0] + " " + self.trajectory[self.index][1])    
            print("Kappa : "+ str(kappa))   
            print("Omega : "+ str(omega))
            
            print("Index")
            print("###################################################")

            if omega > 1: 
                omega = 1
            elif omega < -1: 
                omega = -1

            if not self.forward: 
                self.cmd_vel.linear.x = -self.v_ref*self.slow_down_fact
                self.cmd_vel.angular.z = omega*self.slow_down_fact
            else: 
                self.cmd_vel.linear.x = self.v_ref*self.slow_down_fact
                self.cmd_vel.angular.z = omega*self.slow_down_fact
            
            self.cmd_pub.publish(self.cmd_vel)

    def final(self): 
        self.brake = True
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_pub.publish(self.cmd_vel)
        self.plot += 1
        
        if self.plot == 3:
            self.plot_tracking()
       
    def plot_tracking(self): 
        plt.plot(self.x_traj,self.y_traj, label="Path")
        plt.plot(self.x_rob,self.y_rob, label="Robot")
        plt.legend()
        plt.xlabel('X')
        plt.xlabel('Y')
        plt.title('Pure pursuit')
        plt.show()
        
        
