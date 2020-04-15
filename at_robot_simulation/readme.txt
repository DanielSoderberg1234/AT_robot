This directory contains simulations that were carried out in this project. 

In order to make sure that the keyboard2.py was generic, it was tried on a simple robot at first. This robot had two controllers that needed the velocity for each wheel, therefore there is an intermediate node that converts the linear and angular velocities to corresponding wheel velocities.

The keyboard2.py will be updated to take parameters inorder for it to be generic for any differntial drive robot, but for the moment, please change the values manually. 

Inorder to try drivning the simple robot:

1) roslaunch at_robot_simulation simpleRobot_gazebo.launch
2) rosrun at_robot_drive keyboard2.py
3) rosrun at_robot_simulation drive_simple_robot.py
