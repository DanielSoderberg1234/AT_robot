cmake_minimum_required(VERSION 2.8.3)
project(at_robot_simulation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES at_robot_simulation
   CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()





install(DIRECTORY images meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



