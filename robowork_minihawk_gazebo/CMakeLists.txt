cmake_minimum_required(VERSION 2.8.3)
project(robowork_minihawk_gazebo)

find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs
roslaunch
)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
