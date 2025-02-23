cmake_minimum_required(VERSION 3.5)
project(IAcquaBotSim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  gazebo_ros
  xacro
)

add_subdirectory(submodules/usv_description)
add_subdirectory(submodules/usv_gazebo)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs gazebo_ros xacro
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS
  scripts/mi_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

