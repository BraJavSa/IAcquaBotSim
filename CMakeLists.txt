cmake_minimum_required(VERSION 3.5)
project(IAcquaBotSim)

# Buscar dependencias principales
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  gazebo_ros
  usv_description
  usv_gazebo
)

catkin_package()

# Instalar archivos de launch y scripts
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