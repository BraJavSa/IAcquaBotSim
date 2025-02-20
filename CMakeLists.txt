cmake_minimum_required(VERSION 3.5)
project(IAcquaBotSim)

# Buscar dependencias principales
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  gazebo_ros
)

# Añadir subdirectorios para los subpaquetes
add_subdirectory(submodules/usv_gazebo)
add_subdirectory(submodules/usv_description)

# Definir el paquete y sus archivos de instalación
catkin_package()

# Instalar archivos de launch y scripts
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS
  scripts/mi_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
