# IAcquaBotSim
The IAcquaBotSim is a digital twin of the real USV developed by the National University of San Juan as a modular, low-cost platform for research in autonomous surface navigation. Built in the Gazebo simulation environment and based on work from the VRX (Virtual RobotX) project.

# Installation

## Install ROS Noetic:
  
  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt install curl 
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
  sudo apt update
  sudo apt install ros-noetic-desktop-full
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  sudo rosdep init
  rosdep update
``` 
then close all terminals and open one again 


## Install MAVROS
  
  ```
  sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
  sudo bash ./install_geographiclib_datasets.sh 
  
  ```

## Install SIMULATOR

    follow this link for install joy-ros http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

    ```
    sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
    sudo apt-get install ros-noetic-hector-gazebo-plugins
    sudo apt-get install -y libgazebo11-dev
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install python3-catkin-tools
    cd
    source .bashrc
    catkin_init_workspace
    cd ~/catkin_ws
    catkin_make
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    cd ~/catkin_ws/src
    
    git clone https://github.com/BraJavSa/hector-quadrotor-noetic.git
    git clone https://github.com/BraJavSa/px4_offboard_control.git
    git clone https://github.com/BraJavSa/usv_sim.git
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
    git clone -b gazebo_classic https://github.com/BraJavSa/vrx.git
    cd ~/catkin_ws
    echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/catkin_ws/src/IAcquaBotSim/submodules/usv_description" >> ~/.bashrc
    echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/catkin_ws/src/IAcquaBotSim/submodules/usv_gazebo" >> ~/.bashrc
    source ~/.bashrc
    catkin_make
    ```
## run SIMULATOR


    ```
    roslaunch IAcquabot sandisland.launch
    ```

