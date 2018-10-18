#!/bin/bash

# installation file for lizi over ROS Kinetic and ubuntu 16.04 #

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
NO_COLOR='\033[0m'


printf "${WHITE_TXT}\n***Installing Lizi ROS-Kinetic Package***\n${NO_COLOR}"

# validate ros version #
printf "${WHITE_TXT}\nChecking ROS Version...\n${NO_COLOR}"

version=`rosversion -d`
if [ "$version" == "kinetic" ]; then
  printf "${GREEN_TXT}ROS version OK${NO_COLOR}\n"
else
  printf "${RED_TXT}Error: found not compatible ROS version ${version}, please install ROS Kinetic and try again${NO_COLOR}\n"
  exit 1
fi

CATKIN_WS_SRC=$( cd "$(dirname "$0")" && cd ../.. && pwd )
printf "${WHITE_TXT}\nAssuming catkin workspace src folder path is: '$CATKIN_WS_SRC'\n${NO_COLOR}"

printf "${WHITE_TXT}\nInstalling 3rd party packages...\n${NO_COLOR}"
# third party packages #
sudo apt-get update
sudo apt-get dist-upgrade 
sudo apt-get upgrade 

# from this point on, exit and notify immediately if a command exits with a non-zero status
set -eb

sudo apt-get -y install ros-kinetic-controller-manager 
sudo apt-get -y install ros-kinetic-control-toolbox  
sudo apt-get -y install ros-kinetic-transmission-interface 
sudo apt-get -y install ros-kinetic-joint-limits-interface 
sudo apt-get -y install ros-kinetic-ros-controllers 
sudo apt-get -y install ros-kinetic-ros-control 
sudo apt-get -y install ros-kinetic-move-base
sudo apt-get -y install ros-kinetic-navigation
sudo apt-get -y install ros-kinetic-hector-slam
sudo apt-get -y install ros-kinetic-gmapping
sudo apt-get -y install ros-kinetic-twist-mux
sudo apt-get -y install ros-kinetic-pid
sudo apt-get -y install ros-kinetic-ar-track-alvar
sudo apt-get -y install ros-kinetic-serial
sudo apt-get -y install ros-kinetic-robot-localization
sudo apt-get -y install ros-kinetic-trac-ik ros-kinetic-moveit-kinematics 
sudo apt-get -y install ros-kinetic-urg-node
sudo apt-get -y install espeak espeak-data libespeak-dev 

wget https://github.com/robotican/diff_drive_slip_controller/archive/V1.0.0.tar.gz
tar -xvzf V1.0.0.tar.gz
rm V1.0.0.tar.gz
wget https://github.com/robotican/ric_interface_ros/archive/V1.0.0.tar.gz
tar -xvzf V1.0.0.tar.gz
rm V1.0.0.tar.gz
wget https://github.com/robotican/mobilican_macros/archive/V1.0.0.tar.gz
tar -xvzf V1.0.0.tar.gz
rm V1.0.0.tar.gz
wget https://github.com/elhayra/lpf_ros/archive/V1.0.0.tar.gz
tar -xvzf V1.0.0.tar.gz
rm V1.0.0.tar.gz

sudo dpkg -i lizi/lizi/ric_driver/ric-interface.deb

sudo apt-get -y install ros-kinetic-hector-gazebo-plugins

printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# realsense depth camera 
printf "${WHITE_TXT}\nInstalling depth camera...\n${NO_COLOR}"
wget https://github.com/intel-ros/realsense/archive/2.0.3.tar.gz
tar -xvzf 2.0.3.tar.gz
rm 2.0.3.tar.gz     
wget https://github.com/IntelRealSense/librealsense/archive/v2.10.3.tar.gz
tar -xvzf v2.10.3.tar.gz
rm v2.10.3.tar.gz
sudo apt-get -y install libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get -y install libglfw3-dev                                                                                                                                                
cd librealsense-2.10.3                                                                                                                                                               
mkdir build && cd build               
cmake ../  
sudo make uninstall && make clean && make -j8 && sudo make install
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# usb rules #
printf "${WHITE_TXT}\nInstalling USB rules...\n${NO_COLOR}"
sudo apt -y install setserial #for setting port latency
sudo cp $CATKIN_WS_SRC/lizi/rules/* /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
    

# compiling lizi #
printf "${WHITE_TXT}Compiling lizi package...\n${NO_COLOR}"
cd $CATKIN_WS_SRC
catkin_make -DCMAKE_BUILD_TYPE="Release"
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

printf "${GREEN_TXT}Installation process finished.\n\n"  ${NO_COLOR}
exit 0
