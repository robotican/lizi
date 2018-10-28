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

cd $CATKIN_WS_SRC

printf "${WHITE_TXT}\nInstalling 3rd party packages...\n${NO_COLOR}"
# third party packages #
sudo apt-get -y update
sudo apt-get -y dist-upgrade 
sudo apt-get -y upgrade 

# from this point on, exit and notify immediately if a command exits with a non-zero status
set -eb

sudo apt-get -y install ros-kinetic-controller-manager \
ros-kinetic-control-toolbox \
ros-kinetic-transmission-interface \
ros-kinetic-joint-limits-interface \
ros-kinetic-ros-controllers \
ros-kinetic-ros-control \
ros-kinetic-move-base \
ros-kinetic-navigation \
ros-kinetic-hector-slam \
ros-kinetic-gmapping \
ros-kinetic-pid \
ros-kinetic-ar-track-alvar \
ros-kinetic-serial \
ros-kinetic-robot-localization \
ros-kinetic-trac-ik ros-kinetic-moveit-kinematics  \
ros-kinetic-urg-node \
ros-kinetic-usb-cam \
ros-kinetic-rqt-robot-monitor \
ros-kinetic-hector-gazebo-plugins \
espeak espeak-data libespeak-dev

DIFF_SLIP_CONTROLLER_V="1.0.0"
wget https://github.com/robotican/diff_drive_slip_controller/archive/V"$DIFF_SLIP_CONTROLLER_V".tar.gz
tar -xvzf V"$DIFF_SLIP_CONTROLLER_V".tar.gz
rm V"$DIFF_SLIP_CONTROLLER_V".tar.gz

RIC_INTERFACE_ROS_V="1.0.3"
wget https://github.com/robotican/ric_interface_ros/archive/V"$RIC_INTERFACE_ROS_V".tar.gz
tar -xvzf V"$RIC_INTERFACE_ROS_V".tar.gz
rm V"$RIC_INTERFACE_ROS_V".tar.gz

MOBILICAN_MACROS_V="1.0.0"
wget https://github.com/robotican/mobilican_macros/archive/V"$MOBILICAN_MACROS_V".tar.gz
tar -xvzf V"$MOBILICAN_MACROS_V".tar.gz
rm V"$MOBILICAN_MACROS_V".tar.gz

LPF_ROS_V="1.0.0"
wget https://github.com/elhayra/lpf_ros/archive/V"$LPF_ROS_V".tar.gz
tar -xvzf V"$LPF_ROS_V".tar.gz
rm V"$LPF_ROS_V".tar.gz

ESPEAK_ROS_V="1.0.2"
wget https://github.com/robotican/espeak_ros/archive/V"$ESPEAK_ROS_V".tar.gz
tar -xvzf V"$ESPEAK_ROS_V".tar.gz
rm V"$ESPEAK_ROS_V".tar.gz

MOBILICAN_RULES_V="1.0.0"
wget https://github.com/robotican/mobilican_rules/archive/V"$MOBILICAN_RULES_V".tar.gz
tar -xvzf V"$MOBILICAN_RULES_V".tar.gz
rm V"$MOBILICAN_RULES_V".tar.gz

#install ric_interface deb
cd $CATKIN_WS_SRC/lizi/ric-interface-ros-$RIC_INTERFACE_ROS_V/ric_interface_deb/
sudo dpkg -i ric-interface.deb

printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# realsense depth camera 
printf "${WHITE_TXT}\nInstalling depth camera...\n${NO_COLOR}"
cd $CATKIN_WS_SRC/lizi/
wget https://github.com/intel-ros/realsense/archive/2.0.3.tar.gz
tar -xvzf 2.0.3.tar.gz
rm 2.0.3.tar.gz     
wget https://github.com/IntelRealSense/librealsense/archive/v2.10.3.tar.gz
tar -xvzf v2.10.3.tar.gz
rm v2.10.3.tar.gz
sudo apt-get -y install libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev                                                                                                                                                
cd librealsense-2.10.3                                                                                                                                                               
mkdir build && cd build               
cmake ../  
sudo make uninstall && make clean && make -j8 && sudo make install
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# usb rules #
printf "${WHITE_TXT}\nInstalling USB rules...\n${NO_COLOR}"
sudo apt -y install setserial #for setting port latency
sudo cp $CATKIN_WS_SRC/mobilican_rules-$MOBILICAN_RULES_V/rules/* /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
    

# compiling lizi #
printf "${WHITE_TXT}Compiling lizi package...\n${NO_COLOR}"
cd $CATKIN_WS_SRC/..
catkin_make -DCMAKE_BUILD_TYPE="Release"
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

printf "${GREEN_TXT}Installation process finished.\n\n"  ${NO_COLOR}
exit 0
