# lizi

## Installation:

1. download [latest release](https://github.com/robotican/lizi/releases) into your catkin workspace src folder (e.g. ~/catkin_ws/src) 

2. cd into lizi meta package. e.g. : 
```
cd ~/catkin_ws/src/lizi/lizi
```

3. During installation setup script will download 3rd party packages. Make sure you have internet connection


3. execute the installation script:
```
./setup.sh
```
*Note: setup script will execute apt-get upgrade and update

## Cheat sheet

### Robot launch command

Basic launch:
```
roslaunch lizi lizi.launch
```

You can add arguments to the basic launch command, to enable capabilities. Some commonly used arguments:

```gazebo``` - launch robot in gazebo simulation

```cam``` - launch front rgb camera

```depth_cam``` - launch front 3d camera

```lidar``` - launch front 2d LIDAR

```diagnos``` - publish robot diagnostics (these can be monitored through rqt_robot_monitor)

```move_base``` - launch move_base package

```map``` - load map to map_server

```gmapping``` - launch gmapping SLAM algorithm. Must be executed with move_base and lidar

```hector_slam``` - launch Hector SLAM algorithm. Must be executed with move_base and lidar

```amcl``` - launch AMCL algorithm for localization. Must be executed with move_base, lidar and map


e.g. :
```
roslaunch lizi lizi.launch gazebo:=true gmapping:=true lidar:=true move_base:=true
```

### Topics

You can see the full list of available topics using ```rostopic list``` .
Please disregard topics that starts with ```/ric/...``` as they are used for system debugging and communication b/w lizi hardware drivers and ros.

Here is a list of the most commonly used topics:

```/rgb_cam/image_raw``` - front rgb camera raw image

```/camera/color/image_raw``` - front 3d camera raw (2d) image

```/camera/depth/color/points``` - front 3d camera point cloud

```/gps``` - gps data (if gps got fix)

```/imu/data``` - IMU data

```/scan``` - LIDAR 2d scan

```/urf/left``` - Left ultrasonic range finder

```/urf/right``` - Right ultrasonic range finder

```/urf/rear``` - Rear ultrasonic range finder

```/battery``` - Battery info

```/mobile_base_controller/cmd_vel``` - Send twist commands on this topic to drive the robot


*Please note: some of these topics will only apear when adding the right argument to the launch command. E.g. rgb camera topics will apear when ```cam``` argument is added to the basic launch command.
