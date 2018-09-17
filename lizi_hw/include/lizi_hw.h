/* Author: Elhay Rauper*/

#ifndef LIZI_HW_LIZI_HW_H
#define LIZI_HW_LIZI_HW_H

#include "ricboard.h"
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <std_msgs/String.h>

class LiziHW : public hardware_interface::RobotHW
{
private:

    ros::NodeHandle *node_handle_;

    hardware_interface::JointStateInterface joint_state_interface_;

    hardware_interface::VelocityJointInterface vel_joint_interface_;

    RicBoard ric_;

    void registerInterfaces();

public:

    LiziHW(ros::NodeHandle &nh);

    void write(const ros::Time &time, const ros::Duration& duration);
    void read(const ros::Time &time);

};


#endif //LIZI_HW_LIZI_HW_H
