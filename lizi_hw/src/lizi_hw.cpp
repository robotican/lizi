/* Author: Elhay Rauper*/

#include "lizi_hw.h"

LiziHW::LiziHW(ros::NodeHandle &nh) : ric_(nh)
{
    node_handle_ = &nh;

    ric_.registerHandles(joint_state_interface_,
                         vel_joint_interface_);

    registerInterface(&joint_state_interface_);
    registerInterface(&vel_joint_interface_);

    ROS_INFO("[lizi_hw]: lizi hardware interface is up");
}

void LiziHW::write(const ros::Time &time, const ros::Duration& duration)
{
    ric_.write(time, duration);
}

void LiziHW::read(const ros::Time &time)
{
    ric_.read(time);
}