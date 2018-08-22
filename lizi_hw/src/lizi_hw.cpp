/* Author: Elhay Rauper*/

#include "lizi_hw.h"

LiziHW::LiziHW(ros::NodeHandle &nh) : ric_(nh)
{
    node_handle_ = &nh;

    ric_.registerHandles(joint_state_interface_,
                         vel_joint_interface_,
                         effort_joint_interface_);

    registerInterface(&joint_state_interface_);
    registerInterface(&vel_joint_interface_);
    registerInterface(&effort_joint_interface_);

    last_write_time_ = ros::Time::now();

    ROS_INFO("[lizi_hw]: lizi hardware interface is up");
}

void LiziHW::write()
{
    ros::Duration duration = ros::Time::now() - last_write_time_;

    // limit writings to arduino to 50Hz
    if (duration > ros::Duration(0.02))
        ric_.write();
}