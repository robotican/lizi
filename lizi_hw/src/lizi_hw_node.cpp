
/* Author: Elhay Rauper*/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "lizi_hw.h"

#define THREADS_NUM 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lizi_hw_node");
    ros::NodeHandle nh;

    LiziHW lizi_hw(nh);
    controller_manager::ControllerManager controller_manager(&lizi_hw);

    ros::AsyncSpinner asyncSpinner(THREADS_NUM);
    asyncSpinner.start();

    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Duration duration = ros::Time::now() - last_time;

        controller_manager.update(ros::Time::now(), duration);

        lizi_hw.write();

        last_time = ros::Time::now();

        ros::spinOnce;
        ros::Rate(100).sleep();
    }
}