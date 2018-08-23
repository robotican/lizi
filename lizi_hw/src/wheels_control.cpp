//
// Created by sub on 8/22/18.
//

#include "wheels_control.h"

void WheelsControl::init(ros::NodeHandle &nh, std::vector<wheel*> & wheels)
{
    wheels_ = wheels;

    // initiate pid controls with pid from param server
    for(auto & wheel : wheels)
    {
        pids_.push_back(control_toolbox::Pid());
    }

    for (int i=0; i < pids_.size(); i++)
    {
        if (!pids_[i].init(ros::NodeHandle(nh, wheels_[i]->joint_name), false))
        {
            ROS_ERROR("[lizi_hw/wheels_control]: pid params of %s is missing. "
                      "check config file. shutting down.", wheels_[i]->joint_name.c_str());
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        pids_[i].reset();
    }

    start_time_ = ros::Time::now();
}

void WheelsControl::update(const ros::Duration& dt)
{
    // duration since pid controller start time

    for (int i=0; i < pids_.size(); i++)
    {
        double command = wheels_[i]->command_velocity;
        double velocity = wheels_[i]->velocity;
        double error = command  - velocity;

        ROS_INFO("cmd: %f, vel: %f, error: %f, dt: %f", command, velocity, error, dt.toSec());

        wheels_[i]->command_effort = pids_[i].computeCommand(error, dt);

        pids_[i].setCurrentCmd(command);
    }
}
