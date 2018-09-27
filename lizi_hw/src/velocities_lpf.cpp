

#include "velocities_lpf.h"


void VelocitiesLpf::init(ros::NodeHandle &nh, std::vector<wheel*> & wheels)
{

    wheels_ = wheels;
    // initiate pid controls with pid from param server
    for(auto & wheel : wheels)
    {
        filters_.push_back(lpf::Lpf());
    }

    for (int i=0; i < filters_.size(); i++)
    {
        try
        {
            filters_[i].init(
                    ros::NodeHandle(nh, std::string(wheels_[i]->joint_name + "/lpf")),
                    wheels_[i]->vel_lpf_alpha
            );
        }
        catch (std::invalid_argument exp)
        {
            ROS_ERROR("[lizi_hw/velocities_lpf]: wheel %s error: %s. shutting down.",
                      wheels_[i]->joint_name.c_str(),
                      exp.what());
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
    }
}

void VelocitiesLpf::update()
{
    for (int i=0; i < filters_.size(); i++)
    {
        double raw_vel = wheels_[i]->velocity;
        double filtered_vel = filters_[i].filter(raw_vel);
        wheels_[i]->velocity = filtered_vel;
    }
}
