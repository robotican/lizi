/*******************************************************************************
* Copyright (c) 2018 RoboTICan
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*     * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*     * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
* THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Elhay Rauper*/


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
        pids_[i].updateDynamicReconfig( pids_[i].getGains());
        pids_[i].reset();
    }

    start_time_ = ros::Time::now();

    pid_data_pub_ = nh.advertise<lizi_hw::WheelsPID>("wheels_pid", 10);
}

// dt duration is since pid controller start time
void WheelsControl::update(const ros::Duration& dt)
{
    lizi_hw::WheelsPID pid_msg;

    bool trigger_protection = false;

    for (int i=0; i < pids_.size(); i++)
    {
        double command = wheels_[i]->command_velocity;
        double velocity = wheels_[i]->velocity;
        double error = command - velocity;

        // round tiny errors to 0
        if (fabs(error) < 0.0001)
            error = 0;

        wheels_[i]->command_effort = pids_[i].computeCommand(error, dt);

 	// error thresh is relative to command
        float err_thresh_val = protect.error_thresh * command;
        if (fabs(error) >= fabs(err_thresh_val) &&
            fabs(wheels_[i]->command_effort) >= fabs(protect.output_thresh))
            trigger_protection = true;


        double pe=0, ie=0, de=0;
        pids_[i].getCurrentPIDErrors(&pe, &ie, &de);

        lizi_hw::WheelPID pid_data;
        pid_data.joint_name = wheels_[i]->joint_name;
        pid_data.command = command;
        pid_data.error = error;
        pid_data.output = wheels_[i]->command_effort;
        pid_data.raw_velocity = wheels_[i]->raw_velocity;
        pid_data.filtered_velocity = wheels_[i]->velocity;

        pid_msg.pids.push_back(pid_data);
    }

    if (!trigger_protection)
        protect.start_time = ros::Time::now();
    else
    {
        if ( (ros::Time::now() - protect.start_time) >
             ros::Duration(protect.time_thresh))
        {
            if (protect.enable)
                throw std::runtime_error("motor protection signal");
        }
    }

    pid_data_pub_.publish(pid_msg);
}

void WheelsControl::enableOVProtection(float time_thresh,
                                       float error_thresh,
                                       int output_thresh)
{
    if (error_thresh < 0 || error_thresh > 1)
        throw std::invalid_argument("error_thresh param must be between 0 and 1");
    if (output_thresh < 0 || output_thresh > 500)
        throw std::invalid_argument("output_thresh param must be between 0 and 500");
    protect.enable = true;
    protect.time_thresh = time_thresh;
    protect.error_thresh = error_thresh;
    protect.output_thresh = output_thresh;
    protect.start_time = ros::Time::now();
}
