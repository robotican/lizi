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

#ifndef LIZI_HW_PID_CONTROL_H
#define LIZI_HW_PID_CONTROL_H

#include <exception>
#include <control_toolbox/pid.h>
#include <lizi_hw/WheelsPID.h>
#include <lizi_hw/WheelPID.h>
#include "wheel.h"

// This class takes 4 wheels velocity commands from diff_drive_controller,
// and output effort command for each one, so it can be sent to RicBoard

struct ov_protection_settings
{
    ros::Time start_time;
    bool enable = false;
    float error_thresh = 0;
    float time_thresh = 0;
    int output_thresh = 0;
};

class WheelsControl
{
private:

    std::vector<wheel*> wheels_;

    std::vector<control_toolbox::Pid> pids_;

    ros::Time start_time_;

    ros::Publisher pid_data_pub_;

    ov_protection_settings protect;

public:
    void init(ros::NodeHandle &nh, std::vector<wheel*> & wheels);

    void update(const ros::Duration& dt);

    std::vector<wheel*> getWheels() { return wheels_; }

    // enable over voltage protection. if error > error_thresh and
    // time > time_thresh and output > output_thresh, then throw runtime exception
    // @param time_thresh - in seconds
    // @param error_thresh - percentage 0-1
    // @param output_thresh - 0-500
    void enableOVProtection(float time_thresh,
                            float error_thresh,
                            int output_thresh);

};


#endif //LIZI_HW_PID_CONTROL_H
