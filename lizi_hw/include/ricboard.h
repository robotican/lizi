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

#ifndef LIZI_HW_RICBOARD_H
#define LIZI_HW_RICBOARD_H

#include <ric_interface_ros/Encoder.h>
#include <ric_interface_ros/Keepalive.h>
#include <ric_interface_ros/Orientation.h>
#include <ric_interface_ros/Proximity.h>
#include <ric_interface_ros/Servo.h>
#include <ric_interface_ros/Toggle.h>
#include <ric_interface_ros/Logger.h>
#include <ric_interface_ros/Location.h>
#include <ric_interface_ros/Battery.h>
#include <ric_interface/protocol.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/BatteryState.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/String.h>
#include <boost/algorithm/clamp.hpp>
#include <math.h>
#include <exception>
#include "wheel.h"
#include "wheels_control.h"
#include "velocities_lpf.h"

#define RIC_DEAD_TIMEOUT            1.5 //secs
#define G_FORCE                     9.80665
#define ENC_TICKS_PER_ROUND         4480 // 64 * 70

#define URF_MIN_RANGE               0.3
#define URF_MAX_RANGE               3.0
#define URF_FOV                     0.7f

#define WHEEL_FRONT_LEFT_JOINT      "front_left_wheel_joint"
#define WHEEL_FRONT_RIGHT_JOINT     "front_right_wheel_joint"
#define WHEEL_REAR_LEFT_JOINT       "rear_left_wheel_joint"
#define WHEEL_REAR_RIGHT_JOINT      "rear_right_wheel_joint"

#define WHEEL_FRONT_RIGHT_ID          1
#define WHEEL_FRONT_LEFT_ID           2
#define WHEEL_REAR_RIGHT_ID           3
#define WHEEL_REAR_LEFT_ID            4

#define URF_REAR_ID                 10
#define URF_RIGHT_ID                11
#define URF_LEFT_ID                 12

#define IMU_ID                      20

#define ACCEL_OFFSET_X              0.23
#define ACCEL_OFFSET_Y              0.13
#define ACCEL_OFFSET_Z              0.1

#define BATT_MAX                   16.7
#define BATT_MIN                   12.8
#define BATT_CELLS                 4

class RicBoard
{

private:

    bool got_keepalive_ = false;
    bool first_keepalive_ = true;

    ros::Subscriber encoder_sub_,
                    keepalive_sub_,
                    orientation_sub_,
                    proximity_sub_,
                    logger_sub_,
                    location_sub_,
                    battery_sub_;

    ros::Publisher urf_rear_pub_,
            urf_right_pub_,
            urf_left_pub_;
    ros::Publisher imu_pub_,
            mag_pub_,
            ric_servo_pub_,
            gps_pub_,
            battery_pub_;
    ros::Publisher espeak_pub_;
    ros::Publisher diagnos_pub_;

    ros::ServiceClient terminate_ric_client_;

    ros::Timer ric_pub_timer_,
            keepalive_timer_,
            vel_delta_timer_;

    ros::NodeHandle *nh_;

    WheelsControl wheels_control_;

    VelocitiesLpf vels_lpf_;

    wheel front_right_wheel_,
            front_left_wheel_,
            rear_right_wheel_,
            rear_left_wheel_;

    double ric_servo_bias_ = 0;

    double control_loop_interval_ = 0;

    ros::Time prev_lpf_time_;

    void onKeepAliveTimeout(const ros::TimerEvent &event);

    void onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg);
    void onKeepaliveMsg(const ric_interface_ros::Keepalive::ConstPtr& msg);
    void onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg);
    void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg);
    void onLoggerMsg(const ric_interface_ros::Logger::ConstPtr& msg);
    void onLocationMsg(const ric_interface_ros::Location::ConstPtr& msg);
    void onBatteryMsg(const ric_interface_ros::Battery::ConstPtr& msg);

    static void updateWheelPosition(wheel &wheel, double new_pos);

    void onControlLoopTimer(const ros::TimerEvent &);

    void sendDiagnosticsMsg(const diagnostic_msgs::DiagnosticStatus & status);


public:

    RicBoard(ros::NodeHandle &nh);

    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::VelocityJointInterface& vel_joint_interface);

    void write(const ros::Time &now, const ros::Duration& duration);
    void read(const ros::Time &now);

    static double ticksToRads(int32_t icks);

    void speakMsg(const std::string& msg);

    void terminateWithMessage(const char * msg, bool speak);
};


#endif //LIZI_HW_RICBOARD_H
