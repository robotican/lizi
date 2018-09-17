/* Author: Elhay Rauper*/


#ifndef LIZI_HW_RICBOARD_H
#define LIZI_HW_RICBOARD_H

#include <ric_interface_ros/Encoder.h>
#include <ric_interface_ros/Error.h>
#include <ric_interface_ros/Keepalive.h>
#include <ric_interface_ros/Orientation.h>
#include <ric_interface_ros/Proximity.h>
#include <ric_interface_ros/Servo.h>
#include <ric_interface_ros/Toggle.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <hardware_interface/joint_state_interface.h>
#include <math.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/String.h>
#include "wheel.h"
#include "wheels_control.h"
#include "velocities_lpf.h"

#define RIC_DEAD_TIMEOUT            1.5 //secs
#define G_FORCE                     9.80665
#define ENC_TICKS_PER_ROUND         2048 // 512 * 4

#define URF_MIN_RANGE               0.3
#define URF_MAX_RANGE               3.0
#define URF_FOV                     0.7f

#define WHEEL_FRONT_LEFT_JOINT      "front_left_wheel_joint"
#define WHEEL_FRONT_RIGHT_JOINT     "front_right_wheel_joint"
#define WHEEL_REAR_LEFT_JOINT       "rear_left_wheel_joint"
#define WHEEL_REAR_RIGHT_JOINT      "rear_right_wheel_joint"

#define ENC_FRONT_RIGHT_ID          1
#define ENC_FRONT_LEFT_ID           2
#define ENC_REAR_RIGHT_ID           3
#define ENC_REAR_LEFT_ID            4

#define URF_REAR_ID                 14
#define URF_RIGHT_ID                15
#define URF_LEFT_ID                 16

#define SERVO_FRONT_RIGHT_ID	30
#define SERVO_FRONT_LEFT_ID		31
#define SERVO_REAR_RIGHT_ID		32
#define SERVO_REAR_LEFT_ID		33

#define IMU_ID                      20

#define ACCEL_OFFSET_X              0.23
#define ACCEL_OFFSET_Y              0.13
#define ACCEL_OFFSET_Z              0.1

#define SERVO_MAX                   2000
#define SERVO_MIN                   1000

class RicBoard
{

private:

    int keepalive_timeouts_ = 0;
    bool got_keepalive_ = false;

    ros::Subscriber encoder_sub_,
                    error_sub_,
                    keepalive_sub_,
                    orientation_sub_,
                    proximity_sub_;

    ros::Publisher urf_rear_pub_,
            urf_right_pub_,
            urf_left_pub_;
    ros::Publisher ric_imu_pub_,
            ric_mag_pub_,
            ric_servo_pub_;
    ros::Publisher espeak_pub_;

    ros::Timer ric_pub_timer_,
            keepalive_timer;

    ros::NodeHandle *nh_;

    WheelsControl wheels_control_;

    VelocitiesLpf vels_lpf_;

    wheel front_right_wheel_,
            front_left_wheel_,
            rear_right_wheel_,
            rear_left_wheel_;

    double motor_max_vel_ = 0; // rad/s

    ros::Time prev_write_time_, prev_read_time_;

    void onKeepAliveTimeout(const ros::TimerEvent &event);

    void onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg);
    void onErrorMsg(const ric_interface_ros::Error::ConstPtr& msg);
    void onKeepaliveMsg(const ric_interface_ros::Keepalive::ConstPtr& msg);
    void onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg);
    void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg);

    static void updateWheelState(wheel &wheel, double new_pos);


public:

    RicBoard(ros::NodeHandle &nh);

    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::VelocityJointInterface& vel_joint_interface);

    // write controllers commands to ricboard
    void write(const ros::Time &now, const ros::Duration& duration);

    void read(const ros::Time &now);

    static double map(double value, double in_min, double in_max, double out_min, double out_max);
    static double rpmToRadPerSec(double rpm);
};


#endif //LIZI_HW_RICBOARD_H
