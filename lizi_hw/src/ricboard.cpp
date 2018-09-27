/* Author: Elhay Rauper*/

#include "ricboard.h"

RicBoard::RicBoard(ros::NodeHandle &nh)
{
    nh_ = &nh;

    /* ric_interface_ros subscribers */
    encoder_sub_ = nh.subscribe("ric/encoder", 10, &RicBoard::onEncoderMsg, this);
    error_sub_ = nh.subscribe("ric/error", 10, &RicBoard::onErrorMsg, this);
    keepalive_sub_ = nh.subscribe("ric/keepalive", 10, &RicBoard::onKeepaliveMsg, this);
    orientation_sub_ = nh.subscribe("ric/orientation", 10, &RicBoard::onOrientationMsg, this);
    proximity_sub_ = nh.subscribe("ric/proximity", 10, &RicBoard::onProximityMsg, this);

    ric_servo_pub_ = nh.advertise<ric_interface_ros::Servo>("ric/servo/cmd", 10);

    /* ros publishers */
    urf_rear_pub_ = nh.advertise<sensor_msgs::Range>("urf/front", 10);
    urf_right_pub_ = nh.advertise<sensor_msgs::Range>("urf/right", 10);
    urf_left_pub_ = nh.advertise<sensor_msgs::Range>("urf/left", 10);
    ric_imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    ric_mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/magnetic", 10);
    espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);

    keepalive_timer_ = nh.createTimer(ros::Duration(RIC_DEAD_TIMEOUT), &RicBoard::onKeepAliveTimeout, this);

    front_right_wheel_.joint_name = WHEEL_FRONT_RIGHT_JOINT;
    front_left_wheel_.joint_name = WHEEL_FRONT_LEFT_JOINT;
    rear_right_wheel_.joint_name = WHEEL_REAR_RIGHT_JOINT;
    rear_left_wheel_.joint_name = WHEEL_REAR_LEFT_JOINT;

    front_right_wheel_.servo_id = SERVO_FRONT_RIGHT_ID;
    front_left_wheel_.servo_id = SERVO_FRONT_LEFT_ID;
    rear_right_wheel_.servo_id = SERVO_REAR_RIGHT_ID;
    rear_left_wheel_.servo_id = SERVO_REAR_LEFT_ID;

    if (!nh.getParam("lpf/front_right", front_right_wheel_.vel_lpf_alpha) ||
        !nh.getParam("lpf/front_left", front_left_wheel_.vel_lpf_alpha) ||
        !nh.getParam("lpf/rear_right", rear_right_wheel_.vel_lpf_alpha) ||
        !nh.getParam("lpf/rear_left", rear_left_wheel_.vel_lpf_alpha))
    {
        ROS_ERROR("[lizi_hw/ricboard]: one of the lpf params is missing. "
                  "check config file. shutting down.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    if (!nh.getParam("reverse_command/front_right", front_right_wheel_.reverse_command) ||
        !nh.getParam("reverse_command/front_left", front_left_wheel_.reverse_command) ||
        !nh.getParam("reverse_command/rear_right", rear_right_wheel_.reverse_command) ||
        !nh.getParam("reverse_command/rear_left", rear_left_wheel_.reverse_command))
    {
        ROS_ERROR("[lizi_hw/ricboard]: one of the reverse_command params is missing. "
                  "check config file. shutting down.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    if (!nh.getParam("reverse_feedback/front_right", front_right_wheel_.reverse_feedback) ||
        !nh.getParam("reverse_feedback/front_left", front_left_wheel_.reverse_feedback) ||
        !nh.getParam("reverse_feedback/rear_right", rear_right_wheel_.reverse_feedback) ||
        !nh.getParam("reverse_feedback/rear_left", rear_left_wheel_.reverse_feedback))
    {
        ROS_ERROR("[lizi_hw/ricboard]: one of the reverse_feedback params is missing. "
                  "check config file. shutting down.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    std::vector<wheel*> wheels;
    wheels.push_back(&rear_right_wheel_);
    wheels.push_back(&rear_left_wheel_);
    wheels.push_back(&front_right_wheel_);
    wheels.push_back(&front_left_wheel_);

    wheels_control_.init(nh, wheels);

    vels_lpf_.init(nh, wheels);

    double motor_max_rpm = 0;
    if (!nh.getParam("motor_max_rpm", motor_max_rpm))
    {
        ROS_ERROR("[lizi_hw/ricboard]: motor_max_rpm param is missing. "
                  "check config file. shutting down.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
    motor_max_vel_ = rpmToRadPerSec(motor_max_rpm);

    nh.getParam("ric_servo_bias", ric_servo_bias_);

    nh.getParam("velocity_delta_t", vel_delta_t_);

    vel_delta_timer_ = nh.createTimer(ros::Duration(vel_delta_t_), &RicBoard::onVelDeltaTimer, this);
}

void RicBoard::onVelDeltaTimer(const ros::TimerEvent &)
{
    for (auto &wheel : wheels_control_.getWheels())
    {
        double delta_x = wheel->position - wheel->last_position;
        wheel->velocity = delta_x / vel_delta_t_;
        wheel->last_position = wheel->position;
    }
    vels_lpf_.update();





    wheels_control_.update(ros::Duration(vel_delta_t_));

    for (wheel *w : wheels_control_.getWheels())
    {
        if (w->reverse_command)
            w->reverse_command *= -1;

        // map control loop values to servo values
        double servo_command = map(w->command_effort,
                                   -motor_max_vel_,
                                   motor_max_vel_,
                                   SERVO_MIN,
                                   SERVO_MAX);

        // add bias if needed
        if (servo_command > SERVO_NEUTRAL)
            servo_command += ric_servo_bias_;
        else if (servo_command < SERVO_NEUTRAL)
            servo_command -= ric_servo_bias_;

        // sturation
        if (servo_command > SERVO_MAX)
            servo_command = SERVO_MAX;
        else if (servo_command < SERVO_MIN)
            servo_command = SERVO_MIN;

        ROS_INFO("%s_cmd: %f | vel: %f | ric_cmd: %f",
                 w->joint_name.c_str(),
                 w->command_velocity,
                 w->velocity,
                 servo_command);

        // send servo commands to ricboard
        ric_interface_ros::Servo servo_msg;

        servo_msg.id = w->servo_id;
        servo_msg.value = servo_command;
        ric_servo_pub_.publish(servo_msg);
    }
}

void RicBoard::onKeepAliveTimeout(const ros::TimerEvent &event)
{
    if (got_keepalive_)
        got_keepalive_ = false;
    else
    {
        std_msgs::String str_msg;
        str_msg.data = "RIK BOARD DISCONNECTED. SHUTTING DOWN";
        espeak_pub_.publish(str_msg);
        ROS_ERROR("[lizi_hw/ricboard_pub]: ricboard disconnected. shutting down...");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
}


void RicBoard::onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg)
{
    double new_pos = ((double)msg->ticks / ENC_TICKS_PER_ROUND) * 2.0 * M_PI;
    int encoder_id = msg->id;

    switch (encoder_id)
    {
        case ENC_FRONT_RIGHT_ID:
            updateWheelPosition(front_right_wheel_, new_pos);
            break;

        case ENC_FRONT_LEFT_ID:
            updateWheelPosition(front_left_wheel_, new_pos);
            break;

        case ENC_REAR_RIGHT_ID:
            updateWheelPosition(rear_right_wheel_, new_pos);
            break;

        case ENC_REAR_LEFT_ID:
            updateWheelPosition(rear_left_wheel_, new_pos);
            break;
    }
}

void RicBoard::updateWheelPosition(wheel &wheel, double new_pos)
{
    if (wheel.reverse_feedback)
        new_pos *= -1;

    wheel.position = new_pos;
}

void RicBoard::onErrorMsg(const ric_interface_ros::Error::ConstPtr& msg)
{
    ROS_WARN("[lizi_hw/ricboard]: component of type %i and id %i reported an error code %i",
             msg->comp_type, msg->comp_id, msg->code);
}

void RicBoard::onKeepaliveMsg(const ric_interface_ros::Keepalive::ConstPtr& msg)
{
    got_keepalive_ = true;
    keepalive_timeouts_ = 0;
}

void RicBoard::onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg)
{
    /* publish imu */
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_footprint";

    double roll, pitch, yaw;
    pitch = -msg->roll;
    roll = -msg->pitch;
    yaw = msg->yaw - M_PI / 2;

    //wrap to PI
    if (yaw > M_PI )
        yaw -= 2 * M_PI;
    else if (yaw < -M_PI)
        yaw += 2 * M_PI;

    tf::Quaternion orientation_q =
            tf::createQuaternionFromRPY(roll, pitch, yaw);

    imu_msg.orientation.x = orientation_q.x();
    imu_msg.orientation.y = orientation_q.y();
    imu_msg.orientation.z = orientation_q.z();
    imu_msg.orientation.w = orientation_q.w();
    imu_msg.angular_velocity.x = -1 * msg->gyro_y;
    imu_msg.angular_velocity.y = -1 * msg->gyro_x;
    imu_msg.angular_velocity.z = -1 * msg->gyro_z;
    imu_msg.linear_acceleration.x = msg->accl_x * G_FORCE + ACCEL_OFFSET_X;
    imu_msg.linear_acceleration.y = msg->accl_y * G_FORCE - ACCEL_OFFSET_Y;
    imu_msg.linear_acceleration.z = msg->accl_z * G_FORCE - ACCEL_OFFSET_Z;
    ric_imu_pub_.publish(imu_msg);

    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp = ros::Time::now();
    mag_msg.header.frame_id = "base_link";
    mag_msg.magnetic_field.x = msg->mag_x;
    mag_msg.magnetic_field.y = msg->mag_y;
    mag_msg.magnetic_field.z = msg->mag_z;
    ric_mag_pub_.publish(mag_msg);
}

void RicBoard::onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg)
{
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.min_range = URF_MIN_RANGE;
    range_msg.max_range = URF_MAX_RANGE;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = msg->distance / 1000.0;
    range_msg.field_of_view = URF_FOV;

    int urf_id = msg->id;

    switch (urf_id)
    {
        case URF_REAR_ID:
            range_msg.header.frame_id = "urf_rear_link";
            urf_rear_pub_.publish(range_msg);
            break;

        case URF_RIGHT_ID:
            range_msg.header.frame_id = "urf_right_link";
            urf_right_pub_.publish(range_msg);
            break;

        case URF_LEFT_ID:
            range_msg.header.frame_id = "urf_left_link";
            urf_left_pub_.publish(range_msg);
            break;
    }
}

void RicBoard::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                               hardware_interface::VelocityJointInterface &vel_joint_interface)
{
    for (wheel *w : wheels_control_.getWheels())
    {
        hardware_interface::JointStateHandle state_handle(w->joint_name,
                                                             &w->position,
                                                             &w->velocity,
                                                             &w->effort);
        joint_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle joint_handle(joint_state_interface.getHandle(w->joint_name),
                                                        &w->command_velocity);
        vel_joint_interface.registerHandle(joint_handle);
    }
}

void RicBoard::read(const ros::Time &now)
{

}

void RicBoard::write(const ros::Time &now, const ros::Duration& duration)
{
    // limit writings to arduino to 50Hz
//    if (now - prev_write_time_ >= ros::Duration(0.02))
//    {
//        wheels_control_.update(duration);
//
//        for (wheel *w : wheels_control_.getWheels())
//        {
//            if (w->reverse_command)
//                w->reverse_command *= -1;
//
//            // map control loop values to servo values
//            double servo_command = map(w->command_effort,
//                                               -motor_max_vel_,
//                                               motor_max_vel_,
//                                               SERVO_MIN,
//                                               SERVO_MAX);
//
//            // add bias if needed
//            if (servo_command > SERVO_NEUTRAL)
//                servo_command += ric_servo_bias_;
//            else if (servo_command < SERVO_NEUTRAL)
//                servo_command -= ric_servo_bias_;
//
//            // sturation
//            if (servo_command > SERVO_MAX)
//                servo_command = SERVO_MAX;
//            else if (servo_command < SERVO_MIN)
//                servo_command = SERVO_MIN;
//
//            ROS_INFO("%s_cmd: %f | vel: %f | ric_cmd: %f",
//                     w->joint_name.c_str(),
//                     w->command_velocity,
//                     w->velocity,
//                     servo_command);
//
//            // send servo commands to ricboard
//            ric_interface_ros::Servo servo_msg;
//
//            servo_msg.id = w->servo_id;
//            servo_msg.value = servo_command;
//            ric_servo_pub_.publish(servo_msg);
//        }
//
//        prev_write_time_ = ros::Time::now();
//    }
}

double RicBoard::map(double value, double in_min, double in_max, double out_min, double out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double RicBoard::rpmToRadPerSec(double rpm)
{
    return ((2 * M_PI) / 60) * rpm;
}