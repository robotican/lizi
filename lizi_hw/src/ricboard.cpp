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

    keepalive_timer = nh.createTimer(ros::Duration(RIC_DEAD_TIMEOUT), &RicBoard::onKeepAliveTimeout, this);
}

void RicBoard::onKeepAliveTimeout(const ros::TimerEvent &event)
{
    if (got_keepalive_)
        got_keepalive_ = false;
    else
        keepalive_timeouts_ ++;

    if (keepalive_timeouts_ >= MAX_KEEPALIVE_TIMEOUTS)
    {
        ROS_ERROR("[lizi_hw/ricboard_pub]: ricboard disconnected. shutting down...");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
}


void RicBoard::onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg)
{
    double new_pos = ((double)msg->ticks / ENC_TICKS_PER_ROUND) * 2.0 * M_PI;
    double new_time = ros::Time::now().toSec();
    int encoder_id = msg->id;

    switch (encoder_id)
    {
        case ENC_FRONT_RIGHT_ID:
            updateWheelState(front_right_wheel_, new_pos, new_time);
            break;

        case ENC_FRONT_LEFT_ID:
            updateWheelState(front_left_wheel_, new_pos, new_time);
            break;

        case ENC_REAR_RIGHT_ID:
            updateWheelState(rear_right_wheel_, new_pos, new_time);
            break;

        case ENC_REAR_LEFT_ID:
            updateWheelState(rear_left_wheel_, new_pos, new_time);
            break;
    }
}

void RicBoard::updateWheelState(wheel &wheel, double new_pos, double new_time)
{
    double delta_t = 0;
    double delta_x = 0;

    delta_t = new_time - wheel.timestamp;
    delta_x = new_pos - wheel.pos;
    wheel.vel = delta_x / delta_t;
    wheel.pos = new_pos;
    wheel.timestamp = new_time;
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
                               hardware_interface::VelocityJointInterface &vel_joint_interface,
                               hardware_interface::EffortJointInterface &effort_joint_interface)
{
    // joint state registration
    hardware_interface::JointStateHandle fr_wheel_handle(WHEEL_FRONT_RIGHT_JOINT,
                                                      &front_right_wheel_.pos,
                                                      &front_right_wheel_.vel,
                                                      &front_right_wheel_.effort);
    hardware_interface::JointStateHandle fl_wheel_handle(WHEEL_FRONT_LEFT_JOINT,
                                                   &front_left_wheel_.pos,
                                                   &front_left_wheel_.vel,
                                                    &front_left_wheel_.effort);
    hardware_interface::JointStateHandle rr_wheel_handle(WHEEL_REAR_RIGHT_JOINT,
                                                         &rear_right_wheel_.pos,
                                                         &rear_right_wheel_.vel,
                                                         &rear_right_wheel_.effort);
    hardware_interface::JointStateHandle rl_wheel_handle(WHEEL_REAR_LEFT_JOINT,
                                                         &rear_left_wheel_.pos,
                                                         &rear_left_wheel_.vel,
                                                         &rear_left_wheel_.effort);


    /*joint_state_handles_.push_back(fr_wheel_handle);
    joint_state_handles_.push_back(fl_wheel_handle);
    joint_state_handles_.push_back(rr_wheel_handle);
    joint_state_handles_.push_back(rl_wheel_handle);*/

    joint_state_interface.registerHandle(fr_wheel_handle);
    joint_state_interface.registerHandle(fl_wheel_handle);
    joint_state_interface.registerHandle(rr_wheel_handle);
    joint_state_interface.registerHandle(rl_wheel_handle);

    // register command interfaces

    hardware_interface::JointHandle fr_joint_handle(joint_state_interface.getHandle(WHEEL_FRONT_RIGHT_JOINT),
                                                    &front_right_wheel_.cmd_vel);
    hardware_interface::JointHandle fl_joint_handle(joint_state_interface.getHandle(WHEEL_FRONT_LEFT_JOINT),
                                                    &front_left_wheel_.cmd_vel);
    hardware_interface::JointHandle rr_joint_handle(joint_state_interface.getHandle(WHEEL_REAR_RIGHT_JOINT),
                                                    &rear_right_wheel_.cmd_vel);
    hardware_interface::JointHandle rl_joint_handle(joint_state_interface.getHandle(WHEEL_REAR_LEFT_JOINT),
                                                    &rear_left_wheel_.cmd_vel);

    // register handles to this class
    /*vel_handles_.push_back(fr_joint_handle);
    vel_handles_.push_back(fl_joint_handle);
    vel_handles_.push_back(rr_joint_handle);
    vel_handles_.push_back(rl_joint_handle);*/

    vel_joint_interface.registerHandle(fr_joint_handle);
    vel_joint_interface.registerHandle(fl_joint_handle);
    vel_joint_interface.registerHandle(rr_joint_handle);
    vel_joint_interface.registerHandle(rl_joint_handle);

    /*effort_handles_.push_back(fr_joint_handle);
    effort_handles_.push_back(fl_joint_handle);
    effort_handles_.push_back(rr_joint_handle);
    effort_handles_.push_back(rl_joint_handle);*/

    effort_joint_interface.registerHandle(fr_joint_handle);
    effort_joint_interface.registerHandle(fl_joint_handle);
    effort_joint_interface.registerHandle(rr_joint_handle);
    effort_joint_interface.registerHandle(rl_joint_handle);
}

void RicBoard::write()
{
    ric_interface_ros::Servo servo_msg;

    servo_msg.id = ENC_FRONT_RIGHT_ID;
    servo_msg.value = front_right_wheel_.effort;
    ric_servo_pub_.publish(servo_msg);

    servo_msg.id = ENC_FRONT_LEFT_ID;
    servo_msg.value = front_left_wheel_.effort;
    ric_servo_pub_.publish(servo_msg);

    servo_msg.id = ENC_REAR_RIGHT_ID;
    servo_msg.value = rear_right_wheel_.effort;
    ric_servo_pub_.publish(servo_msg);

    servo_msg.id = ENC_REAR_LEFT_ID;
    servo_msg.value = rear_left_wheel_.effort;
    ric_servo_pub_.publish(servo_msg);
}