/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include "ros_interface.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <sensor_msgs/Joy.h>


// static uint32_t HAL_GetTick() {
//     static auto time_start = std::chrono::steady_clock::now();
//     auto time_now = std::chrono::steady_clock::now();
//     auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_start).count();
//     return elapsed_time_ms;
// }

RosInterface::RosInterface(int argc, char** argv) {
    ros::init(argc, argv, "cyphal_communicator");
    ros_node = new ros::NodeHandle;
}

bool RosInterface::init() {
    _setpoint_pub = ros_node->advertise<sensor_msgs::Joy>("/uav/actuators_raw", 1);

    static auto _imu_sub = ros_node->subscribe("/uav/imu", 1, &RosInterface::_imu_cb, this);
    static auto _mag_sub = ros_node->subscribe("/uav/mag", 1, &RosInterface::_mag_cb, this);

    static auto _baro_temp_sub = ros_node->subscribe("/uav/static_temperature", 1, &RosInterface::_baro_temp_cb, this);
    static auto _baro_pres_sub = ros_node->subscribe("/uav/static_pressure", 1, &RosInterface::_baro_pres_cb, this);

    static auto _gps_point_sub = ros_node->subscribe("/uav/gps_point", 1, &RosInterface::_gps_point_cb, this);
    static auto _gps_velocity_sub = ros_node->subscribe("/uav/velocity", 1, &RosInterface::_gps_velocity_cb, this);

    return true;
}

bool RosInterface::send_setpoint(const Setpoint16& setpoint) {
    sensor_msgs::Joy ros_msg;
    ros_msg.header.stamp = ros::Time::now();
    for (uint_fast8_t idx = 0; idx < 16; idx++) {
        ros_msg.axes.push_back(std::clamp(setpoint[idx], -1.0f, +1.0f));
    }
    _setpoint_pub.publish(ros_msg);
    return true;
}

void RosInterface::_imu_cb(const sensor_msgs::Imu& msg) {
    Vector3 accel{msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z};
    Vector3 gyro{msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
    // ROS_ERROR("get imu from ros");
    // fflush(stdout);
    for (auto imu_cb : imu_callbacks) {
        imu_cb(accel, gyro);
    }
}

void RosInterface::_mag_cb(const sensor_msgs::MagneticField& msg) {
    Vector3 magnetic_field_gauss{msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z};
    // ROS_ERROR("get mag from ros");
    // fflush(stdout);
    for (auto mag_cb : magnetometer_callbacks) {
        mag_cb(magnetic_field_gauss);
    }
}

void RosInterface::_baro_temp_cb(const std_msgs::Float32& msg) {
    (void)msg;
}

void RosInterface::_baro_pres_cb(const std_msgs::Float32& msg) {
    for (auto baro_cb : baro_callbacks) {
        baro_cb(msg.data, 312);
    }
}

void RosInterface::_gps_point_cb(const sensor_msgs::NavSatFix& msg) {
    // ROS_ERROR("get gps from ros");
    // fflush(stdout);
    Vector3 global_pose{msg.latitude, msg.longitude, msg.altitude};
    for (auto gnss_cb : gnss_callbacks) {
        gnss_cb(global_pose, _velocity);
    }
}

void RosInterface::_gps_velocity_cb(const geometry_msgs::Twist& msg) {
    _velocity[0] = msg.linear.x;
    _velocity[1] = msg.linear.y;
    _velocity[2] = msg.linear.z;
}

bool RosInterface::receive_sensors() {
    ros::spinOnce();
    return true;
}
