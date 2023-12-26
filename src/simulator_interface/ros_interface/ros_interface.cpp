// This software is distributed under the terms of the GPL v3 License.
// Copyright (c) 2023 Dmitry Ponomarev.
// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

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


RosInterface::RosInterface(int argc, char** argv) {
    ros::init(argc, argv, "cyphal_communicator");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros_node = new ros::NodeHandle;
}

bool RosInterface::init() {
    _setpoint_pub = ros_node->advertise<sensor_msgs::Joy>("/uav/actuators_raw", 1);
    _arm_pub = ros_node->advertise<std_msgs::Bool>("/uav/arm", 1);

    static auto _imu_sub = ros_node->subscribe("/uav/imu", 1, &RosInterface::_imu_cb, this);
    static auto _mag_sub = ros_node->subscribe("/uav/mag", 1, &RosInterface::_mag_cb, this);

    static auto _baro_temp_sub = ros_node->subscribe("/uav/static_temperature", 1, &RosInterface::_baro_temp_cb, this);
    static auto _baro_pres_sub = ros_node->subscribe("/uav/static_pressure", 1, &RosInterface::_baro_pres_cb, this);

    static auto _gps_point_sub = ros_node->subscribe("/uav/gps_point", 1, &RosInterface::_gps_point_cb, this);
    static auto _gps_velocity_sub = ros_node->subscribe("/uav/velocity", 1, &RosInterface::_gps_velocity_cb, this);

    static auto _esc_feedback_sub = ros_node->subscribe("/uav/esc_status", 1, &RosInterface::_esc_feedback_cb, this);
    static auto _battery_sub = ros_node->subscribe("/uav/battery", 1, &RosInterface::_battery_cb, this);
    static auto _diff_pressure_sub = ros_node->subscribe("/uav/raw_air_data", 1, &RosInterface::_diff_pressure_cb, this);
    static auto _rangefinder_sub = ros_node->subscribe("/uav/range", 1, &RosInterface::_rangefinder_cb, this);


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

void RosInterface::send_arming_status(bool arming_status) {
    std_msgs::Bool ros_msg;
    ros_msg.data = arming_status;
    _arm_pub.publish(ros_msg);
}

void RosInterface::_imu_cb(const sensor_msgs::Imu& msg) {
    Vector3 accel{msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z};
    Vector3 gyro{msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
    for (auto callback : imu_callbacks) {
        callback(accel, gyro);
    }
}

void RosInterface::_mag_cb(const sensor_msgs::MagneticField& msg) {
    Vector3 magnetic_field_gauss{msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z};
    for (auto callback : magnetometer_callbacks) {
        callback(magnetic_field_gauss);
    }
}

void RosInterface::_baro_temp_cb(const std_msgs::Float32& msg) {
    (void)msg;
}

void RosInterface::_baro_pres_cb(const std_msgs::Float32& msg) {
    for (auto callback : baro_callbacks) {
        callback(msg.data, 312);
    }
}

void RosInterface::_gps_point_cb(const sensor_msgs::NavSatFix& msg) {
    Vector3 global_pose{msg.latitude, msg.longitude, msg.altitude};
    for (auto callback : gnss_callbacks) {
        callback(global_pose, _velocity);
    }
}

void RosInterface::_gps_velocity_cb(const geometry_msgs::Twist& msg) {
    _velocity[0] = msg.linear.x;
    _velocity[1] = msg.linear.y;
    _velocity[2] = msg.linear.z;
}

void RosInterface::_esc_feedback_cb(const mavros_msgs::ESCTelemetryItem& msg) {
    for (auto callback : esc_feedback_callbacks) {
        callback(msg.count, msg.voltage, msg.current, msg.rpm);
    }
}

void RosInterface::_battery_cb(const sensor_msgs::BatteryState& msg) {
    for (auto callback : battery_callbacks) {
        BatteryStatus battery;
        battery.voltage = msg.voltage;
        battery.current = msg.current;
        battery.soc = msg.percentage;
        battery.temperature_kelvin = msg.temperature;
        battery.full_capacity_ah = msg.capacity;
        battery.remaining_capacity_ah = msg.charge;
        callback(battery);
    }
}


void RosInterface::_diff_pressure_cb(const std_msgs::Float32& msg) {
    for (auto callback : diff_pressure_callbacks) {
        callback(msg.data);
    }
}

void RosInterface::_rangefinder_cb(const std_msgs::Float32& msg) {
    for (auto callback : rangefinder_callbacks) {
        callback(msg.data);
    }
}

bool RosInterface::spin_once() {
    ros::spinOnce();
    return true;
}
