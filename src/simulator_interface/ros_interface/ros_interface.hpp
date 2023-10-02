/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#ifndef SIMULATOR_INTERFACE_ROS_INTERFACE_
#define SIMULATOR_INTERFACE_ROS_INTERFACE_

#include <stdint.h>
#include <cstddef>
#include <arpa/inet.h>
#include <array>
#include <vector>
#include <string>
#include "simulator_interface/simulator_interface.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/ESCTelemetryItem.h>
#include <mavros_msgs/ESCStatusItem.h>


class RosInterface : public SimulatorBaseInterface{
public:
    RosInterface(int argc, char** argv);
    bool init() override;
    bool send_setpoint(const Setpoint16& setpoint) override;
    bool receive_sensors() override;
private:
    void _imu_cb(const sensor_msgs::Imu& msg);
    void _mag_cb(const sensor_msgs::MagneticField& msg);
    void _baro_temp_cb(const std_msgs::Float32& msg);
    void _baro_pres_cb(const std_msgs::Float32& msg);
    void _gps_point_cb(const sensor_msgs::NavSatFix& msg);
    void _gps_velocity_cb(const geometry_msgs::Twist& msg);

    ros::NodeHandle* ros_node;
    ros::Publisher _setpoint_pub;

    Vector3 _velocity;
};

#endif  // SIMULATOR_INTERFACE_ROS_INTERFACE_
