/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cyphal_communicator");
    ros::NodeHandle ros_node;

    std::cout << "Hello, world" << std::endl;
}
