# Cyphal communicator

Cyphal communicator is a bridge between Cyphal and ROS.

It covers a minimal set of sensors required for such applications as PX4 HITL simulation. This communicator can be used for other purposes as well.

## Content
  - [1. Conversions](#1-conversions)
  - [2. Preparation](#2-preparation)
  - [3. Running](#3-running)
  - [4. Usage example](#4-usage-example)

## 1. Conversions

The tables below represent the supported conversions:

**CYPHAL->ROS**

| № | ROS msg | Cyphal msg | ROS topic | Cyphal subject name |
| - | ------- | ---------- | --------- | ------------------- |
| 1 | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) | [reg.udral.service.actuator.common.sp.Scalar_0_1](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/actuator/common/sp/Vector4.0.1.dsdl) | /uav/actuators | setpoint |
| 2 | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)             | [reg.udral.service.common.Readiness_0_1](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/common/Readiness.0.1.dsdl) | /uav/arm | readiness |

## 2. Preparation

It is expected that you are using CAN-sniffer device such as [UAVCAN sniffer and programmer](https://github.com/InnopolisAero/inno_uavcan_node_binaries/blob/master/doc/programmer_sniffer/README.md). You need to connect it to your PC.

After connection, you should create SLCAN and configure environment variables using following scripts:

```bash
./create_slcan_from_serial.sh
source config.sh
```

## 3. Running

1. At first, you need to create a virtual can port
2. Then specify in `config/params.yaml` which conversions do you need to use
3. Then launch communicator typing:

Example:
```
roslaunch cyphal_communicator cyphal_communicator.launch
```

## 4. Usage example

Below you can see an example of using the cyphal_communicator in conjunction with a VTOL dynamics simulator.

[![vtol HITL dynamics simulator](https://img.youtube.com/vi/JmElAwgAoSc/0.jpg)](https://youtu.be/JmElAwgAoSc)
