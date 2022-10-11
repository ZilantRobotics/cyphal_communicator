# Cyphal communicator

Cyphal communicator converts Cyphal messages to ROS and vice versa.

It covers a minimal set of sensors required for such applications as Ardupilot/PX4 Cyphal HITL simulation. This communicator can be used for other purposes as well.

## Content
  - [1. Conversions](#1-conversions)
  - [2. Preparation](#2-preparation)
  - [3. Running](#3-running)
  - [4. Usage example](#4-usage-example)

## 1. Conversions

The tables below represent the supported conversions:

**CYPHAL->ROS**

| № | ROS msg | ROS topic | Cyphal msg | Cyphal subject name |
| - | ------- | --------- | ---------- | ------------------- |
| 1 | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) | /uav/actuators_raw | [reg.udral.service.actuator.common.sp.Scalar_0_1](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/actuator/common/sp/Vector4.0.1.dsdl) |setpoint |
| 2 | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html) | /uav/arm | [reg.udral.service.common.Readiness_0_1](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/common/Readiness.0.1.dsdl) | readiness |

**ROS->CYPHAL**

1. IMU

| ROS sub msg | ROS sub topic | Cyphal pub msg | Cyphal pub subject name |
| ------- | --------- | ---------- | ------------------- |
| [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) | /uav/imu | [uavcan.si.sample.angular_velocity.Vector3.1.0](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/angular_velocity/Vector3.1.0.dsdl) | gyro |
|  | | [uavcan.si.sample.acceleration.Vector3.1.0](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/acceleration/Vector3.1.0.dsdl) | accel |

2. Compass

| ROS msg | ROS topic | Cyphal msg | Cyphal subject name |
| ------- | --------- | ---------- | ------------------- |
| [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html) | /uav/mag | [uavcan.si.sample.magnetic_field_strength.Vector3.1.0](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/magnetic_field_strength/Vector3.1.0.dsdl) | mag |

3. Barometer

| ROS msg | ROS topic | Cyphal msg | Cyphal subject name |
| ------- | --------- | ---------- | ------------------- |
| [std_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) | /uav/static_temperature | [uavcan.si.sample.temperature.Scalar.1.0](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/temperature/Scalar.1.0.dsdl) | baro_temperature |
| [std_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) | /uav/static_pressure | [uavcan.si.sample.pressure.Scalar.1.0](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/pressure/Scalar.1.0.dsdl) | baro_pressure |

4. Gps

| ROS msg | ROS topic | Cyphal msg | Cyphal subject name |
| ------- | --------- | ---------- | ------------------- |
| [std_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) | /uav/yaw | [uavcan.si.sample.angle.Scalar.1.0](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/angle/Scalar.1.0.dsdl) | gps_yaw |
| [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) | /uav/gps_position | [uavcan.primitive.scalar.Integer16](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/primitive/scalar/Integer16.1.0.dsdl) | gps_status |
|  |  | [reg.udral.physics.kinematics.geodetic.PointStateVarTs.0.1](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/physics/kinematics/geodetic/PointStateVarTs.0.1.dsdl) | gps_point |
| [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) | /uav/velocity | | |
| - | - | [uavcan.primitive.scalar.Integer16](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/primitive/scalar/Integer16.1.0.dsdl) | gps_sats |
| - | - | [uavcan.primitive.scalar.Integer16](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/primitive/scalar/Integer16.1.0.dsdl) | gps_pdop |


## 2. Preparation

Before running the communicator, you need to do 3 things:
1. Compile DSDL. You can use [compile_dsdl.sh](compile_dsdl.sh) as an example. You should compile it each time after updating DSDL.
2. Create virtual CAN. It is expected that you are using CAN-sniffer device such as [UAVCAN sniffer and programmer](https://github.com/InnopolisAero/inno_uavcan_node_binaries/blob/master/doc/programmer_sniffer/README.md). An example of script that creates SLCAN is [scripts/create_slcan_from_serial.sh](scripts/create_slcan_from_serial.sh). This script automatically detect a connected device and create `slcan0` port. You should create virtual CAN once after each sniffer connection to your PC.
3. Configure environment variables. This step is required for setting subjects port id and few pathes. As an example, you can run `source scripts/config.sh`. You should call this script in each shell session.

After these steps you are able to run the communicator.

## 3. Running

It is recommended to run the communicator using launch file as shown below:

```
roslaunch cyphal_communicator cyphal_communicator.launch
```

## 4. Usage example

Below you can see an example of using the cyphal_communicator in conjunction with a VTOL dynamics simulator.

[![vtol HITL dynamics simulator](https://img.youtube.com/vi/JmElAwgAoSc/0.jpg)](https://youtu.be/JmElAwgAoSc)
