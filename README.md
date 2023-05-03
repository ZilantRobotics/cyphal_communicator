# Cyphal communicator

Cyphal communicator converts Cyphal messages to ROS and vice versa.

It covers a minimal set of sensors required for such applications as Ardupilot/PX4 Cyphal HITL simulation. This communicator can be used for other purposes as well.

## Content
  - [1. Conversions](#1-conversions)
  - [2. Preparation](#2-preparation)
  - [3. Running](#3-running)
  - [4. Usage example](#4-usage-example)

## 1. Conversions

**CYPHAL->ROS**

```mermaid
flowchart LR

setpoint[ setpoint, reg.udral.service.actuator.common.sp.Vector8] --> F(SetpointCyphalToRos) --> actuators_raw[ /uav/actuators_raw, sensor_msgs/Joy]
readiness[ readiness, reg.udral.service.common.Readiness] --> R(ReadinessCyphalToRos) --> arm[ /uav/arm, std_msgs::Bool]
```

**ROS->CYPHAL**

```mermaid
flowchart LR

imu[ /uav/imu, sensor_msgs/Imu] --> F(ImuRosToCyphal)
F(ImuRosToCyphal) --> gyro[ gyro, uavcan.si.sample.angular_velocity.Vector3]
F(ImuRosToCyphal) --> accel[ accel, uavcan.si.sample.acceleration.Vector3]
```

```mermaid
flowchart LR

imu[ /uav/mag, sensor_msgs/MagneticField] --> F(MagRosToCyphal) --> mag[ mag, uavcan.si.sample.magnetic_field_strength.Vector3]
```

```mermaid
flowchart LR

static_temperature[ /uav/static_temperature, std_msgs/Float32] --> BaroRosToCyphal(BaroRosToCyphal) --> baro_temperature[ baro_temperature, uavcan.si.sample.temperature.Scalar]

static_pressure[ /uav/static_pressure, std_msgs/Float32] --> BaroRosToCyphal --> baro_pressure[ baro_pressure, uavcan.si.sample.pressure.Scalar]
```

```mermaid
flowchart LR
yaw[ /uav/yaw, std_msgs/Float32] --> GpsRosToCyphal(GpsRosToCyphal) --> gps_yaw[ gps_yaw, uavcan.si.sample.angle.Scalar]

point[ /uav/gps_point, sensor_msgs/NavSatFix] --> GpsRosToCyphal
GpsRosToCyphal --> gps_status[ gps_status, uavcan.primitive.scalar.Integer16]
GpsRosToCyphal --> gps_point[ gps_point, reg.udral.physics.kinematics.geodetic.PointStateVarTs]

velocity[ /uav/velocity, geometry_msgs/Twist] --> GpsRosToCyphal --> gps_point[ gps_point, reg.udral.physics.kinematics.geodetic.PointStateVarTs]

GpsRosToCyphal --> gps_sats[ gps_sats, uavcan.primitive.scalar.Integer16]
GpsRosToCyphal --> gps_pdop[ gps_pdop, uavcan.primitive.scalar.Integer16]
```

```mermaid
flowchart LR
esc_status[ /uav/esc_status, mavros_msgs/ESCTelemetryItem] --> EscStatusRosToCyphal(EscStatusRosToCyphal)
EscStatusRosToCyphal --> esc_feedback_0[ esc_feedback_0, zubax.telega.CompactFeedback]
EscStatusRosToCyphal --> esc_feedback_1[ esc_feedback_1, zubax.telega.CompactFeedback]
EscStatusRosToCyphal --> esc_feedback_n[ ...]
EscStatusRosToCyphal --> esc_feedback_7[ esc_feedback_7, zubax.telega.CompactFeedback]
```

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
