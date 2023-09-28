# Cyphal communicator

Cyphal communicator converts Cyphal messages to ROS and vice versa.

It covers a minimal set of sensors required for such applications as Ardupilot/PX4 Cyphal HITL simulation. This communicator can be used for other purposes as well.

For a quadrotor application we should have:
- actuators (ESC),
- gnss,
- barometer,
- magnetometer,
- imu.

For a VTOL application we should additionally have:
- differential pressure sensor or airspeed,

It would be nice to also have:
- rangefinder,
- BMS,
- ESC feedback.

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

> On PX4 side actuators logic is implemented within [UavcanEscController and ReadinessPublisher drivers](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Actuators/EscClient.hpp).

**ROS->CYPHAL**

**1. IMU**

```mermaid
flowchart LR

imu[ /uav/imu, sensor_msgs/Imu] --> F(ImuRosToCyphal)
F(ImuRosToCyphal) --> gyro[ gyro, uavcan.si.sample.angular_velocity.Vector3]
F(ImuRosToCyphal) --> accel[ accel, uavcan.si.sample.acceleration.Vector3]
```

> On PX4 side IMU logic is implemented within [UavcanAccelerometerSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Accelerometer.hpp) and [UavcanGyroscopeSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Gyroscope.hpp) drivers.

**2. Magnetometer**

```mermaid
flowchart LR

imu[ /uav/mag, sensor_msgs/MagneticField] --> F(MagRosToCyphal) --> mag[ mag, uavcan.si.sample.magnetic_field_strength.Vector3]
```

> On PX4 side IMU logic is implemented within [UavcanMagnetometerSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Magnetometer.hpp) driver.

**3. Barometer**

```mermaid
flowchart LR

static_temperature[ /uav/static_temperature, std_msgs/Float32] --> BaroRosToCyphal(BaroRosToCyphal) --> baro_temperature[ baro_temperature, uavcan.si.sample.temperature.Scalar]

static_pressure[ /uav/static_pressure, std_msgs/Float32] --> BaroRosToCyphal --> baro_pressure[ baro_pressure, uavcan.si.sample.pressure.Scalar]
```

> On PX4 side IMU logic is implemented within [UavcanBarometerSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Barometer.hpp) driver.

**4. GNSS**

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

> On PX4 side GNSS logic is implemented within [UavcanGnssSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Gnss.hpp) driver.

**5. ESC Feedback**

```mermaid
flowchart LR
esc_status[ /uav/esc_status, mavros_msgs/ESCTelemetryItem] --> EscStatusRosToCyphal(EscStatusRosToCyphal)
EscStatusRosToCyphal --> esc_feedback_0[ esc_feedback_0, zubax.telega.CompactFeedback]
EscStatusRosToCyphal --> esc_feedback_1[ esc_feedback_1, zubax.telega.CompactFeedback]
EscStatusRosToCyphal --> esc_feedback_n[ ...]
EscStatusRosToCyphal --> esc_feedback_7[ esc_feedback_7, zubax.telega.CompactFeedback]
```

> On PX4 side GNSS logic is implemented within [UavcanEscFeedbackSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Actuators/EscClient.hpp) driver.


## 2. Preparation

Before running the communicator, you need to do 3 things:
1. Compile DSDL. You can use [compile_dsdl.sh](compile_dsdl.sh) as an example. You should compile it each time after updating DSDL.
2. Create virtual CAN. It is expected that you are using CAN-sniffer device such as [UAVCAN sniffer and programmer](https://github.com/InnopolisAero/inno_uavcan_node_binaries/blob/master/doc/programmer_sniffer/README.md). An example of script that creates SLCAN is [scripts/create_slcan.sh](scripts/create_slcan.sh). This script automatically detect a connected device and create `slcan0` port. You should create virtual CAN once after each sniffer connection to your PC.
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
