# Cyphal communicator

Cyphal communicator is a ROS package that connects the Cyphal HITL autopilot interface and a simulator interface (based on ROS, ArduPilot JSON, etc). It is primailry intended for ArduPilot/PX4 Cyphal HITL simulation, but it can be used for other purposes as well.

## Content

  - [1. Interface](#1-interface)
    - [1.1. Minimal quadcopter interface](#11-minimal-quadcopter-interface)
    - [1.2. Extended interface](#12-extended-interface)
  - [2. Installation](#2-installation)
  - [3. Running](#3-running)
  - [4. Usage example](#4-usage-example)

## 1. Interface

### 1.1. Minimal quadcopter interface

The following Cyphal interface is supported for a minimal quadrotor application:

**1. Actuator**

Cyphal interface:

| Interface | Port | Message |
| --------- | ---- | ------- |
| [udral/actuator](https://github.com/OpenCyphal/public_regulated_data_types/tree/master/reg/udral/service/actuator) | sub.setpoint </br> sub.readiness </br> pub.feedback | [udral.service.actuator.common.sp.Vector31](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/actuator/common/sp/Vector31.0.1.dsdl) </br> [reg.udral.service.common.Readiness](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/common/Readiness.0.1.dsdl) </br> [zubax.telega.CompactFeedback](https://github.com/Zubax/zubax_dsdl/blob/master/zubax/telega/CompactFeedback.1.0.dsdl) |

ROS-interface for necessary setpoint and readiness:

```mermaid
flowchart LR

setpoint[ setpoint, reg.udral.service.actuator.common.sp.Vector8] --> F(SetpointCyphalToRos) --> actuators_raw[ /uav/actuators_raw, sensor_msgs/Joy]
readiness[ readiness, reg.udral.service.common.Readiness] --> R(ReadinessCyphalToRos) --> arm[ /uav/arm, std_msgs::Bool]
```

> On PX4 side actuators logic is implemented within [UavcanEscController and ReadinessPublisher drivers](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Actuators/EscClient.hpp).

ROS-interface for auxilliary ESC feedback:

```mermaid
flowchart LR
esc_status[ /uav/esc_status, mavros_msgs/ESCTelemetryItem] --> EscStatusRosToCyphal(EscStatusRosToCyphal)
EscStatusRosToCyphal --> esc_feedback_0[ esc_feedback_0, zubax.telega.CompactFeedback]
EscStatusRosToCyphal --> esc_feedback_1[ esc_feedback_1, zubax.telega.CompactFeedback]
EscStatusRosToCyphal --> esc_feedback_n[ ...]
EscStatusRosToCyphal --> esc_feedback_7[ esc_feedback_7, zubax.telega.CompactFeedback]
```

> On PX4 side ESC Feedback logic is implemented within [UavcanEscFeedbackSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Actuators/EscClient.hpp) driver.

**2. IMU**

Cyphal interface:

| Interface | Port | Message |
| --------- | ---- | ------- |
| imu | pub.accelerometer </br> pub.gyro | [uavcan.si.sample.acceleration.Vector3](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/acceleration/Vector3.1.0.dsdl) </br> [uavcan.si.sample.angular_velocity.Vector3](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/angular_velocity/Vector3.1.0.dsdl) |

ROS-interface:

```mermaid
flowchart LR

imu[ /uav/imu, sensor_msgs/Imu] --> F(ImuRosToCyphal)
F(ImuRosToCyphal) --> gyro[ gyro, uavcan.si.sample.angular_velocity.Vector3]
F(ImuRosToCyphal) --> accel[ accel, uavcan.si.sample.acceleration.Vector3]
```

> On PX4 side IMU logic is implemented within [UavcanAccelerometerSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Accelerometer.hpp) and [UavcanGyroscopeSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Gyroscope.hpp) drivers.

**3. Magnetometer**

Cyphal interface:

| Interface | Port | Message |
| --------- | ---- | ------- |
| Magnetometer | pub.mag | [uavcan.si.sample.magnetic_field_strength.Vector3](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/magnetic_field_strength/Vector3.1.0.dsdl) |

ROS-interface:

```mermaid
flowchart LR

imu[ /uav/mag, sensor_msgs/MagneticField] --> F(MagRosToCyphal) --> mag[ mag, uavcan.si.sample.magnetic_field_strength.Vector3]
```

> On PX4 side IMU logic is implemented within [UavcanMagnetometerSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Magnetometer.hpp) driver.

**4. Barometer**

Cyphal interface:

| Interface | Port | Message |
| --------- | ---- | ------- |
| Barometer | pub.pressure </br> pub.temperature | [uavcan.si.sample.pressure.Scalar](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/pressure/Scalar.1.0.dsdl) </br> [uavcan.si.sample.temperature.Scalar](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/temperature/Scalar.1.0.dsdl) |

ROS-interface:

```mermaid
flowchart LR

static_temperature[ /uav/static_temperature, std_msgs/Float32] --> BaroRosToCyphal(BaroRosToCyphal) --> baro_temperature[ baro_temperature, uavcan.si.sample.temperature.Scalar]

static_pressure[ /uav/static_pressure, std_msgs/Float32] --> BaroRosToCyphal --> baro_pressure[ baro_pressure, uavcan.si.sample.pressure.Scalar]
```

> On PX4 side IMU logic is implemented within [UavcanBarometerSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Barometer.hpp) driver.

**5. GNSS**

Cyphal interface:

| Interface | Port | Message |
| --------- | ---- | ------- |
| [udral/gnss](https://nunaweb.opencyphal.org/api/storage/docs/docs/reg/index.html#reg_drone_service_gnss) | pub.point </br> pub.status </br> pub.sats </br> pub.pdop | [reg.udral.physics.kinematics.geodetic.PointStateVarTs](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/physics/kinematics/geodetic/PointStateVarTs.0.1.dsdl) </br> [uavcan.primitive.scalar.Integer16](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/primitive/scalar/Integer16.1.0.dsdl) </br> [uavcan.primitive.scalar.Integer16](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/primitive/scalar/Integer16.1.0.dsdl) </br> [uavcan.primitive.scalar.Integer16](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/primitive/scalar/Integer16.1.0.dsdl) |

ROS-interface:

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

### 1.2. Extended interface

The following Cyphal allows to run VTOL application and add a few auxilliary features:

**6. RGB LED**

Cyphal interface:

| Interface | Port | Message |
| --------- | ---- | ------- |
| Udral/rgbled | sub.rgbled | [reg.udral.physics.optics.HighColor](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/physics/optics/HighColor.0.1.dsdl) |

ROS-interface:

> not supported yet

On PX4 side RGB LED logic is implemented within [RGBControllerPublisher](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Publishers/udral/RGBController.hpp) driver.


**7. Airspeed (including Differential Pressure)**

Cyphal interface:

| Interface | Port | Message |
| --------- | ---- | ------- |
| [udral/airspeed](https://nunaweb.opencyphal.org/api/storage/docs/docs/reg/index.html#reg_drone_service_air_data_computer) | pub.aspd.cas </br> pub.aspd.tas </br> pub.aspd.dpres </br> pub.aspd.temperature | [uavcan.si.sample.velocity.Scalar](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/velocity/Scalar.1.0.dsdl) </br> [uavcan.si.sample.velocity.Scalar](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/velocity/Scalar.1.0.dsdl) </br> [uavcan.si.sample.pressure.Scalar](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/pressure/Scalar.1.0.dsdl) </br> [uavcan.si.sample.temperature.Scalar](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/temperature/Scalar.1.0.dsdl) |

ROS-interface for Airspeed:

```mermaid
flowchart LR
tas[ /uav/airspeed/tas, std_msgs/Float32] --> AirspeedRosToCyphal(AirspeedRosToCyphal)
cas[ /uav/airspeed/cas, std_msgs/Float32] --> AirspeedRosToCyphal(AirspeedRosToCyphal)
cas[ /uav/static_temperature, std_msgs/Float32] --> AirspeedRosToCyphal(AirspeedRosToCyphal)
AirspeedRosToCyphal --> aspd_tas[ aspd.tas, uavcan.si.sample.velocity.Scalar]
AirspeedRosToCyphal --> aspd_cas[ aspd.cas, uavcan.si.sample.velocity.Scalar]
AirspeedRosToCyphal --> aspd_temp[ aspd.temp, uavcan.si.sample.temperature.Scalar]
```

> On PX4 side Airspeed logic is implemented within [UavcanDiffPressureSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/DifferentialPressure.hpp) driver.

ROS-interface for Differential Pressure:

```mermaid
flowchart LR
dpres[ /uav/raw_air_data, std_msgs/Float32] --> DiffPressureRosToCyphal(DiffPressureRosToCyphal)
DiffPressureRosToCyphal --> aspd_dpres[ aspd.dpres, uavcan.si.sample.pressure.Scalar]
```

> On PX4 side Differential Pressure logic is implemented within [UavcanDiffPressureSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/DifferentialPressure.hpp) driver.

**8. Battery**

Cyphal interface:

| Interface | Port | Message | Rate |
| --------- | ---- | ------- | ---- |
| [Udral/Battery](https://nunaweb.opencyphal.org/api/storage/docs/docs/reg/index.html#reg_drone_service_battery) | pub.energy_source </br> pub.battery_status </br> pub.battery_parameters | [reg.udral.physics.electricity.SourceTs](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/physics/electricity/SourceTs.0.1.dsdl) </br> [reg.udral.service.battery.Status](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/battery/Status.0.2.dsdl) </br> [reg.udral.service.battery.Parameters](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/battery/Parameters.0.3.dsdl) | 1...100 </br> ~1 </br> ~0.2 |

ROS-interface:

```mermaid
flowchart LR
battery[ /uav/battery, sensor_msgs/BatteryState] --> BatteryRosToCyphal(BatteryRosToCyphal)
BatteryRosToCyphal --> energy_source[ energy_source, reg.udral.physics.electricity.SourceTs]
BatteryRosToCyphal --> battery_status[ battery_status, reg.udral.service.battery.Status]
BatteryRosToCyphal --> battery_parameters[ battery_parameters, reg.udral.service.battery.Parameters]
```

> On PX4 side Battery logic is implemented within [UavcanBmsSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Battery.hpp) driver.

**9. Rangefinder**

Cyphal interface:

| Interface | Port | Message | Rate |
| --------- | ---- | ------- | ---- |
| Rangefinder | pub.range | [uavcan.si.sample.length.Scalar](https://github.com/OpenCyphal/public_regulated_data_types/blob/master/uavcan/si/sample/length/Scalar.1.0.dsdl) | |

ROS-interface:

```mermaid
flowchart LR
tas[ /uav/height, std_msgs/Float32] --> RangefinderRosToCyphal(RangefinderRosToCyphal)
RangefinderRosToCyphal --> aspd_tas[ aspd.tas, uavcan.si.sample.length.Scalar]
```

> On PX4 side Rangefinder logic is implemented within [RangefinderSubscriber](https://github.com/ZilantRobotics/PX4-Autopilot/blob/cyphal-hitl/src/drivers/cyphal/Subscribers/udral/Rangefinder.hpp) driver.


## 2. Installation

Build the package as usual ROS package with `catkin build`. It will automatically compile DSDL at the build time with [compile_dsdl.sh](compile_dsdl.sh) script.

Before running the communicator, you need to do 3 things:
1. Create virtual CAN. It is expected that you are using CAN-sniffer device such as [UAVCAN sniffer and programmer](https://docs.raccoonlab.co/guide/programmer_sniffer/). An example of script that creates SLCAN is [scripts/create_slcan.sh](scripts/create_slcan.sh). This script automatically detect a connected device and create `slcan0` port. You should create virtual CAN once after each sniffer connection to your PC.
2. Configure environment variables. This step is required for setting subjects port id and few pathes. As an example, you can run `source scripts/config.sh`. You should call this script in each shell session.

After these steps you are able to run the communicator.

## 3. Running

It is recommended to run the communicator using launch file as shown below:

```
roslaunch cyphal_communicator cyphal_communicator.launch
```

## 4. Usage example

Below you can see an example of using the cyphal_communicator in conjunction with a VTOL dynamics simulator.

[![vtol HITL dynamics simulator](https://img.youtube.com/vi/JmElAwgAoSc/0.jpg)](https://youtu.be/JmElAwgAoSc)
