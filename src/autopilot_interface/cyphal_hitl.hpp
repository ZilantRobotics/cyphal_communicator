/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022-2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#ifndef SRC_AUTOPILOT_INTERFACE_CYPHAL_HITL_HPP_
#define SRC_AUTOPILOT_INTERFACE_CYPHAL_HITL_HPP_

#include "cyphal.hpp"
#include "actuator.hpp"
#include "gnss.hpp"
#include "barometer.hpp"
#include "magnetometer.hpp"
#include "imu.hpp"
#include "math.hpp"
#include "simulator_interface/simulator_interface.hpp"

class CyphalHitlInterface {
public:
    CyphalHitlInterface() : setpoint(&cyphal),
                            readiness(&cyphal),
                            gps_point(&cyphal,          2406),
                            gps_sats(&cyphal,           2407),
                            gps_status(&cyphal,         2408),
                            gps_pdop(&cyphal,           2409),
                            baro_pressure(&cyphal,      2404),
                            baro_temperature(&cyphal,   2403),
                            magnetometer(&cyphal,       2402),
                            accel(&cyphal,              2400),
                            gyro(&cyphal,               2401) {}
    int init();
    void process();

    void publish_barometer(float pressure, float temperature);
    void publish_imu(const Vector3 linear_accel, const Vector3 ang_vel);
    void publish_magnetometer(const Vector3 magnetic_field_gauss);
    void publish_gnss(const Vector3& global_pose, const Vector3& ned_velocity);

    bool get_setpoint(Setpoint16& setpoint);
    bool get_arming_status();
    uint32_t get_setpoint_recv_counter();
    void clear_servo_pwm_counter();
    void set_time_factor(double time_factor);
private:
    Cyphal cyphal;

    SetpointSubscriber setpoint;
    ReadinessSubscriber readiness;

    GpsPointPublisher gps_point;
    Int16Publisher gps_sats;
    Int16Publisher gps_status;
    Int16Publisher gps_pdop;

    BaroPressurePublisher baro_pressure;
    BaroTemperaturePublisher baro_temperature;

    MagneticFieldPublisher magnetometer;

    ImuAccelPublisher accel;
    ImuGyroPublisher gyro;

    double _time_factor{1.0};
};

#endif  // SRC_AUTOPILOT_INTERFACE_CYPHAL_HITL_HPP_
