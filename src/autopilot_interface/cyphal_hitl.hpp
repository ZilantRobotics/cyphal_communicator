/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022-2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#ifndef SRC_AUTOPILOT_INTERFACE_CYPHAL_HITL_HPP_
#define SRC_AUTOPILOT_INTERFACE_CYPHAL_HITL_HPP_

#include <array>
#include "cyphal.hpp"

#include "actuator.hpp"
#include "airspeed.hpp"
#include "battery.hpp"
#include "barometer.hpp"
#include "gnss.hpp"
#include "imu.hpp"
#include "magnetometer.hpp"
#include "rangefinder.hpp"
#include "rgbled.hpp"

#include "math.hpp"
#include "simulator_interface/simulator_interface.hpp"
#include "params.hpp"

class CyphalHitlInterface {
public:
    CyphalHitlInterface() : setpoint(&cyphal),
                            readiness(&cyphal),
                            rgbled(&cyphal),
                            esc_feedback_0(&cyphal,     2500),
                            esc_feedback_1(&cyphal,     2501),
                            esc_feedback_2(&cyphal,     2502),
                            esc_feedback_3(&cyphal,     2503),
                            esc_feedback{{&esc_feedback_0, &esc_feedback_1, &esc_feedback_2, &esc_feedback_3}},
                            gps_point(&cyphal,          2406),
                            gps_sats(&cyphal,           2407),
                            gps_status(&cyphal,         2408),
                            gps_pdop(&cyphal,           2409),
                            baro_pressure(&cyphal,      2404),
                            baro_temperature(&cyphal,   2403),
                            magnetometer(&cyphal,       2402),
                            accel(&cyphal,              2400),
                            gyro(&cyphal,               2401),
                            imu(&cyphal,                2300),
                            diff_pressure(&cyphal,      2600),
                            rangefinder(&cyphal,        2800),
                            battery(&cyphal,            2700, 2701, 2702) {}
    int init();
    void process();

    void publish_barometer(float pressure, float temperature);
    void publish_battery(float voltage, float current, float temperature_kelvin, float full_capacity_ah, float remaining_capacity_ah);
    void publish_diff_pressure(float pressure);
    void publish_esc_feedback(uint8_t esc_idx, float voltage, float current, uint32_t rpm);
    void publish_imu(const Vector3 linear_accel, const Vector3 ang_vel);
    void publish_gnss(const Vector3& global_pose, const Vector3& ned_velocity);
    void publish_magnetometer(const Vector3 magnetic_field_gauss);
    void publish_rangefinder(float range);

    bool get_setpoint(Setpoint16& setpoint);
    bool get_arming_status();
    uint32_t get_setpoint_recv_counter();
    void clear_servo_pwm_counter();
    void set_time_factor(double time_factor);
private:
    Cyphal cyphal;

    SetpointSubscriber setpoint;
    ReadinessSubscriber readiness;
    HighColorSubscriber rgbled;
    ZubaxCompactFeedbackPublisher esc_feedback_0;
    ZubaxCompactFeedbackPublisher esc_feedback_1;
    ZubaxCompactFeedbackPublisher esc_feedback_2;
    ZubaxCompactFeedbackPublisher esc_feedback_3;
    std::array<ZubaxCompactFeedbackPublisher*, 4> esc_feedback;

    GpsPointPublisher gps_point;
    Int16Publisher gps_sats;
    Int16Publisher gps_status;
    Int16Publisher gps_pdop;

    BaroPressurePublisher baro_pressure;
    BaroTemperaturePublisher baro_temperature;

    MagneticFieldPublisher magnetometer;

    ImuAccelPublisher accel;
    ImuGyroPublisher gyro;
    RawImuPublisher imu;

    DiffPressurePublisher diff_pressure;
    RangefinderRangePublisher rangefinder;
    UdralBatteryPublisher battery;

    double _time_factor{1.0};
};

#endif  // SRC_AUTOPILOT_INTERFACE_CYPHAL_HITL_HPP_
