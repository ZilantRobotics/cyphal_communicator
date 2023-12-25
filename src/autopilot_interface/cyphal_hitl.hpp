/// For license information, see the LICENSE file in the root directory of this source tree 
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

enum class ArmingStatus {
    UNKNOWN,
    STANDBY,
    ENGAGED,
};

class CyphalHitlInterface {
public:
    CyphalHitlInterface() :
            setpoint(&cyphal),
            readiness(&cyphal),
            rgbled(&cyphal),
            esc_feedback_0(&cyphal, 0),
            esc_feedback_1(&cyphal, 0),
            esc_feedback_2(&cyphal, 0),
            esc_feedback_3(&cyphal, 0),
            esc_feedback{{&esc_feedback_0, &esc_feedback_1, &esc_feedback_2, &esc_feedback_3}},
            gps_point(&cyphal, 0),
            gps_sats(&cyphal, 0),
            gps_status(&cyphal, 0),
            gps_pdop(&cyphal, 0),
            baro_pressure(&cyphal, 0),
            baro_temperature(&cyphal, 0),
            magnetometer(&cyphal, 0),
            accel(&cyphal, 0),
            gyro(&cyphal, 0),
            imu(&cyphal, 0),
            diff_pressure_0(&cyphal, 0),
            diff_pressure_1(&cyphal, 0),
            rangefinder(&cyphal, 0),
            battery(&cyphal, 0, 0, 0) {}
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
    ArmingStatus get_arming_status();
    uint32_t get_setpoint_recv_counter();
    void clear_servo_pwm_counter();
    void set_time_factor(double time_factor);
private:
    void _update_port_identifiers();

    cyphal::Cyphal cyphal;

    cyphal::SetpointSubscriber setpoint;
    cyphal::ReadinessSubscriber readiness;
    cyphal::HighColorSubscriber rgbled;
    cyphal::ZubaxCompactFeedbackPublisher esc_feedback_0;
    cyphal::ZubaxCompactFeedbackPublisher esc_feedback_1;
    cyphal::ZubaxCompactFeedbackPublisher esc_feedback_2;
    cyphal::ZubaxCompactFeedbackPublisher esc_feedback_3;
    std::array<cyphal::ZubaxCompactFeedbackPublisher*, 4> esc_feedback;

    cyphal::GpsPointPublisher gps_point;
    cyphal::Int16Publisher gps_sats;
    cyphal::Int16Publisher gps_status;
    cyphal::Int16Publisher gps_pdop;

    cyphal::BaroPressurePublisher baro_pressure;
    cyphal::BaroTemperaturePublisher baro_temperature;

    cyphal::MagneticFieldPublisher magnetometer;

    cyphal::ImuAccelPublisher accel;
    cyphal::ImuGyroPublisher gyro;
    cyphal::RawImuPublisher imu;

    cyphal::DiffPressurePublisher diff_pressure_0;
    cyphal::DiffPressurePublisher diff_pressure_1;
    cyphal::RangefinderRangePublisher rangefinder;
    cyphal::UdralBatteryPublisher battery;

    double _time_factor{1.0};
};

#endif  // SRC_AUTOPILOT_INTERFACE_CYPHAL_HITL_HPP_
