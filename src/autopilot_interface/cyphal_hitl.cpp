// This software is distributed under the terms of the GPL v3 License.
// Copyright (c) 2023 Dmitry Ponomarev.
// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include "cyphal_hitl.hpp"
#include <algorithm>
#include <iostream>
#include <random>
#include <chrono>
#include "main.h"
#include "params.hpp"

#define GAUSS_TO_AMPERE_PER_METER 79.577471545947673925

uint32_t HAL_GetTick() {
    static auto time_start = std::chrono::steady_clock::now();
    auto time_now = std::chrono::steady_clock::now();
    auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_start).count();
    return elapsed_time_ms;
}

int CyphalHitlInterface::init() {
    romInit(0, 1);
    paramsInit(IntParamsIndexes::INTEGER_PARAMS_AMOUNT, NUM_OF_STR_PARAMS, -1, 1);
    paramsLoad();
    int init_res = cyphal.init();
    if (init_res < 0) {
        std::cout << "Cyphal Initialization Error: " << init_res << std::endl;
        return -1;
    }

    setpoint.init();
    readiness.init();
    rgbled.init();

    battery.set_nominal_data(5.0f, 42, 14.8f);

    return 0;
}

void CyphalHitlInterface::process() {
    _update_port_identifiers();
    cyphal.process();
    cyphal.process();
}

void CyphalHitlInterface::publish_barometer(float pressure, float temperature) {
    uavcan_si_sample_pressure_Scalar_1_0 pressure_msg;
    pressure_msg.pascal = pressure;
    baro_pressure.publish(pressure_msg);

    uavcan_si_sample_temperature_Scalar_1_0 temperature_msg;
    temperature_msg.kelvin = temperature;
    baro_temperature.publish(temperature_msg);
}

void CyphalHitlInterface::publish_imu(const Vector3 linear_accel, const Vector3 ang_vel) {
    // uavcan_si_unit_acceleration_Vector3_1_0 accel_msg;

    // accel_msg.meter_per_second_per_second[0] = linear_accel[0];
    // accel_msg.meter_per_second_per_second[1] = linear_accel[1];
    // accel_msg.meter_per_second_per_second[2] = linear_accel[2];
    // accel.publish(accel_msg);

    // uavcan_si_unit_angular_velocity_Vector3_1_0 gyro_msg;
    // gyro_msg.radian_per_second[0] = ang_vel[0] * _time_factor;
    // gyro_msg.radian_per_second[1] = ang_vel[1] * _time_factor;
    // gyro_msg.radian_per_second[2] = ang_vel[2] * _time_factor;
    // gyro.publish(gyro_msg);

    imu.publish(linear_accel, ang_vel);
}

void CyphalHitlInterface::publish_magnetometer(const Vector3 magnetic_field_gauss) {
    uavcan_si_sample_magnetic_field_strength_Vector3_1_1 magnetic_field;
    magnetic_field.ampere_per_meter[0] = GAUSS_TO_AMPERE_PER_METER * magnetic_field_gauss[0];
    magnetic_field.ampere_per_meter[1] = GAUSS_TO_AMPERE_PER_METER * magnetic_field_gauss[1];
    magnetic_field.ampere_per_meter[2] = GAUSS_TO_AMPERE_PER_METER * magnetic_field_gauss[2];
    magnetometer.publish(magnetic_field);
}

// Simulate GNSS timestamp by wrapping the time to align with a typical GNSS time reference (e.g., GPS epoch).
// GPS epoch started on January 6, 1980, so we calculate the time since then.
uint64_t simulate_gnss_utc_timestamp_usec() {
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    constexpr uint64_t gps_epoch_offset_usec = 315964800000000ULL;
    uint64_t gnss_utc_timestamp_usec = duration_since_epoch.count() - gps_epoch_offset_usec;
    return gnss_utc_timestamp_usec;
}

void CyphalHitlInterface::publish_gnss(const Vector3& global_pose, const Vector3& ned_velocity) {
    uint32_t crnt_time = HAL_GetTick() % 60000;
    float time_multiplier;
    if (crnt_time < 30000) {
        time_multiplier = 1.0 + crnt_time / 60000.0f;
    } else {
        time_multiplier = 2.0 - crnt_time / 60000.0f;
    }
    static float random_multiplier = 1.0;
    float random_offset = (std::rand() % 100 - 50) * 0.0002;
    random_multiplier = std::clamp(random_multiplier + random_offset, 0.85f, 1.15f);

    auto gnss_utc_timestamp_usec = simulate_gnss_utc_timestamp_usec();
    uint64_t MICROSECONDS_IN_WEEK = 604800000000;  // 7*24*60*60*1e6;
    gps_gnss.msg.time_week = static_cast<uint16_t>(gnss_utc_timestamp_usec / MICROSECONDS_IN_WEEK);
    gps_gnss.msg.time_week_ms = static_cast<uint32_t>((gnss_utc_timestamp_usec % MICROSECONDS_IN_WEEK) / 1000);

    gps_gnss.msg.point.latitude = global_pose[0] * RAD_TO_DEGREE;
    gps_gnss.msg.point.longitude = global_pose[1] * RAD_TO_DEGREE;
    gps_gnss.msg.point.altitude.meter = global_pose[2];
    gps_gnss.msg.velocity.meter_per_second[0] = ned_velocity[0] * _time_factor;
    gps_gnss.msg.velocity.meter_per_second[1] = ned_velocity[1] * _time_factor;
    gps_gnss.msg.velocity.meter_per_second[2] = ned_velocity[2] * _time_factor;
    gps_gnss.msg.num_sats = 10 * time_multiplier * random_multiplier;
    gps_gnss.msg.status.status = 3;
    gps_gnss.msg.hdop = 1.0f / time_multiplier / random_multiplier;
    gps_gnss.msg.vdop = 1.0f / time_multiplier / random_multiplier;
    gps_gnss.msg.horizontal_accuracy = 1.0f / time_multiplier / random_multiplier;
    gps_gnss.msg.vertical_accuracy = 1.0f / time_multiplier / random_multiplier;
    gps_gnss.publish();
}

void CyphalHitlInterface::publish_esc_feedback(uint8_t esc_idx, float voltage, float current, uint32_t rpm) {
    if (esc_idx >= esc_feedback.size()) {
        return;
    }

    cyphal::ZubaxCompactFeedback feedback;
    feedback.dc_voltage = std::round(voltage * 5.0f);
    feedback.dc_current = std::round(current * 5.0f);
    feedback.velocity = rpm * cyphal::ZubaxCompactFeedbackPublisher::RPM_TO_RAD_PER_SEC;

    esc_feedback[esc_idx]->publish(feedback);
}

void CyphalHitlInterface::publish_battery(float voltage, float current, float temperature_kelvin, float full_capacity_ah, float remaining_capacity_ah) {
    battery.publish(voltage, current, temperature_kelvin, full_capacity_ah, remaining_capacity_ah);
}

void CyphalHitlInterface::publish_diff_pressure(float pressure) {
    diff_pressure_0.publish(pressure);
    diff_pressure_1.publish(pressure);
}

void CyphalHitlInterface::publish_rangefinder(float range) {
    rangefinder.publish(range);
}

bool CyphalHitlInterface::get_setpoint(Setpoint16& out_setpoint) {
    static uint32_t prev_setpoint_recv_counter = 0;
    uint32_t crnt_setpoint_recv_counter = setpoint.get_recv_counter();
    if (crnt_setpoint_recv_counter == prev_setpoint_recv_counter) {
        return false;
    }
    prev_setpoint_recv_counter = crnt_setpoint_recv_counter;

    auto sp = setpoint.get_setpoint();
    for (size_t sp_idx = 0; sp_idx < setpoint.get_setpoint_size(); sp_idx++) {
        out_setpoint[sp_idx] = sp.value[sp_idx];
    }
    for (size_t sp_idx = setpoint.get_setpoint_size(); sp_idx < out_setpoint.size(); sp_idx++) {
        out_setpoint[sp_idx] = 0.0;
    }

    return true;
}

ArmingStatus CyphalHitlInterface::get_arming_status() {
    ArmingStatus arming_status;
    switch (readiness.get_readiness()) {
        case reg_udral_service_common_Readiness_0_1_ENGAGED:
            arming_status = ArmingStatus::ENGAGED;
            break;
        case reg_udral_service_common_Readiness_0_1_STANDBY:
            arming_status = ArmingStatus::STANDBY;
            break;

        default:
            arming_status = ArmingStatus::UNKNOWN;
            break;
    }

    return arming_status;
}


uint32_t CyphalHitlInterface::get_setpoint_recv_counter() {
    return setpoint.get_recv_counter();
}

void CyphalHitlInterface::clear_servo_pwm_counter() {
    setpoint.clear_recv_counter();
}

void CyphalHitlInterface::set_time_factor(double time_factor) {
    _time_factor = std::clamp(time_factor, 0.7, 1.0);
}

void CyphalHitlInterface::_update_port_identifiers() {
    esc_feedback_0.setPortId(paramsGetIntegerValue(PARAM_ESC_FEEDBACK_0_ID));
    esc_feedback_1.setPortId(paramsGetIntegerValue(PARAM_ESC_FEEDBACK_1_ID));
    esc_feedback_2.setPortId(paramsGetIntegerValue(PARAM_ESC_FEEDBACK_2_ID));
    esc_feedback_3.setPortId(paramsGetIntegerValue(PARAM_ESC_FEEDBACK_3_ID));

    gps_gnss.setPortId(paramsGetIntegerValue(PARAM_DS015_GPS_GNSS_ID));
    baro_pressure.setPortId(paramsGetIntegerValue(BAROMETER_PRESSURE_ID));
    baro_temperature.setPortId(paramsGetIntegerValue(BAROMETER_TEMPERATURE_ID));
    magnetometer.setPortId(paramsGetIntegerValue(PARAM_MAGNETOMETER_ID));
    accel.setPortId(paramsGetIntegerValue(IMU_ACCEL_ID));
    gyro.setPortId(paramsGetIntegerValue(IMU_GYRO_ID));
    imu.setPortId(paramsGetIntegerValue(PARAM_IMU_IMU_ID));
    diff_pressure_0.setPortId(paramsGetIntegerValue(PARAM_ASPD_DIFF_PRESSURE_0_ID));
    diff_pressure_1.setPortId(paramsGetIntegerValue(PARAM_ASPD_DIFF_PRESSURE_1_ID));
    rangefinder.setPortId(paramsGetIntegerValue(PARAM_RANGEFINDER_ID));

    battery.source_pub.setPortId(paramsGetIntegerValue(BMS_ENERGY_SOURCE_ID));
    battery.status_pub.setPortId(paramsGetIntegerValue(BMS_BATTERY_STATUS_ID));
    battery.parameters_pub.setPortId(paramsGetIntegerValue(BMS_BATTERY_PARAMETERS_ID));
}
