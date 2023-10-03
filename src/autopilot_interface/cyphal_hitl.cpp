/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022-2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include "cyphal_hitl.hpp"
#include <algorithm>
#include <iostream>
#include <chrono>
#include "main.h"
#include "params.hpp"
#include "string_params.hpp"

uint32_t HAL_GetTick() {
    static auto time_start = std::chrono::steady_clock::now();
    auto time_now = std::chrono::steady_clock::now();
    auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_start).count();
    return elapsed_time_ms;
}

int CyphalHitlInterface::init() {
    romInit(0, 1);
    paramsInit(IntParamsIndexes::INTEGER_PARAMS_AMOUNT, NUM_OF_STR_PARAMS);
    paramsLoadFromFlash();
    int init_res = cyphal.init();
    if (init_res < 0) {
        std::cout << "Cyphal Initialization Error: " << init_res << std::endl;
        return -1;
    }

    setpoint.init();
    readiness.init();

    return 0;
}

void CyphalHitlInterface::process() {
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
    uavcan_si_unit_acceleration_Vector3_1_0 accel_msg;

    accel_msg.meter_per_second_per_second[0] = linear_accel[0];
    accel_msg.meter_per_second_per_second[1] = linear_accel[1];
    accel_msg.meter_per_second_per_second[2] = linear_accel[2];
    accel.publish(accel_msg);

    uavcan_si_unit_angular_velocity_Vector3_1_0 gyro_msg;
    gyro_msg.radian_per_second[0] = ang_vel[0] * _time_factor;
    gyro_msg.radian_per_second[1] = ang_vel[1] * _time_factor;
    gyro_msg.radian_per_second[2] = ang_vel[2] * _time_factor;
    gyro.publish(gyro_msg);
}

void CyphalHitlInterface::publish_magnetometer(const Vector3 magnetic_field_gauss) {
    uavcan_si_sample_magnetic_field_strength_Vector3_1_0 magnetic_field;
    magnetic_field.tesla[0] = 1e-04 * magnetic_field_gauss[0];
    magnetic_field.tesla[1] = 1e-04 * magnetic_field_gauss[1];
    magnetic_field.tesla[2] = 1e-04 * magnetic_field_gauss[2];
    magnetometer.publish(magnetic_field);
}

void CyphalHitlInterface::publish_gnss(const Vector3& global_pose, const Vector3& ned_velocity) {
    reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1 gps_point_msg;
    gps_point_msg.value.position.value.latitude =  global_pose[0] * RAD_TO_DEGREE;
    gps_point_msg.value.position.value.longitude = global_pose[1] * RAD_TO_DEGREE;
    gps_point_msg.value.position.value.altitude.meter = global_pose[2];

    gps_point_msg.value.velocity.value.meter_per_second[0] = ned_velocity[0] * _time_factor;
    gps_point_msg.value.velocity.value.meter_per_second[1] = ned_velocity[1] * _time_factor;
    gps_point_msg.value.velocity.value.meter_per_second[2] = ned_velocity[2] * _time_factor;
    gps_point.publish(gps_point_msg);

    gps_sats.msg.value = 10;
    gps_sats.publish();

    gps_status.msg.value = 3;
    gps_status.publish();

    gps_pdop.msg.value = 1;
    gps_pdop.publish();
}

bool CyphalHitlInterface::get_setpoint(Setpoint16& out_setpoint) {
    static uint32_t prev_setpoint_recv_counter = 0;
    uint32_t crnt_setpoint_recv_counter = setpoint.get_recv_counter();
    if (crnt_setpoint_recv_counter == prev_setpoint_recv_counter) {
        return false;
    }
    prev_setpoint_recv_counter = crnt_setpoint_recv_counter;

    auto sp = setpoint.get_setpoint();
    for (size_t sp_idx = 0; sp_idx < 4; sp_idx++) {
        out_setpoint[sp_idx] = sp.value[sp_idx];
    }
    return true;
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