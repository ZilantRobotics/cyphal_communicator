/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include "params.hpp"
#include "string_params.hpp"
#include "storage.h"

IntegerDesc_t integer_desc_pool[] = {
    {(uint8_t*)"id", 2, 2, 2, false},

    {(uint8_t*)"uavcan.sub.setpoint.id",    2342, 2342, 2342, false},
    {(uint8_t*)"uavcan.sub.readiness.id",   2343, 2343, 2343, false},

    {(uint8_t*)"uavcan.pub.baro.press.id",  2404, 2404, 2404, false},
    {(uint8_t*)"uavcan.pub.baro.temp.id",   2403, 2403, 2403, false},

    {(uint8_t*)"uavcan.pub.gps.point.id",   2406, 2406, 2406, false},
    {(uint8_t*)"uavcan.pub.gps.sats.id",    2407, 2407, 2407, false},
    {(uint8_t*)"uavcan.pub.gps.status.id",  2408, 2408, 2408, false},
    {(uint8_t*)"uavcan.pub.gps.pdop.id",    2409, 2409, 2409, false},

    {(uint8_t*)"uavcan.pub.mag.id",         2402, 2402, 2402, false},

    {(uint8_t*)"uavcan.pub.imu.accel.id",   2400, 2400, 2400, false},
    {(uint8_t*)"uavcan.pub.imu.gyro.id",    2401, 2401, 2401, false},
};
IntegerParamValue_t integer_values_pool[sizeof(integer_desc_pool) / sizeof(IntegerDesc_t)];

StringDesc_t string_desc_pool[NUM_OF_STR_PARAMS] = {
    {(uint8_t*)"name", "", false},

    {(uint8_t*)"uavcan.sub.setpoint.type",      "reg.udral.service.actuator.common.sp.Vector4", true},
    {(uint8_t*)"uavcan.sub.readiness.type",     "reg.udral.service.common.Readiness", true},

    {(uint8_t*)"uavcan.pub.baro.press.type",    "uavcan.si.sample.pressure.Scalar", true},
    {(uint8_t*)"uavcan.pub.baro.temp.type",     "uavcan.si.sample.temperature.Scalar", true},

    {(uint8_t*)"uavcan.pub.gps.point.type",     "reg.udral.physics.kinematics.geodetic.PointStateVarTs", true},
    {(uint8_t*)"uavcan.pub.gps.sats.type",      "uavcan.primitive.scalar.Integer16", true},
    {(uint8_t*)"uavcan.pub.gps.status.type",    "uavcan.primitive.scalar.Integer16", true},
    {(uint8_t*)"uavcan.pub.gps.pdop.type",      "uavcan.primitive.scalar.Integer16", true},

    {(uint8_t*)"uavcan.pub.mag.type",           "uavcan.si.sample.magnetic_field_strength.Vector3", true},

    {(uint8_t*)"uavcan.pub.imu.accel.type",     "uavcan.si.sample.acceleration.Vector3", true},
    {(uint8_t*)"uavcan.pub.imu.gyro.type",      "uavcan.si.sample.angular_velocity.Vector3", true},
};
StringParamValue_t string_values_pool[sizeof(string_desc_pool) / sizeof(StringDesc_t)];
