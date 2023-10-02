/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#ifndef PARAMS_HPP_
#define PARAMS_HPP_
#include "storage.h"
enum IntParamsIndexes : ParamIndex_t {
    PARAM_NODE_ID,

    SETPOINT_ID,
    READINESS_ID,

    BAROMETER_PRESSURE_ID,
    BAROMETER_TEMPERATURE_ID,

    GPS_POINT_ID,
    GPS_SATS_ID,
    GPS_STATUS_ID,
    GPS_PDOP_ID,

    MAGNETOMETER_ID,

    IMU_ACCEL_ID,
    IMU_GYRO_ID,

    INTEGER_PARAMS_AMOUNT
};

#endif  // PARAMS_HPP_
