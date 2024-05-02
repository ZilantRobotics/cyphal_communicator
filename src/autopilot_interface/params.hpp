// This software is distributed under the terms of the GPL v3 License.
// Copyright (c) 2023 Dmitry Ponomarev.
// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#ifndef PARAMS_HPP_
#define PARAMS_HPP_
#include "storage.h"
enum IntParamsIndexes : ParamIndex_t {
    PARAM_NODE_ID,

    SETPOINT_ID,
    READINESS_ID,
    RGBLED_ID,

    PARAM_MAGNETOMETER_ID,

    BAROMETER_TEMPERATURE_ID,
    BAROMETER_PRESSURE_ID,

    PARAM_DS015_GPS_GNSS_ID,

    IMU_ACCEL_ID,
    IMU_GYRO_ID,
    PARAM_IMU_IMU_ID,

    PARAM_ESC_FEEDBACK_0_ID,
    PARAM_ESC_FEEDBACK_1_ID,
    PARAM_ESC_FEEDBACK_2_ID,
    PARAM_ESC_FEEDBACK_3_ID,

    PARAM_ASPD_DIFF_PRESSURE_0_ID,
    PARAM_ASPD_DIFF_PRESSURE_1_ID,

    BMS_ENERGY_SOURCE_ID,
    BMS_BATTERY_STATUS_ID,
    BMS_BATTERY_PARAMETERS_ID,

    PARAM_RANGEFINDER_ID,

    INTEGER_PARAMS_AMOUNT
};

#define NUM_OF_STR_PARAMS 21

#endif  // PARAMS_HPP_
