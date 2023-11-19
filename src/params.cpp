/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include "params.hpp"
#include "storage.h"

IntegerDesc_t integer_desc_pool[] = {
    {"id", 2, 2, 2, MUTABLE},

    {"uavcan.sub.setpoint.id",              1, 65535, 2342, MUTABLE},
    {"uavcan.sub.readiness.id",             1, 65535, 2343, MUTABLE},
    {"uavcan.sub.rgbled.id",                1, 65535, 2344, MUTABLE},

    {"uavcan.pub.mag.id",                   1, 65535, 2402, MUTABLE},

    {"uavcan.pub.baro.temp.id",             1, 65535, 2403, MUTABLE},
    {"uavcan.pub.baro.press.id",            1, 65535, 2404, MUTABLE},

    {"uavcan.pub.gps.point.id",             1, 65535, 2406, MUTABLE},
    {"uavcan.pub.gps.sats.id",              1, 65535, 2407, MUTABLE},
    {"uavcan.pub.gps.status.id",            1, 65535, 2408, MUTABLE},
    {"uavcan.pub.gps.pdop.id",              1, 65535, 2409, MUTABLE},

    {"uavcan.pub.imu.accel.id",             1, 65535, 2400, MUTABLE},
    {"uavcan.pub.imu.gyro.id",              1, 65535, 2401, MUTABLE},
    {"uavcan.pub.imu.imu.id",               1, 65535, 2300, MUTABLE},

    {"uavcan.pub.esc.fb.0.id",              1, 65535, 2500, MUTABLE},
    {"uavcan.pub.esc.fb.1.id",              1, 65535, 2501, MUTABLE},
    {"uavcan.pub.esc.fb.2.id",              1, 65535, 2502, MUTABLE},
    {"uavcan.pub.esc.fb.3.id",              1, 65535, 2503, MUTABLE},

    {"uavcan.pub.aspd.dpres.0.id",          1, 65535, 2600, MUTABLE},
    {"uavcan.pub.aspd.dpres.1.id",          1, 65535, 2601, MUTABLE},

    {"uavcan.pub.energy_source.id",         1, 65535, 2700, MUTABLE},
    {"uavcan.pub.battery_status.id",        1, 65535, 2701, MUTABLE},
    {"uavcan.pub.battery_parameters.id",    1, 65535, 2702, MUTABLE},

    {"uavcan.pub.range.id",                 1, 65535, 2800, MUTABLE},
};
IntegerParamValue_t integer_values_pool[sizeof(integer_desc_pool) / sizeof(IntegerDesc_t)];

StringDesc_t string_desc_pool[NUM_OF_STR_PARAMS] = {
    {"name",                            "co.racconlab.cyphal_communicator", IMMUTABLE},

    {"uavcan.sub.setpoint.type",        "reg.udral.service.actuator.common.sp.Vector4", IMMUTABLE},
    {"uavcan.sub.readiness.type",       "reg.udral.service.common.Readiness", IMMUTABLE},
    {"uavcan.sub.rgbled.type",          "reg.udral.physics.optics.HighColor", IMMUTABLE},

    {"uavcan.pub.mag.type",             "uavcan.si.sample.magnetic_field_strength.Vector3", IMMUTABLE},

    {"uavcan.pub.baro.temp.type",       "uavcan.si.sample.temperature.Scalar", IMMUTABLE},
    {"uavcan.pub.baro.press.type",      "uavcan.si.sample.pressure.Scalar", IMMUTABLE},

    {"uavcan.pub.gps.point.type",       "reg.udral.physics.kinematics.geodetic.PointStateVarTs", IMMUTABLE},
    {"uavcan.pub.gps.sats.type",        "uavcan.primitive.scalar.Integer16", IMMUTABLE},
    {"uavcan.pub.gps.status.type",      "uavcan.primitive.scalar.Integer16", IMMUTABLE},
    {"uavcan.pub.gps.pdop.type",        "uavcan.primitive.scalar.Integer16", IMMUTABLE},

    {"uavcan.pub.imu.accel.type",       "uavcan.si.sample.acceleration.Vector3", IMMUTABLE},
    {"uavcan.pub.imu.gyro.type",        "uavcan.si.sample.angular_velocity.Vector3", IMMUTABLE},
    {"uavcan.pub.imu.imu.type",         "uavcan.primitive.array.Real16", IMMUTABLE},

    {"uavcan.pub.esc.fb.0.type",        "zubax.telega.CompactFeedback.0.1", IMMUTABLE},
    {"uavcan.pub.esc.fb.1.type",        "zubax.telega.CompactFeedback.0.1", IMMUTABLE},
    {"uavcan.pub.esc.fb.2.type",        "zubax.telega.CompactFeedback.0.1", IMMUTABLE},
    {"uavcan.pub.esc.fb.3.type",        "zubax.telega.CompactFeedback.0.1", IMMUTABLE},

    {"uavcan.pub.aspd.dpres.type",      "uavcan.si.sample.pressure.Scalar", IMMUTABLE},
    {"uavcan.pub.aspd.dpres.type",      "uavcan.si.sample.pressure.Scalar", IMMUTABLE},

    {"uavcan.pub.energy_source.type",   "reg.udral.physics.electricity.SourceTs", IMMUTABLE},
    {"uavcan.pub.battery_status.type",  "reg.udral.service.battery.Status", IMMUTABLE},
    {"uavcan.pub.battery_parameters.type",  "reg.udral.service.battery.Parameters", IMMUTABLE},

    {"uavcan.pub.range.type",           "uavcan.si.sample.length.Scalar", IMMUTABLE},
};
StringParamValue_t string_values_pool[sizeof(string_desc_pool) / sizeof(StringDesc_t)];
