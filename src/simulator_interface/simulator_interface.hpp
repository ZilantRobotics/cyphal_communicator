/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#ifndef AUTOPILOT_HITL_SIMULATOR_INTERFACE_
#define AUTOPILOT_HITL_SIMULATOR_INTERFACE_

#include <stdint.h>
#include <cstddef>
#include <arpa/inet.h>
#include <array>
#include <vector>
#include <string>
#include <vector>
#include <functional>
#include "math.hpp"


typedef std::function<void(float, float)> BaroCallback;
typedef std::function<void(Vector3&, Vector3&)> GnssCallback;
typedef std::function<void(Vector3&, Vector3&)> ImuCallback;
typedef std::function<void(Vector3&)> MagCallback;

typedef std::function<void(float)> AirCallback;
typedef std::function<void(float)> EscFeedbackCallback;
typedef std::function<void(float)> RangefinderCallback;


class SimulatorBaseInterface{
public:
    SimulatorBaseInterface() = default;
    virtual bool init() = 0;

    /**
     * @note You send should send setpoint with at least 200 Hz frequency
     */
    virtual bool send_setpoint(const Setpoint16& setpoint) = 0;

    virtual void send_arming_status(bool armings_status) {(void)armings_status;}

    /**
     * @note The implementation can have a blocking call.
     * @return true if there is a new sensor data, false otherwise
     */
    virtual bool spin_once() = 0;


    void subscribe_baro(BaroCallback lambda) { baro_callbacks.push_back(lambda); }
    void subscribe_gnss(GnssCallback lambda) { gnss_callbacks.push_back(lambda); }
    void subscribe_imu(ImuCallback lambda) { imu_callbacks.push_back(lambda); }
    void subscribe_mag(MagCallback lambda) { magnetometer_callbacks.push_back(lambda); }

    void subscribe_airspeed(AirCallback lambda) { airspeed_callbacks.push_back(lambda); }
    void subscribe_esc_feedback(EscFeedbackCallback lambda) { esc_feedback_callbacks.push_back(lambda); }
    void subscribe_rangefidner(RangefinderCallback lambda) { rangefidner_callbacks.push_back(lambda); }

    double get_last_recv_timestamp() {return _last_recv_timestamp; }
protected:
    std::vector<BaroCallback> baro_callbacks;
    std::vector<GnssCallback> gnss_callbacks;
    std::vector<ImuCallback> imu_callbacks;
    std::vector<MagCallback> magnetometer_callbacks;

    std::vector<AirCallback>airspeed_callbacks;
    std::vector<EscFeedbackCallback>esc_feedback_callbacks;
    std::vector<RangefinderCallback>rangefidner_callbacks;

    double _last_recv_timestamp{0.0};
};

#endif  // AUTOPILOT_HITL_SIMULATOR_INTERFACE_
