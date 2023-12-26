// This software is distributed under the terms of the GPL v3 License.
// Copyright (c) 2023 Dmitry Ponomarev.
// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#ifndef SIMULATOR_INTERFACE_AP_JSON_
#define SIMULATOR_INTERFACE_AP_JSON_

#include <stdint.h>
#include <cstddef>
#include <arpa/inet.h>
#include <array>
#include <vector>
#include <string>
#include "simulator_interface/simulator_interface.hpp"

struct servo_packet {
    uint16_t magic; // 18458 expected magic value
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[16];
};

class ArdupilotJsonInterface : public SimulatorBaseInterface{
public:
    ArdupilotJsonInterface(double home_lat, double home_lon, double home_alt);
    bool init() override;
    bool send_setpoint(const Setpoint16& setpoint) override;
    bool spin_once() override;

private:
    bool parse_json(const char* buffer, int size);

    bool parse_json_list(const std::string& str,
                         size_t first_idx,
                         size_t last_idx,
                         std::vector<double>& numbers) const;
    int _client_fd;
    servo_packet _servo_pkt;
    struct sockaddr_in _server_addr;

    inline static const Vector3 _initial_mag_gauss{0.232, 0.052, -0.528};
    Vector3 _home;

    Vector3 gyro = {0, 0, 0};
    Vector3 accel = {0, 0, 0};
    Vector3 position = {0, 0, 0};
    Quaternion quaternion_wxyz = {1, 0, 0, 0};
    Vector3 velocity = {0, 0, 0};
};

#endif  // SIMULATOR_INTERFACE_AP_JSON_
