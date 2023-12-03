/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include "ap_json.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

static uint32_t HAL_GetTick() {
    static auto time_start = std::chrono::steady_clock::now();
    auto time_now = std::chrono::steady_clock::now();
    auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_start).count();
    return elapsed_time_ms;
}

ArdupilotJsonInterface::ArdupilotJsonInterface(double home_lat, double home_lon, double home_alt) {
    _servo_pkt.magic = 18458;
    _servo_pkt.frame_rate = 1000;
    _servo_pkt.frame_count = 0;

    _home[0] = home_lat;
    _home[1] = home_lon;
    _home[2] = home_alt;
}

bool ArdupilotJsonInterface::init() {
    if ((_client_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cout << "Socket creation error" << std::endl;
        return false;
    }
    _server_addr.sin_family = AF_INET;
    _server_addr.sin_port = htons(9002);
    if (inet_pton(AF_INET, "127.0.0.1", &_server_addr.sin_addr) <= 0) {
        std::cout << "Invalid address" << std::endl;
        return false;
    }

    int status;
    if ((status = connect(_client_fd, (struct sockaddr*)&_server_addr, sizeof(_server_addr))) < 0) {
        std::cout << "Connection Failed" << std::endl;
        return false;
    }

    return true;
}

bool ArdupilotJsonInterface::send_setpoint(const Setpoint16& setpoint, size_t size) {
    (void)size;

    std::array<uint16_t, 16> servo_pwm;
    for (size_t sp_idx = 0; sp_idx < 4; sp_idx++) {
        servo_pwm[sp_idx] = 1000 + 1000.0 * setpoint[sp_idx];
    }

    for (size_t sp_idx = 4; sp_idx < 16; sp_idx++) {
        servo_pwm[sp_idx] = 0;
    }

    std::copy(std::begin(servo_pwm), std::end(servo_pwm), std::begin(_servo_pkt.pwm));
    send(_client_fd, &_servo_pkt, sizeof(_servo_pkt), 0);
    return true;
}

bool ArdupilotJsonInterface::spin_once() {
    char buffer[1024];
    int buffer_size = read(_client_fd, buffer, 1024);
    if (buffer_size < 0) {
        return false;
    }

    _servo_pkt.frame_count++;
    buffer[buffer_size] = 0;
    parse_json(buffer, buffer_size);

    uint32_t crnt_time_ms = HAL_GetTick();

    static uint32_t gnss_prev_time_ms = 1000;
    if (crnt_time_ms >= gnss_prev_time_ms + 100) {
        gnss_prev_time_ms = crnt_time_ms;
        Vector3 global_pose;
        local_pose_to_global(_home, position, global_pose);
        for (auto gnss_cb : gnss_callbacks) {
            gnss_cb(global_pose, velocity);
        }
    }

    static uint32_t baro_prev_time_ms = 1000;
    if (crnt_time_ms >= baro_prev_time_ms + 100) {
        baro_prev_time_ms = crnt_time_ms;
        float pressure = local_height_to_baro_pressure_pascal(position[2]);
        for (auto baro_cb : baro_callbacks) {
            baro_cb(pressure, 312);
        }
    }

    static uint32_t imu_prev_time_us = 1000000;
    if (crnt_time_ms * 1000 >= imu_prev_time_us + 5000) {
        imu_prev_time_us += 5000;
        for (auto imu_cb : imu_callbacks) {
            imu_cb(accel, gyro);
        }
    }

    static uint32_t mag_prev_time_ms = 1000;
    if (crnt_time_ms >= mag_prev_time_ms + 20) {
        Vector3 magnetic_field_gauss;
        rotate_vector_by_quaternion(_initial_mag_gauss, quaternion_wxyz, magnetic_field_gauss);
        mag_prev_time_ms = crnt_time_ms;
        for (auto mag_cb : magnetometer_callbacks) {
            mag_cb(magnetic_field_gauss);
        }
    }

    return true;
}


// Example:
// {"timestamp":1131.743,
// "imu":{"gyro":[1.138086090003379e-17,-9.26007201697338e-18,-1.7638184005081048e-18],
// "accel_body":[3.1950471578911405e-15,-0.0000033948200292251887,-9.800000002272988]},
// "position":[-5.123849296223462e-19,-6.754994481824757e-8,-0.19499986873364534],
// "quaternion":[1.0,1.5628551970229893e-18,-4.2277906432607347e-19,2.0098806949291254e-19],
// "velocity":[-3.2982527747048776e-18,-3.370127907671133e-13,-9.728853853536417e-7]}
bool ArdupilotJsonInterface::parse_json(const char* buffer, int size) {
    if (buffer == nullptr || size <= 0) {
        return false;
    }

    std::string str(buffer);

    int timestamp_first_char_idx = str.find("timestamp");
    int gyro_first_char_idx = str.find("gyro");
    int accel_first_char_idx = str.find("accel_body", gyro_first_char_idx);
    int position_first_char_idx = str.find("position", accel_first_char_idx);
    int quaternion_first_char_idx = str.find("quaternion", position_first_char_idx);
    int velocity_first_char_idx = str.find("velocity", quaternion_first_char_idx);

    std::vector<double> vector_3d = {0, 0, 0};
    std::vector<double> vector_4d = {0, 0, 0, 0};

    auto ts_string = str.substr(timestamp_first_char_idx+11, gyro_first_char_idx - 20 - timestamp_first_char_idx);
    _last_recv_timestamp = std::stof(ts_string);

    if (!parse_json_list(str, gyro_first_char_idx + 6, accel_first_char_idx - 3, vector_3d)) {
        std::cout << "skip gyro" << std::endl;
    }
    std::copy_n(vector_3d.begin(), 3, gyro.begin());

    if (!parse_json_list(str, accel_first_char_idx + 12, position_first_char_idx - 4, vector_3d)) {
        std::cout << "skip accel" << std::endl;
    }
    std::copy_n(vector_3d.begin(), 3, accel.begin());

    if (!parse_json_list(str, position_first_char_idx + 10, quaternion_first_char_idx - 3, vector_3d)) {
        std::cout << "skip position" << std::endl;
    }
    std::copy_n(vector_3d.begin(), 3, position.begin());

    if (!parse_json_list(str, quaternion_first_char_idx + 12, velocity_first_char_idx - 3, vector_4d)) {
        std::cout << "skip quaternion" << std::endl;
    }
    std::copy_n(vector_4d.begin(), 4, quaternion_wxyz.begin());

    if (!parse_json_list(str, velocity_first_char_idx + 10, size - 3, vector_3d)) {
        std::cout << "skip velocity" << std::endl;
    }
    std::copy_n(vector_3d.begin(), 3, velocity.begin());

    return true;
}

// 3d example: "[-5.123849296223462e-19,-6.754994481824757e-8,-0.19499986873364534]"
// 4d example: "[1.0,1.5628551970229893e-18,-4.2277906432607347e-19,2.0098806949291254e-19]"
bool ArdupilotJsonInterface::parse_json_list(const std::string& str,
                                    size_t first_idx,
                                    size_t last_idx,
                                    std::vector<double>& numbers) const {
    const size_t vector_size = numbers.size();
    if (str[first_idx] != '[' || str[last_idx] != ']' || vector_size > 4) {
        return false;
    }

    first_idx++;

    std::array<int, 3> comma_idx;
    std::array<std::string, 4> str_numbers;

    comma_idx[0] = str.find(',', first_idx);
    str_numbers[0] = str.substr(first_idx, comma_idx[0] - first_idx);
    for (size_t crnt_idx = 1; crnt_idx < vector_size - 1; crnt_idx++) {
        size_t prev_idx = crnt_idx - 1;
        comma_idx[crnt_idx] = str.find(',', comma_idx[prev_idx] + 1);
        str_numbers[crnt_idx] = str.substr(comma_idx[prev_idx] + 1, comma_idx[crnt_idx] - comma_idx[prev_idx] - 1);
    }
    str_numbers[vector_size - 1] = str.substr(comma_idx[vector_size - 2] + 1, last_idx - comma_idx[vector_size - 2] - 1);

    for (size_t number_idx = 0; number_idx < vector_size; number_idx++) {
        numbers[number_idx] = std::stof(str_numbers[number_idx]);
    }

    return true;
}
