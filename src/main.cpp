/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include "main.h"

#include <algorithm>
#include <iostream>
#include <chrono>
#include <thread>

#include "autopilot_interface/cyphal_hitl.hpp"
#include "simulator_interface/ap_json/ap_json.hpp"
#include "simulator_interface/ros_interface/ros_interface.hpp"

static void print_log_info_periodically(uint32_t& json_sensors_recv_counter,
                                        uint32_t cyphal_servo_recv_counter,
                                        double crnt_ts,
                                        CyphalHitlInterface& cyphal_hitl);


std::unique_ptr<SimulatorBaseInterface> init_ardupilot_json_sim() {
    std::unique_ptr<ArdupilotJsonInterface> sim(new ArdupilotJsonInterface(-35.3632621, +149.1652374, 584.19));
    if (sim == nullptr || !sim->init()) {
        std::cout << "ArduPilot Initialization Error." << std::endl;
    }
    Setpoint16 setpoint;
    sim->send_setpoint(setpoint);

    std::cout << "Hello, ArduPilot JSON." << std::endl;

    return sim;
}

std::unique_ptr<SimulatorBaseInterface> init_ros_sim(int argc, char** argv) {
    std::unique_ptr<RosInterface> sim(new RosInterface(argc, argv));
    sim->init();

    return sim;
}

int main(int argc, char** argv) {
    CyphalHitlInterface cyphal_hitl;
    int init_res = cyphal_hitl.init();
    if (init_res < 0) {
        std::cout << "Cyphal HITL Initialization Error: " << init_res << std::endl;
        return -1;
    }
    std::cout << "Hello, Cyphal HITL." << std::endl;

    uint32_t json_sensors_recv_counter = 0;
    std::unique_ptr<SimulatorBaseInterface> simulator = init_ros_sim(argc, argv);

    simulator->subscribe_baro([&cyphal_hitl](float pressure, float temperature) {
        cyphal_hitl.publish_barometer(pressure, temperature);
    });

    simulator->subscribe_gnss([&cyphal_hitl](Vector3& global_pose, Vector3& ned_velocity) {
        cyphal_hitl.publish_gnss(global_pose, ned_velocity);
    });

    simulator->subscribe_imu([&cyphal_hitl](Vector3& accel, Vector3& gyro) {
        cyphal_hitl.publish_imu(accel, gyro);
    });

    simulator->subscribe_mag([&cyphal_hitl](Vector3& magnetic_field_gauss) {
        cyphal_hitl.publish_magnetometer(magnetic_field_gauss);
    });

    while(true) {
        cyphal_hitl.process();

        Setpoint16 setpoint;
        if (cyphal_hitl.get_setpoint(setpoint)) {
            simulator->send_setpoint(setpoint);
        }

        if (simulator->receive_sensors()) {
            json_sensors_recv_counter++;
        }

        print_log_info_periodically(json_sensors_recv_counter,
                                    cyphal_hitl.get_setpoint_recv_counter(),
                                    simulator->get_last_recv_timestamp(),
                                    cyphal_hitl);
    }
}

void print_log_info_periodically(uint32_t& json_sensors_recv_counter,
                                 uint32_t cyphal_servo_recv_counter,
                                 double crnt_ts,
                                 CyphalHitlInterface& cyphal_hitl) {
    (void)cyphal_hitl;
    uint32_t crnt_time_ms = HAL_GetTick();
    static uint32_t last_hint_time_ms = 0;
    if (crnt_time_ms < last_hint_time_ms + 1000) {
        return;
    }
    last_hint_time_ms = crnt_time_ms;

    static double prev_ts = crnt_ts - 1.0;
    double time_factor = std::clamp(crnt_ts - prev_ts, 0.5, 2.0);
    std::cout << "Status: "
              << "gz time factor = " << (int)(100 * time_factor) << "%, "
              << "cyphal input = " << (int)(0.5 * cyphal_servo_recv_counter) << "%, "
              << "json input = " << (int)(0.1 * json_sensors_recv_counter) << "%."
              << std::endl;

    if (time_factor < 0.8 || time_factor > 1.1) {
        std::cout << "\033[1;31m"
                  << "Gazebo time factor should be ~ 1.0. "
                  << "Now it is not enough for a stable flight. "
                  << "Try headless-rendering for gazebo."
                  << "\033[0m\n";
    }

    if (cyphal_servo_recv_counter < 180) {
        std::cout << "\033[1;31m"
                  << "Setpoint rate should be ~200 Hz. "
                  << "Don't you have a problem with transport layer?"
                  << "\033[0m\n";
    }

    // cyphal_hitl.clear_servo_pwm_counter();
    // cyphal_hitl.set_time_factor(time_factor);
    json_sensors_recv_counter = 0;
    prev_ts = crnt_ts;
}
