/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2023 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#include "main.h"

#include <algorithm>
#include <iostream>
#include <chrono>
#include <thread>

#include "autopilot_interface/cyphal_hitl.hpp"

#ifdef ARDUPILOT_JSON_SIM_INTERFACE
#include "simulator_interface/ap_json/ap_json.hpp"
static std::unique_ptr<SimulatorBaseInterface> init_sim_interface(int argc, char** argv, CyphalHitlInterface& cyphal_hitl) {
    (void)argc;
    (void)argv;
    (void)cyphal_hitl;
    std::unique_ptr<ArdupilotJsonInterface> sim(new ArdupilotJsonInterface(-35.3632621, +149.1652374, 584.19));
    if (sim == nullptr || !sim->init()) {
        std::cout << "ArduPilot Initialization Error." << std::endl;
    }
    Setpoint16 setpoint;
    sim->send_setpoint(setpoint, 16);

    std::cout << "Hello, ArduPilot JSON." << std::endl;

    return sim;
}
#else
#include "simulator_interface/ros_interface/ros_interface.hpp"
static std::unique_ptr<SimulatorBaseInterface> init_sim_interface(int argc, char** argv, CyphalHitlInterface& cyphal_hitl) {
    (void)cyphal_hitl;
    std::unique_ptr<RosInterface> sim(new RosInterface(argc, argv));
    sim->init();

    sim->subscribe_esc_feedback([&cyphal_hitl](uint8_t esc_idx, float voltage, float curent, uint32_t rpm) {
        cyphal_hitl.publish_esc_feedback(esc_idx, voltage, curent, rpm);
    });

    sim->subscribe_battery([&cyphal_hitl](const BatteryStatus& b) {
        cyphal_hitl.publish_battery(b.voltage, b.current, b.temperature_kelvin, b.full_capacity_ah, b.remaining_capacity_ah);
    });

    sim->subscribe_diff_pressure([&cyphal_hitl](float diff_pressure) {
        cyphal_hitl.publish_diff_pressure(diff_pressure);
    });

    sim->subscribe_rangefidner([&cyphal_hitl](float range) {
        cyphal_hitl.publish_rangefinder(range);
    });

    return sim;
}
#endif


int main(int argc, char** argv) {
    CyphalHitlInterface cyphal_hitl;
    int init_res = cyphal_hitl.init();
    if (init_res < 0) {
        std::cout << "Cyphal HITL Initialization Error: " << init_res << std::endl;
        return -1;
    }
    std::cout << "Hello, Cyphal HITL." << std::endl;

    std::unique_ptr<SimulatorBaseInterface> simulator = init_sim_interface(argc, argv, cyphal_hitl);

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
        auto setpoint_size = cyphal_hitl.get_setpoint(setpoint);
        if (setpoint_size) {
            simulator->send_setpoint(setpoint, setpoint_size);
        }

        uint32_t crnt_time_ms = HAL_GetTick();
        static uint32_t last_arm_send_time_ms = 0;
        if (crnt_time_ms > last_arm_send_time_ms + 100) {
            last_arm_send_time_ms = crnt_time_ms;
            simulator->send_arming_status(cyphal_hitl.get_arming_status());
        }

        simulator->spin_once();
    }
}
