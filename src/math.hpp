// This software is distributed under the terms of the GPL v3 License.
// Copyright (c) 2023 Dmitry Ponomarev.
// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

#ifndef HITL_MATH_HPP_
#define HITL_MATH_HPP_

#include <array>

#define RAD_TO_DEGREE   (0.017453293)

typedef std::array<float, 16> Setpoint16;
typedef std::array<double, 3> Vector3;
typedef std::array<double, 4> Quaternion;

void rotate_vector_by_quaternion(const Vector3& v, const Quaternion& q, Vector3& res);

void local_pose_to_global(const Vector3& home,
                          const Vector3& local_position,
                          Vector3& global_position);

float local_height_to_baro_pressure_pascal(float local_height_meters);

#endif  // HITL_MATH_HPP_
