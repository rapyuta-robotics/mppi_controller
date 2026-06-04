// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
#define MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_

#include <cmath>

namespace mppi::models
{

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints
{
  double vx_max        = 0.0;
  double vx_min        = 0.0;
  double vy            = 0.0;
  double wz            = 0.0;
  double max_vel_trans = 0.0;
  double max_accel_trans   = 0.0;  // 0 = disabled
  double max_decel_trans   = 0.0;  // 0 = disabled
  double max_accel_angular = 0.0;  // 0 = disabled
  double max_decel_angular = 0.0;  // 0 = disabled
};

/**
 * @struct mppi::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 */
struct SamplingStd
{
  double vx;
  double vy;
  double wz;
};

/**
 * @brief Clamp a velocity step to respect acceleration/deceleration limits.
 *        Magnitude-based: |v| increasing = acceleration, |v| decreasing = deceleration.
 *        A limit of 0 means disabled (no clamping applied).
 */
inline float clampByAccel(float v_prev, float v_curr, double max_accel, double max_decel, float dt)
{
  if (max_accel <= 0.0 && max_decel <= 0.0) {
    return v_curr;
  }
  const float abs_prev = std::fabs(v_prev);
  const float abs_curr = std::fabs(v_curr);
  if (abs_curr > abs_prev) {
    if (max_accel > 0.0) {
      const float allowed = abs_prev + static_cast<float>(max_accel) * dt;
      if (abs_curr > allowed) {
        return std::copysign(allowed, v_curr);
      }
    }
  } else {
    if (max_decel > 0.0) {
      float allowed = abs_prev - static_cast<float>(max_decel) * dt;
      if (allowed < 0.0f) {
        allowed = 0.0f;
      }
      if (abs_curr < allowed) {
        return std::copysign(allowed, v_prev);
      }
    }
  }
  return v_curr;
}

}  // namespace mppi::models

#endif  // MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
