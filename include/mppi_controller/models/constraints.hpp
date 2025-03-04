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

namespace mppi::models
{

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints
{
  double vx_max;
  double vx_min;
  double vy;
  double wz;
  double max_vel_trans;
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

}  // namespace mppi::models

#endif  // MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
