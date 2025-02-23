// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey
// Budyakov
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

#ifndef MPPI_CONTROLLER__MODELS__STATE_HPP_
#define MPPI_CONTROLLER__MODELS__STATE_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <xtensor/xtensor.hpp>

namespace mppi::models {

/**
 * @struct mppi::models::State
 * @brief State information: velocities, controls, poses, speed
 */
struct State {
  xt::xtensor<double, 2, xt::layout_type::column_major> vx;
  xt::xtensor<double, 2, xt::layout_type::column_major> vy;
  xt::xtensor<double, 2, xt::layout_type::column_major> wz;

  xt::xtensor<double, 2, xt::layout_type::column_major> cvx;
  xt::xtensor<double, 2, xt::layout_type::column_major> cvy;
  xt::xtensor<double, 2, xt::layout_type::column_major> cwz;

  geometry_msgs::PoseStamped pose;
  geometry_msgs::Twist speed;

  /**
   * @brief Reset state data
   */
  void reset(unsigned int batch_size, unsigned int time_steps) {
    vx = xt::zeros<double>({batch_size, time_steps});
    vy = xt::zeros<double>({batch_size, time_steps});
    wz = xt::zeros<double>({batch_size, time_steps});

    cvx = xt::zeros<double>({batch_size, time_steps});
    cvy = xt::zeros<double>({batch_size, time_steps});
    cwz = xt::zeros<double>({batch_size, time_steps});
  }
};
}  // namespace mppi::models

#endif  // MPPI_CONTROLLER__MODELS__STATE_HPP_
