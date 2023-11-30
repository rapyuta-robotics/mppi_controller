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

#ifndef MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <memory>
#include <optional>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "mppi_controller/models/path.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/trajectories.hpp"
#include "mppi_controller/motion_models.hpp"

namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories,
 * path, costs, and important parameters to share
 */
struct CriticData
{
  const models::State& state;
  const models::Trajectories& trajectories;
  const models::Path& path;

  xt::xtensor<float, 1>& costs;
  double& model_dt;

  bool fail_flag;
  std::shared_ptr<MotionModel> motion_model;
  std::optional<std::vector<bool>> path_pts_valid;
  std::optional<size_t> furthest_reached_path_point;
};

}  // namespace mppi

#endif  // MPPI_CONTROLLER__CRITIC_DATA_HPP_
