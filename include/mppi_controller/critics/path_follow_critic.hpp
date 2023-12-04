// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
#define MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_

#include "mppi_controller/critic_function.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/tools/utils.hpp"
#include "mppi_controller/PathFollowCriticConfig.h"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for following the path approximately
 * To allow for deviation from path in case of dynamic obstacles. Path Align
 * is what aligns the trajectories to the path more or less precisely, if desireable.
 * A higher weight here with an offset > 1 will accelerate the samples to full speed
 * faster and push the follow point further ahead, creating some shortcutting.
 */
class PathFollowCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  std::unique_ptr<dynamic_reconfigure::Server<mppi_controller::PathFollowCriticConfig>> dsrv_;

  int offset_from_furthest_{ 0 };
  double threshold_to_consider_{ 0.0 };

private:
  inline void reconfigureCB(mppi_controller::PathFollowCriticConfig& config, uint32_t level)
  {
    offset_from_furthest_ = config.offset_from_furthest;
    threshold_to_consider_ = config.threshold_to_consider;
  }
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
