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

#ifndef MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
#define MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_

#include "mppi_controller/critic_function.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/tools/utils.hpp"

#include <mppi_controller/PathHeadingCriticConfig.h>

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for aligning to the path. Note:
 * High settings of this will follow the path more precisely, but also makes it
 * difficult (or impossible) to deviate in the presence of dynamic obstacles.
 * This is an important critic to tune and consider in tandem with Obstacle.
 */
class PathHeadingCritic : public CriticFunction<mppi_controller::PathHeadingCriticConfig>
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;

private:
  void reconfigureCB(mppi_controller::PathHeadingCriticConfig& config, uint32_t level);

protected:
  size_t offset_from_furthest_{0};
  int trajectory_point_step_{0};
  float threshold_to_consider_{0};
  float max_path_occupancy_ratio_{0};
  bool use_path_orientations_{ false };
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
