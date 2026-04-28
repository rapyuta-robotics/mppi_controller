// Copyright (c) 2023 Robocc Brice Renaudeau
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

#ifndef MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
#define MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_

#include <memory>
#include <string>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/inflation_layer.h>

#include "mppi_controller/CostCriticConfig.h"
#include "mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class CostCritic : public CriticFunction<mppi_controller::CostCriticConfig>
{
public:
  void initialize() override;
  void score(CriticData& data) override;

protected:
  inline bool inCollision(float cost, float x, float y, float theta) const;
  float findCircumscribedCost(costmap_2d::Costmap2DROS* costmap);

private:
  void reconfigureCB(mppi_controller::CostCriticConfig& config, uint32_t level);

protected:
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;

  bool consider_footprint_{true};
  bool is_tracking_unknown_{true};
  float possible_collision_cost_{0.0f};
  float circumscribed_radius_{0.0f};
  float circumscribed_cost_{0.0f};
  float collision_cost_{0.0f};
  float critical_cost_{0.0f};
  float near_goal_distance_{1.0f};
  unsigned int near_collision_cost_{253u};
  unsigned int trajectory_point_step_{2u};
  std::string inflation_layer_name_{""};
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
