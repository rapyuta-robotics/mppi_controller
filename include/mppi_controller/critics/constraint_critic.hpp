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

#ifndef MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
#define MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "mppi_controller/critic_function.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/tools/utils.hpp"
#include "mppi_controller/BaseCriticConfig.h"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 */
class ConstraintCritic : public CriticFunction<>
{
public:
  /**
   * @brief Initialize critic
   */
  void initialize() override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData& data) override;

  float getMaxVelConstraint()
  {
    return max_vel_;
  }

  float getMinVelConstraint()
  {
    return min_vel_;
  }

  void updateConstraints(const models::ControlConstraints& constraints) override final;

protected:
  float min_vel_;
  float max_vel_;
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
