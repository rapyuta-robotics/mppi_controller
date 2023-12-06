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

#ifndef MPPI_CONTROLLER__CRITICS__TWIRLING_CRITIC_HPP_
#define MPPI_CONTROLLER__CRITICS__TWIRLING_CRITIC_HPP_

#include "mppi_controller/critic_function.hpp"
#include "mppi_controller/tools/utils.hpp"
#include "mppi_controller/TwirlingCriticConfig.h"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for penalizing wiggling/twirling
 */
class TwirlingCritic : public CriticFunction<mppi_controller::TwirlingCriticConfig>
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
  void score(CriticData& data) override;

protected:
  double threshold_to_consider_{ 0 };

private:
  inline void reconfigureCB(mppi_controller::TwirlingCriticConfig& config, uint32_t level) override final
  {
    threshold_to_consider_ = config.threshold_to_consider;
    CriticFunction::reconfigureCB(config, level);
  }
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITICS__TWIRLING_CRITIC_HPP_
