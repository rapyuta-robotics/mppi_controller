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

#ifndef MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
#define MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_

#include <string>
#include "mppi_controller/critic_function.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/tools/utils.hpp"
#include <mppi_controller/PathAngleCriticConfig.h>

namespace mppi::critics
{

/**
 * @brief Enum type for different modes of operation
 */
enum class PathAngleMode
{
  FORWARD_PREFERENCE = 0,
  NO_DIRECTIONAL_PREFERENCE = 1,
  CONSIDER_FEASIBLE_PATH_ORIENTATIONS = 2
};

/**
 * @brief Method to convert mode enum to string for printing
 */
std::string modeToStr(const PathAngleMode & mode)
{
  if (mode == PathAngleMode::FORWARD_PREFERENCE) {
    return "Forward Preference";
  } else if (mode == PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS) {
    return "Consider Feasible Path Orientations";
  } else if (mode == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
    return "No Directional Preference";
  } else {
    return "Invalid mode!";
  }
}

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for aligning to path in cases of extreme misalignment
 * or turning
 */
class PathAngleCritic : public CriticFunction
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

  void updateConstraints(const models::ControlConstraints& constraints) override;

protected:
  std::unique_ptr<dynamic_reconfigure::Server<mppi_controller::PathAngleCriticConfig>> dsrv_;
  float max_angle_to_furthest_{0};
  float threshold_to_consider_{0};

  size_t offset_from_furthest_{0};
  bool reversing_allowed_{true};
  PathAngleMode mode_{0};

  unsigned int power_{0};
  float weight_{0};

private:
  inline void reconfigureCB(mppi_controller::PathAngleCriticConfig& config, uint32_t level)
  {
    max_angle_to_furthest_ = config.max_angle_to_furthest;
    threshold_to_consider_ = config.threshold_to_consider;
    offset_from_furthest_ = config.offset_from_furthest;
    mode_ = static_cast<PathAngleMode>(config.mode);
  }
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
