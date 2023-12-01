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

#ifndef MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
#define MPPI_CONTROLLER__CRITIC_MANAGER_HPP_

#include <costmap_2d/costmap_2d_ros.h>

#include <memory>
#include <pluginlib/class_loader.hpp>
#include <string>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "mppi_controller/critic_data.hpp"
#include "mppi_controller/critic_function.hpp"
#include "mppi_controller/tools/utils.hpp"
#include "mppi_controller/models/constraints.hpp"

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */
class CriticManager
{
public:
  /**
   * @brief Constructor for mppi::CriticManager
   */
  CriticManager();

  /**
   * @brief Virtual Destructor for mppi::CriticManager
   */
  virtual ~CriticManager() = default;

  /**
   * @brief Configure critic manager on bringup and load plugins
   * @param parent_nh Private parent node handle
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   */
  void on_configure(const ros::NodeHandle& parent_nh, const std::string& name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Score trajectories by the set of loaded critic functions
   * @param CriticData Struct of necessary information to pass to the critic
   * functions
   */
  void evalTrajectoriesScores(CriticData& data) const;

  void updateConstraints(const models::ControlConstraints& constraints);

protected:
  /**
   * @brief Get parameters (critics to load)
   */
  void getParams();

protected:
  ros::NodeHandle parent_nh_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  std::string name_;

  pluginlib::ClassLoader<critics::CriticFunction> loader_;
  std::vector<boost::shared_ptr<critics::CriticFunction>> critics_;
};

}  // namespace mppi

#endif  // MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
