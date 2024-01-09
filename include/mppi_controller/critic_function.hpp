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

#ifndef MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
#define MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <costmap_2d/costmap_2d_ros.h>
#include "mppi_controller/BaseCriticConfig.h"

#include <memory>
#include <string>

#include "mppi_controller/critic_data.hpp"
#include "mppi_controller/models/constraints.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{ 0 };
  bool using_footprint{ false };
};

class CriticBase
{
public:
  virtual void on_configure(const ros::NodeHandle& parent_nh, const std::string& name,
                            costmap_2d::Costmap2DROS* costmap_ros) = 0;
  virtual void score(CriticData& data) = 0;
  virtual void initialize() = 0;
  virtual std::string getName() = 0;
  virtual void updateConstraints(const models::ControlConstraints& constraints) = 0;
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
template <typename Config = mppi_controller::BaseCriticConfig>
class CriticFunction : public CriticBase
{
public:
  /**
   * @brief Constructor for mppi::critics::CriticFunction
   */
  CriticFunction() = default;

  /**
   * @brief Destructor for mppi::critics::CriticFunction
   */
  virtual ~CriticFunction() = default;

  /**
   * @brief Configure critic on bringup
   * @param parent_nh Private parent node handle
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
  void on_configure(const ros::NodeHandle& parent_nh, const std::string& name,
                    costmap_2d::Costmap2DROS* costmap_ros) override final
  {
    name_ = name;
    parent_nh_ = parent_nh;
    pnh_ = ros::NodeHandle(parent_nh, name);
    costmap_ros_ = costmap_ros;

    dsrv_ = std::make_unique<dynamic_reconfigure::Server<Config>>(pnh_);
    dsrv_->setCallback(boost::bind(&CriticFunction::reconfigureCB, this, _1, _2));

    initialize();
  }

  /**
   * @brief Main function to score trajectory
   * @param data Critic data to use in scoring
   */
  virtual void score(CriticData& data) = 0;

  /**
   * @brief Initialize critic
   */
  virtual void initialize() = 0;

  /**
   * @brief Get name of critic
   */
  std::string getName() override final
  {
    return name_;
  }

  inline virtual void updateConstraints(const models::ControlConstraints& constraints)
  {
    std::lock_guard<std::mutex> lock(constraint_mtx_);
    constraints_ = constraints;
  }

protected:
  std::string name_;
  ros::NodeHandle parent_nh_;
  ros::NodeHandle pnh_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  // common parameters to all critics
  bool enabled_ = false;
  double power_ = 0.0;
  double weight_ = 0.0;
  std::mutex constraint_mtx_;
  models::ControlConstraints constraints_;

  std::unique_ptr<dynamic_reconfigure::Server<Config>> dsrv_;

  virtual void reconfigureCB(Config& config, uint32_t level)
  {
    power_ = config.cost_power;
    weight_ = config.cost_weight;
    enabled_ = config.enabled;
  }
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
