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
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include <memory>
#include <string>

#include "mppi_controller/critic_data.hpp"

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

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
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
  void on_configure(const ros::NodeHandle& parent_nh, const std::string& name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    name_ = name;
    parent_nh_ = parent_nh;
    pnh_ = ros::NodeHandle(parent_nh, name);
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ddynamic_reconfig_.emplace(pnh_);
    ddynamic_reconfig_->registerVariable<bool>("enabled", &enabled_, boost::bind(&CriticFunction::toggle, this, _1),
                                               "Enable/disable the critic");
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
  std::string getName()
  {
    return name_;
  }

private:
  inline void toggle(bool enabled)
  {
    if (enabled)
    {
      ROS_INFO_NAMED(name_, "Critic enabled");
      initialize();
    }
    else
    {
      ROS_INFO_NAMED(name_, "Critic enabled");
    }
  }

protected:
  bool enabled_ = true;
  std::string name_;
  ros::NodeHandle parent_nh_;
  ros::NodeHandle pnh_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_{ nullptr };
  std::optional<ddynamic_reconfigure::DDynamicReconfigure> ddynamic_reconfig_;
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
