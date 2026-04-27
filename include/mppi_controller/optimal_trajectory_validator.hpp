// Copyright (c) 2025 Open Navigation LLC
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

#ifndef MPPI_CONTROLLER__OPTIMAL_TRAJECTORY_VALIDATOR_HPP_
#define MPPI_CONTROLLER__OPTIMAL_TRAJECTORY_VALIDATOR_HPP_

#include <memory>
#include <string>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <xtensor/xtensor.hpp>

#include "mppi_controller/models/control_sequence.hpp"
#include "mppi_controller/models/optimizer_settings.hpp"

namespace mppi
{

enum class ValidationResult
{
  SUCCESS,
  SOFT_RESET,
  FAILURE,
};

class OptimalTrajectoryValidator
{
public:
  using Ptr = boost::shared_ptr<OptimalTrajectoryValidator>;

  OptimalTrajectoryValidator() = default;
  virtual ~OptimalTrajectoryValidator() = default;

  virtual void initialize(
    const ros::NodeHandle& parent_nh,
    const std::string& name,
    costmap_2d::Costmap2DROS* costmap,
    const models::OptimizerSettings& settings)
  {
    name_ = name;
    costmap_ros_ = costmap;
    world_model_ = std::make_unique<base_local_planner::CostmapModel>(*costmap_ros_->getCostmap());

    ros::NodeHandle validator_nh(parent_nh, name_);
    validator_nh.param("collision_lookahead_time", collision_lookahead_time_, 2.0);
    validator_nh.param("consider_footprint", consider_footprint_, false);

    traj_samples_to_evaluate_ = static_cast<unsigned int>(collision_lookahead_time_ / settings.model_dt);
    if (traj_samples_to_evaluate_ > static_cast<unsigned int>(settings.time_steps))
    {
      traj_samples_to_evaluate_ = settings.time_steps;
      ROS_WARN_NAMED(
        "OptimalTrajectoryValidator",
        "Collision lookahead time is greater than available trajectory samples, clamping to %u.",
        traj_samples_to_evaluate_);
    }
  }

  virtual ValidationResult validateTrajectory(
    const xt::xtensor<float, 2>& optimal_trajectory,
    const models::ControlSequence& /*control_sequence*/,
    const geometry_msgs::PoseStamped& /*robot_pose*/,
    const geometry_msgs::Twist& /*robot_speed*/,
    const nav_msgs::Path& /*plan*/)
  {
    if (!costmap_ros_ || optimal_trajectory.shape().empty())
    {
      return ValidationResult::SUCCESS;
    }

    auto* costmap = costmap_ros_->getCostmap();
    const bool tracking_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
    const size_t samples = std::min<size_t>(traj_samples_to_evaluate_, optimal_trajectory.shape()[0]);

    for (size_t i = 0; i < samples; ++i)
    {
      const double x = static_cast<double>(optimal_trajectory(i, 0));
      const double y = static_cast<double>(optimal_trajectory(i, 1));

      if (consider_footprint_)
      {
        const double theta = static_cast<double>(optimal_trajectory(i, 2));
        const double footprint_cost = world_model_->footprintCost(
          x, y, theta, costmap_ros_->getRobotFootprint());

        if (footprint_cost == -1.0)
        {
          return ValidationResult::SOFT_RESET;
        }

        if ((footprint_cost == -2.0 || footprint_cost == -3.0) && !tracking_unknown)
        {
          return ValidationResult::SOFT_RESET;
        }
        continue;
      }

      unsigned int mx = 0u;
      unsigned int my = 0u;
      if (!costmap->worldToMap(x, y, mx, my))
      {
        continue;
      }

      const unsigned char cost = costmap->getCost(mx, my);
      if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return ValidationResult::SOFT_RESET;
      }

      if (cost == costmap_2d::NO_INFORMATION && !tracking_unknown)
      {
        return ValidationResult::SOFT_RESET;
      }
    }

    return ValidationResult::SUCCESS;
  }

protected:
  std::string name_;
  costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;
  double collision_lookahead_time_{2.0};
  unsigned int traj_samples_to_evaluate_{0u};
  bool consider_footprint_{false};
};

}  // namespace mppi

#endif  // MPPI_CONTROLLER__OPTIMAL_TRAJECTORY_VALIDATOR_HPP_
