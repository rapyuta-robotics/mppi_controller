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

#include <cmath>
#include "mppi_controller/critics/obstacles_critic.hpp"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  world_model_ = std::make_unique<base_local_planner::CostmapModel>(*costmap_ros_->getCostmap());
  possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  pnh_.param("inflation_layer_name", inflation_layer_name_);

  if (possibly_inscribed_cost_ < 1.0)
  {
    ROS_ERROR_NAMED(name_, "Inflation layer either not found or inflation is not set sufficiently for "
                           "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
                           " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
                           "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
                           " for full instructions. This will substantially impact run-time performance.");
  }
}

float ObstaclesCritic::findCircumscribedCost(costmap_2d::Costmap2DROS* costmap)
{
  double result = -1.0;

  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_)
  {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  bool inflation_layer_found = false;
  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    auto inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer || (!inflation_layer_name_.empty() && inflation_layer->getName() != inflation_layer_name_))
    {
      continue;
    }

    inflation_layer_found = true;
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);

    ros::NodeHandle inflation_nh(ros::names::parentNamespace(parent_nh_.getNamespace()), inflation_layer->getName());
    inflation_scale_factor_ = static_cast<float>(inflation_nh.param("cost_scaling_factor", 0.0));
    inflation_radius_ = static_cast<float>(inflation_nh.param("inflation_radius", 0.0));
    break;
  }

  ROS_WARN_COND_NAMED(!inflation_layer_found, name_,
                      "No inflation layer found in costmap configuration. "
                      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
                      "field to speed up collision checking by only checking the full footprint "
                      "when robot is within possibly-inscribed radius of an obstacle. This may "
                      "significantly slow down planning times and not avoid anything but absolute collisions!");

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

float ObstaclesCritic::distanceToObstacle(const CollisionCost& cost)
{
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  float dist_to_obj = (scale_factor * min_radius - log(cost.cost) + log(253.0f)) / scale_factor;

  // If not footprint collision checking, the cost is using the center point cost and
  // needs the radius subtracted to obtain the closest distance to the object
  if (!cost.using_footprint)
  {
    dist_to_obj -= min_radius;
  }

  return dist_to_obj;
}

void ObstaclesCritic::score(CriticData& data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_)
  {
    return;
  }

  if (consider_footprint_)
  {
    // footprint may have changed since initialization if user has dynamic footprints
    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path))
  {
    near_goal = true;
  }

  auto&& raw_cost = xt::xtensor<float, 1>::from_shape({ data.costs.shape(0) });
  raw_cost.fill(0.0f);
  auto&& repulsive_cost = xt::xtensor<float, 1>::from_shape({ data.costs.shape(0) });
  repulsive_cost.fill(0.0f);

  const size_t traj_len = data.trajectories.x.shape(1);
  bool all_trajectories_collide = true;
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i)
  {
    bool trajectory_collide = false;
    float traj_cost = 0.0f;
    const auto& traj = data.trajectories;
    CollisionCost pose_cost;

    for (size_t j = 0; j < traj_len; j++)
    {
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));
      if (!pose_cost.cost)
      {
        continue;
      }  // In free space, nothing else to do

      if (inCollision(pose_cost.cost))
      {
        trajectory_collide = true;
        break;
      }

      // Cannot process repulsion if inflation layer does not exist
      if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f)
      {
        continue;
      }

      const float dist_to_obj = distanceToObstacle(pose_cost);

      // Let near-collision trajectory points be punished severely
      if (dist_to_obj < collision_margin_distance_)
      {
        traj_cost += (collision_margin_distance_ - dist_to_obj);
      }

      // Generally prefer trajectories further from obstacles
      if (!near_goal)
      {
        repulsive_cost[i] += (inflation_radius_ - dist_to_obj);
      }
    }

    if (!trajectory_collide)
    {
      all_trajectories_collide = false;
    }
    raw_cost[i] = trajectory_collide ? collision_cost_ : traj_cost;
  }

  // Normalize repulsive cost by trajectory length & lowest score to not overweight importance
  // This is a preferential cost, not collision cost, to be tuned relative to desired behaviors
  auto&& repulsive_cost_normalized = (repulsive_cost - xt::amin(repulsive_cost, immediate)) / traj_len;

  data.costs += xt::pow((weight_ * raw_cost) + (repulsion_weight_ * repulsive_cost_normalized), power_);
  data.fail_flag = all_trajectories_collide;
}

/**
 * @brief Checks if cost represents a collision
 * @param cost Costmap cost
 * @return bool if in collision
 */
bool ObstaclesCritic::inCollision(float cost) const
{
  using namespace costmap_2d;
  bool is_tracking_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  cost = cost < -1 ? NO_INFORMATION : cost == -1 ? LETHAL_OBSTACLE : static_cast<unsigned char>(cost);

  switch (static_cast<unsigned char>(cost))
  {
    case (LETHAL_OBSTACLE):
      return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
      return !consider_footprint_;
    case (NO_INFORMATION):
      return !is_tracking_unknown;
  }
  return false;
}

CollisionCost ObstaclesCritic::costAtPose(float x, float y, float theta)
{
  CollisionCost collision_cost;
  float& cost = collision_cost.cost;
  collision_cost.using_footprint = consider_footprint_;

  unsigned int x_i, y_i;
  if (!costmap_ros_->getCostmap()->worldToMap(x, y, x_i, y_i))
  {
    cost = costmap_2d::NO_INFORMATION;
    return collision_cost;
  }

  cost = world_model_->pointCost(x_i, y_i);
  if (consider_footprint_ && (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 0.0))
  {
    cost = static_cast<float>(world_model_->footprintCost(x, y, theta, costmap_ros_->getRobotFootprint()));
  }
  return collision_cost;
}

void ObstaclesCritic::reconfigureCB(mppi_controller::ObstacleCriticConfig& config, uint32_t level)
{
  consider_footprint_ = config.consider_footprint;
  collision_cost_ = config.collision_cost;
  collision_margin_distance_ = config.collision_margin_distance;
  near_goal_distance_ = config.near_goal_distance;
  repulsion_weight_ = config.repulsion_weight;
  CriticFunction::reconfigureCB(config, level);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ObstaclesCritic, mppi::critics::CriticBase)
