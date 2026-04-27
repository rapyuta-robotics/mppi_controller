#include <cmath>

#include "mppi_controller/critics/cost_critic.hpp"
#include "mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

void CostCritic::initialize()
{
  world_model_ = std::make_unique<base_local_planner::CostmapModel>(*costmap_ros_->getCostmap());
  pnh_.param("inflation_layer_name", inflation_layer_name_, std::string(""));
  possible_collision_cost_ = findCircumscribedCost(costmap_ros_);

  if (possible_collision_cost_ < 1.0f)
  {
    ROS_ERROR_NAMED(name_, "Inflation layer either not found or inflation is not set sufficiently for optimized non-circular collision checking capabilities.");
  }

  if (costmap_ros_->getUseRadius() == consider_footprint_)
  {
    ROS_WARN_NAMED(name_, "Inconsistent configuration in collision checking between costmap and CostCritic.");
  }

  if (near_collision_cost_ > 253u)
  {
    ROS_WARN_NAMED(name_, "Near collision cost is set higher than INSCRIBED_INFLATED_OBSTACLE.");
  }
}

float CostCritic::findCircumscribedCost(costmap_2d::Costmap2DROS* costmap)
{
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_)
  {
    return circumscribed_cost_;
  }

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
    break;
  }

  ROS_WARN_COND_NAMED(!inflation_layer_found, name_, "No inflation layer found in costmap configuration.");

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);
  return circumscribed_cost_;
}

bool CostCritic::inCollision(float cost, float x, float y, float theta) const
{
  float score_cost = cost;
  if (consider_footprint_ && (cost >= possible_collision_cost_ || possible_collision_cost_ < 1.0f))
  {
    score_cost = static_cast<float>(world_model_->footprintCost(x, y, theta, costmap_ros_->getRobotFootprint()));
  }

  score_cost = score_cost < -1 ? costmap_2d::NO_INFORMATION :
    score_cost == -1 ? costmap_2d::LETHAL_OBSTACLE : static_cast<unsigned char>(score_cost);

  switch (static_cast<unsigned char>(score_cost))
  {
    case costmap_2d::LETHAL_OBSTACLE:
      return true;
    case costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
      return !consider_footprint_;
    case costmap_2d::NO_INFORMATION:
      return !is_tracking_unknown_;
    default:
      return false;
  }
}

void CostCritic::score(CriticData& data)
{
  if (!enabled_)
  {
    return;
  }

  is_tracking_unknown_ = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  if (consider_footprint_)
  {
    possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
  }

  const bool near_goal = utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path);
  auto* costmap = costmap_ros_->getCostmap();
  auto traj_costs = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  traj_costs.fill(0.0f);
  bool all_trajectories_collide = true;

  auto& collisions = data.trajectories_in_collision;
  const bool track_collisions = !collisions.empty();

  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i)
  {
    bool trajectory_collide = false;
    float traj_cost = 0.0f;

    for (size_t j = 0; j < data.trajectories.x.shape(1); j += trajectory_point_step_)
    {
      const float tx = data.trajectories.x(i, j);
      const float ty = data.trajectories.y(i, j);
      const float tyaw = data.trajectories.yaws(i, j);
      unsigned int mx = 0u;
      unsigned int my = 0u;
      float pose_cost = 255.0f;

      if (costmap->worldToMap(tx, ty, mx, my))
      {
        pose_cost = static_cast<float>(costmap->getCost(mx, my));
        if (pose_cost < 1.0f)
        {
          continue;
        }
      }

      if (inCollision(pose_cost, tx, ty, tyaw))
      {
        traj_cost = collision_cost_;
        trajectory_collide = true;
        if (track_collisions && i < collisions.size())
        {
          collisions[i] = true;
        }
        break;
      }

      if (pose_cost >= static_cast<float>(near_collision_cost_))
      {
        traj_cost += critical_cost_;
      }
      else if (!near_goal)
      {
        traj_cost += pose_cost;
      }
    }

    all_trajectories_collide &= trajectory_collide;
    traj_costs(i) = traj_cost;
  }

  const float cost_scale = static_cast<float>(weight_) /
    static_cast<float>(std::max<size_t>(1u, data.trajectories.x.shape(1) / trajectory_point_step_ + 1));
  if (power_ > 1.0)
  {
    data.costs += xt::pow(traj_costs * cost_scale, power_);
  }
  else
  {
    data.costs += traj_costs * cost_scale;
  }

  data.fail_flag = all_trajectories_collide;
}

void CostCritic::reconfigureCB(mppi_controller::CostCriticConfig& config, uint32_t level)
{
  consider_footprint_ = config.consider_footprint;
  critical_cost_ = config.critical_cost;
  near_collision_cost_ = config.near_collision_cost;
  collision_cost_ = config.collision_cost;
  near_goal_distance_ = config.near_goal_distance;
  trajectory_point_step_ = config.trajectory_point_step;
  CriticFunction::reconfigureCB(config, level);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::CostCritic, mppi::critics::CriticBase)
