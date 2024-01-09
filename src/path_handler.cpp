// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
// Copyright (c) 2023 Open Navigation LLC
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

#include "mppi_controller/tools/path_handler.hpp"
#include "mppi_controller/tools/utils.hpp"
#include <costmap_2d/costmap_2d_ros.h>
#include <mbf_utility/navigation_utility.h>
#include <mbf_msgs/ExePathResult.h>

namespace mppi
{

void PathHandler::initialize(const ros::NodeHandle& parent_nh, const std::string& name,
                             costmap_2d::Costmap2DROS* costmap, tf2_ros::Buffer* buffer)
{
  parent_nh_ = parent_nh;
  name_ = name;
  pnh_ = ros::NodeHandle(parent_nh_, name_);
  costmap_ros_ = costmap;
  tf_buffer_ = buffer;

  pnh_.param<double>("max_robot_pose_search_dist", max_robot_pose_search_dist_, getMaxCostmapDist());
  pnh_.param<double>("prune_distance", prune_distance_, 1.5);
  pnh_.param<double>("transform_tolerance", transform_tolerance_, 0.1);
  pnh_.param<bool>("enforce_path_inversion", enforce_path_inversion_, false);

  if (enforce_path_inversion_)
  {
    pnh_.param<double>("inversion_xy_tolerance", inversion_xy_tolerance_, 0.2);
    pnh_.param<double>("inversion_yaw_tolerance", inversion_yaw_tolerance, 0.4);
    inversion_locale_ = 0u;
  }
}

std::pair<nav_msgs::Path, PathIterator>
PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(const geometry_msgs::PoseStamped& global_pose)
{
  auto begin = global_plan_up_to_inversion_.poses.begin();

  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
      utils::firstAfterIntegratedDistance(global_plan_up_to_inversion_.poses.begin(),
                                          global_plan_up_to_inversion_.poses.end(), max_robot_pose_search_dist_);

  // Find closest point to the robot
  auto closest_point = utils::minBy(begin, closest_pose_upper_bound,
                                    [&global_pose](const geometry_msgs::PoseStamped& ps)
                                    { return mbf_utility::distance(global_pose, ps); });

  nav_msgs::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose.header.stamp;

  auto pruned_plan_end =
      utils::firstAfterIntegratedDistance(closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);

  unsigned int mx, my;
  // Find the furthest relevant pose on the path to consider within costmap
  // bounds
  // Transforming it to the costmap frame in the same loop
  for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
    ++global_plan_pose)
  {
    // Transform from global plan frame to costmap frame
    geometry_msgs::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = global_pose.header.stamp;
    global_plan_pose->header.frame_id = global_plan_.header.frame_id;
    transformPose(costmap_ros_->getGlobalFrameID(), *global_plan_pose, costmap_plan_pose);

    // Check if pose is inside the costmap
    if (!costmap_ros_->getCostmap()->worldToMap(costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y,
                                                mx, my))
    {
      return {transformed_plan, closest_point};
    }

    // Filling the transformed plan to return with the transformed pose
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  return {transformed_plan, closest_point};
}

uint32_t PathHandler::transformToGlobalPlanFrame(geometry_msgs::PoseStamped& pose)
{
  if (global_plan_up_to_inversion_.poses.empty()) {
    ROS_ERROR_NAMED("PathHandler", "Received plan with zero length");
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }

  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose, robot_pose)) {
    ROS_ERROR_NAMED("PathHandler", "Unable to transform robot pose into global plan's frame");
    return mbf_msgs::ExePathResult::TF_ERROR;
  }

  pose = robot_pose;
  return mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t PathHandler::transformPath(const geometry_msgs::PoseStamped& robot_pose, nav_msgs::Path& local_plan)
{
  // Find relevent bounds of path to use
  geometry_msgs::PoseStamped global_pose = robot_pose;
  if (uint32_t error = transformToGlobalPlanFrame(global_pose); error != mbf_msgs::ExePathResult::SUCCESS)
  {
    return error;
  }

  auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);

  prunePlan(global_plan_up_to_inversion_, lower_bound);

  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    if (isWithinInversionTolerances(global_pose)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
  }

  if (transformed_plan.poses.empty()) {
    ROS_ERROR_NAMED("PathHandler", "Resulting plan has 0 poses in it");
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }
  local_plan = transformed_plan;
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool PathHandler::transformPose(const std::string& frame, const geometry_msgs::PoseStamped& in_pose,
                                geometry_msgs::PoseStamped& out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(in_pose, out_pose, frame, ros::Duration(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    ROS_ERROR("Exception in transformPose: %s", ex.what());
  }
  return false;
}

double PathHandler::getMaxCostmapDist()
{
  const auto& costmap = costmap_ros_->getCostmap();
  return static_cast<double>(std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY())) *
         costmap->getResolution() * 0.50;
}

void PathHandler::setPath(const nav_msgs::Path& plan)
{
  global_plan_ = plan;
  global_plan_up_to_inversion_ = global_plan_;
  if (enforce_path_inversion_) {
    inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
}

nav_msgs::Path& PathHandler::getPath()
{
  return global_plan_;
}

void PathHandler::prunePlan(nav_msgs::Path& plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
}

bool PathHandler::isWithinInversionTolerances(const geometry_msgs::PoseStamped& robot_pose) const
{
  // Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_inversion_.poses.back();
  float distance = hypotf(
    robot_pose.pose.position.x - last_pose.pose.position.x,
    robot_pose.pose.position.y - last_pose.pose.position.y);

  float angle_distance = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),
    tf2::getYaw(last_pose.pose.orientation));

  return distance <= inversion_xy_tolerance_ && fabs(angle_distance) <= inversion_yaw_tolerance;
}

}  // namespace mppi
