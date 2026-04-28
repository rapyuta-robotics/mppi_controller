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

#include <memory>
#include <algorithm>
#include <limits>
#include "mppi_controller/tools/trajectory_visualizer.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <xtensor/xmath.hpp>

namespace mppi
{

void TrajectoryVisualizer::on_configure(ros::NodeHandle& parent_nh, const std::string& frame_id)
{
  frame_id_ = frame_id;
  trajectory_publisher_ = parent_nh.advertise<visualization_msgs::MarkerArray>("trajectories", 1);
  transformed_path_pub_ = parent_nh.advertise<nav_msgs::Path>("transformed_global_plan", 1);
  reset();
}

void TrajectoryVisualizer::setParams(const mppi_controller::MPPIControllerConfig& config)
{
  trajectory_step_ = config.trajectory_step;
  time_step_ = config.time_step;
}

void TrajectoryVisualizer::add(
  const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace)
{
  auto & size = trajectory.shape()[0];
  if (!size) {
    return;
  }

  auto scale = utils::createScale(0.03, 0.0, 0.0);
  auto marker = utils::createMarker(0, scale, frame_id_, marker_namespace);
  for (size_t i = 0; i < size; i++) {
    float component = static_cast<float>(i) / static_cast<float>(size);
    marker.points.emplace_back(utils::createPose(trajectory(i, 0), trajectory(i, 1), 0.06).position);
    marker.colors.emplace_back(utils::createColor(0, component, component, 1));
  }
  markers_.markers.emplace_back(std::move(marker));
}

void TrajectoryVisualizer::add(
  const models::Trajectories & trajectories, const std::string & marker_namespace)
{
  auto & shape = trajectories.x.shape();
  const float shape_1 = static_cast<float>(shape[1]);
  markers_.markers.reserve(floor(shape[0] / trajectory_step_) * floor(shape[1] * time_step_));

  for (size_t i = 0; i < shape[0]; i += trajectory_step_) {
    auto scale = utils::createScale(0.02, 0.0, 0.0);
    auto marker = utils::createMarker(i, scale, frame_id_, marker_namespace);
    for (size_t j = 0; j < shape[1]; j += time_step_) {
      const float j_flt = static_cast<float>(j);
      float blue_component = 1.0f - j_flt / shape_1;
      float green_component = j_flt / shape_1;
      marker.points.emplace_back(utils::createPose(trajectories.x(i, j), trajectories.y(i, j), 0.03).position);
      marker.colors.emplace_back(utils::createColor(0, green_component, blue_component, 1));
    }
    markers_.markers.emplace_back(std::move(marker));
  }
}

void TrajectoryVisualizer::add(
  const models::Trajectories& trajectories, const xt::xtensor<float, 1>& costs,
  const std::vector<bool>& collisions, const std::string& marker_namespace)
{
  auto& shape = trajectories.x.shape();
  const size_t n_rows = shape[0];

  if (n_rows == 0 || costs.shape().empty() || costs.shape()[0] < n_rows)
  {
    return;
  }

  float min_val = std::numeric_limits<float>::max();
  float max_val = std::numeric_limits<float>::lowest();
  for (size_t i = 0; i < n_rows; ++i)
  {
    if (!collisions.empty() && i < collisions.size() && collisions[i])
    {
      continue;
    }

    const float curr_cost = costs(i);
    if (curr_cost < min_val)
    {
      min_val = curr_cost;
    }
    if (curr_cost > max_val)
    {
      max_val = curr_cost;
    }
  }

  if (max_val < min_val)
  {
    min_val = xt::amin(costs)();
    max_val = xt::amax(costs)();
  }

  const float range = max_val - min_val;
  for (size_t i = 0; i < n_rows; i += trajectory_step_)
  {
    const float norm = range > 0.0f ? (costs(i) - min_val) / range : 0.0f;
    const bool in_collision = !collisions.empty() && i < collisions.size() && collisions[i];
    addCostColoredTrajectory(i, trajectories, norm, in_collision, marker_namespace);
  }
}

void TrajectoryVisualizer::addCostColoredTrajectory(
  size_t trajectory_idx, const models::Trajectories& trajectories, float normalized_cost,
  bool in_collision, const std::string& marker_namespace)
{
  const size_t n_cols = trajectories.x.shape()[1];
  auto scale = utils::createScale(0.02, 0.0, 0.0);
  auto marker = utils::createMarker(static_cast<int>(trajectory_idx), scale, frame_id_, marker_namespace);

  marker.header.stamp = ros::Time::now();
  marker.color = in_collision ?
      utils::createColor(1.0f, 0.0f, 1.0f, 0.6f) :
      costToColor(normalized_cost);

  marker.points.reserve(n_cols / time_step_ + 1);
  for (size_t j = 0; j < n_cols; j += time_step_)
  {
    marker.points.emplace_back(utils::createPose(trajectories.x(trajectory_idx, j), trajectories.y(trajectory_idx, j), 0.03).position);
  }

  markers_.markers.emplace_back(std::move(marker));
}

std_msgs::ColorRGBA TrajectoryVisualizer::costToColor(float normalized_cost)
{
  normalized_cost = std::clamp(normalized_cost, 0.0f, 1.0f);
  float r;
  float g;

  if (normalized_cost < 0.5f)
  {
    r = 2.0f * normalized_cost;
    g = 1.0f;
  }
  else
  {
    r = 1.0f;
    g = 2.0f * (1.0f - normalized_cost);
  }

  return utils::createColor(r, g, 0.0f, 0.8f);
}

void TrajectoryVisualizer::reset()
{
  markers_ = visualization_msgs::MarkerArray();
}

void TrajectoryVisualizer::visualize(const nav_msgs::Path& plan)
{
  if (trajectory_publisher_.getNumSubscribers() > 0)
  {
    trajectory_publisher_.publish(std::move(markers_));
  }

  reset();

  if (transformed_path_pub_.getNumSubscribers() > 0)
  {
    transformed_path_pub_.publish(std::move(plan));
  }
}

}  // namespace mppi
