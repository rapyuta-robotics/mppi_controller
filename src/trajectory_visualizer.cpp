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
#include "mppi_controller/tools/trajectory_visualizer.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

  auto add_marker = [&](auto i) {
      float component = static_cast<float>(i) / static_cast<float>(size);

      auto pose = utils::createPose(trajectory(i, 0), trajectory(i, 1), 0.06);
      auto scale =
        i != size - 1 ?
        utils::createScale(0.03, 0.03, 0.07) :
        utils::createScale(0.07, 0.07, 0.09);
      auto color = utils::createColor(0, component, component, 1);
      auto marker = utils::createMarker(
        marker_id_++, pose, scale, color, frame_id_, marker_namespace);
      points_.markers.push_back(marker);
    };

  for (size_t i = 0; i < size; i++) {
    add_marker(i);
  }
}

void TrajectoryVisualizer::add(
  const models::Trajectories & trajectories, const std::string & marker_namespace)
{
  auto & shape = trajectories.x.shape();
  const float shape_1 = static_cast<float>(shape[1]);
  points_.markers.reserve(floor(shape[0] / trajectory_step_) * floor(shape[1] * time_step_));

  for (size_t i = 0; i < shape[0]; i += trajectory_step_) {
    for (size_t j = 0; j < shape[1]; j += time_step_) {
      const float j_flt = static_cast<float>(j);
      float blue_component = 1.0f - j_flt / shape_1;
      float green_component = j_flt / shape_1;

      auto pose = utils::createPose(trajectories.x(i, j), trajectories.y(i, j), 0.03);
      auto scale = utils::createScale(0.03, 0.03, 0.03);
      auto color = utils::createColor(0, green_component, blue_component, 1);
      auto marker = utils::createMarker(
        marker_id_++, pose, scale, color, frame_id_, marker_namespace);

      points_.markers.push_back(marker);
    }
  }
}

void TrajectoryVisualizer::reset()
{
  marker_id_ = 0;
  points_ = visualization_msgs::MarkerArray();
}

void TrajectoryVisualizer::visualize(const nav_msgs::Path& plan)
{
  if (trajectory_publisher_.getNumSubscribers() > 0)
  {
    trajectory_publisher_.publish(std::move(points_));
  }

  reset();

  if (transformed_path_pub_.getNumSubscribers() > 0)
  {
    transformed_path_pub_.publish(std::move(plan));
  }
}

}  // namespace mppi
