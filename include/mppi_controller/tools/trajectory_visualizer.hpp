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

#ifndef MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
#define MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_

#include <ros/ros.h>

#include <memory>
#include <string>
#include <xtensor/xtensor.hpp>

#include "mppi_controller/models/trajectories.hpp"
#include "mppi_controller/tools/utils.hpp"

namespace mppi
{

/**
 * @class mppi::TrajectoryVisualizer
 * @brief Visualizes trajectories for debugging
 */
class TrajectoryVisualizer
{
public:
  /**
   * @brief Constructor for mppi::TrajectoryVisualizer
   */
  TrajectoryVisualizer() = default;

  /**
   * @brief Configure trajectory visualizer
   * @param parent_nh Parent node handle
   * @param name Name of plugin
   * @param frame_id Frame to publish trajectories in
   */
  void on_configure(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& frame_id);

  /**
   * @brief Add an optimal trajectory to visualize
   * @param trajectory Optimal trajectory
   */
  void add(const xt::xtensor<float, 2>& trajectory, const std::string& marker_namespace);

  /**
   * @brief Add candidate trajectories to visualize
   * @param trajectories Candidate trajectories
   */
  void add(const models::Trajectories& trajectories, const std::string& marker_namespace);

  /**
   * @brief Visualize the plan
   * @param plan Plan to visualize
   */
  void visualize(const nav_msgs::Path& plan);

  /**
   * @brief Reset object
   */
  void reset();

protected:
  std::string frame_id_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher transformed_path_pub_;

  visualization_msgs::MarkerArray points_;
  int marker_id_ = 0;

  int trajectory_step_{ 0 };
  int time_step_{ 0 };
};

}  // namespace mppi

#endif  // MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
