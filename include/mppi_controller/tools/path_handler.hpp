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

#ifndef MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
#define MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_

#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "tf2_ros/buffer.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "mppi_controller/MPPIControllerConfig.h"

namespace mppi
{

using PathIterator = std::vector<geometry_msgs::PoseStamped>::iterator;
using PathRange = std::pair<PathIterator, PathIterator>;

/**
 * @class mppi::PathHandler
 * @brief Manager of incoming reference paths for transformation and processing
 */

class PathHandler
{
public:
  /**
   * @brief Constructor for mppi::PathHandler
   */
  PathHandler() = default;

  /**
   * @brief Destructor for mppi::PathHandler
   */
  ~PathHandler() = default;

  /**
   * @brief Initialize path handler on bringup
   * @param parent_nh Private parent node handle
   * @param costmap_ros Costmap2DROS object of environment
   * @param tf TF buffer for transformations
   */
  void initialize(const ros::NodeHandle& parent_nh, costmap_2d::Costmap2DROS*, tf2_ros::Buffer*);

  /**
   * @brief Set new reference path
   * @param Plan Path to use
   */
  void setPath(const nav_msgs::Path& plan);

  /**
   * @brief Get reference path
   * @return Path
   */
  nav_msgs::Path& getPath();

  /**
   * @brief transform global plan to local applying constraints,
   * then prune global plan
   * @param robot_pose Pose of robot
   * @param[out] local_plan Local plan to be filled
   * @return mbf_msgs::ExePathResult::SUCCESS if successful, otherwise failure
   */
  uint32_t transformPath(const geometry_msgs::PoseStamped& robot_pose, nav_msgs::Path& local_plan);

  /**
   * @brief Set parameters for path handler
   * @param config Dynamic reconfigure config
   */
  void setParams(const mppi_controller::MPPIControllerConfig& config);

protected:
  /**
   * @brief Transform a pose to another frame
   * @param frame Frame to transform to
   * @param in_pose Input pose
   * @param out_pose Output pose
   * @return Bool if successful
   */
  bool transformPose(const std::string& frame, const geometry_msgs::PoseStamped& in_pose,
                     geometry_msgs::PoseStamped& out_pose) const;

  /**
   * @brief Get largest dimension of costmap (radially)
   * @return Max distance from center of costmap to edge
   */
  double getMaxCostmapDist();

  /**
   * @brief Transform a pose to the global reference frame
   * @param[in|out] pose to be transformed
   * @return mbf_msgs::ExePathResult::SUCCESS if successful, otherwise failure
   */
  uint32_t transformToGlobalPlanFrame(geometry_msgs::PoseStamped& pose);

  /**
   * @brief Get global plan within window of the local costmap size
   * @param global_pose Robot pose
   * @return plan transformed in the costmap frame and iterator to the first pose of the global
   * plan (for pruning)
   */
  std::pair<nav_msgs::Path, PathIterator>
  getGlobalPlanConsideringBoundsInCostmapFrame(const geometry_msgs::PoseStamped& global_pose);

  /**
   * @brief Prune a path to only interesting portions
   * @param plan Plan to prune
   * @param end Final path iterator
   */
  void prunePlan(nav_msgs::Path& plan, const PathIterator end);

  /**
   * @brief Check if the robot pose is within the set inversion tolerances
   * @param robot_pose Robot's current pose to check
   * @return bool If the robot pose is within the set inversion tolerances
   */
  bool isWithinInversionTolerances(const geometry_msgs::PoseStamped& robot_pose) const;

  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer* tf_buffer_;
  ros::NodeHandle parent_nh_;
  ros::NodeHandle pnh_;

  nav_msgs::Path global_plan_;
  nav_msgs::Path global_plan_up_to_inversion_;

  mppi_controller::MPPIControllerConfig config_;
  std::atomic<unsigned int> inversion_locale_{ 0u };
};
}  // namespace mppi

#endif  // MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
