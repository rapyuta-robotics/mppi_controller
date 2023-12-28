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

#ifndef MPPI_CONTROLLER__CONTROLLER_HPP_
#define MPPI_CONTROLLER__CONTROLLER_HPP_

#include <ros/ros.h>

#include <memory>
#include <string>

#include "mppi_controller/models/constraints.hpp"
#include "mppi_controller/optimizer.hpp"
#include "mppi_controller/tools/path_handler.hpp"
#include "mppi_controller/tools/trajectory_visualizer.hpp"
#include "mppi_controller/tools/utils.hpp"
#include <mbf_costmap_core/costmap_controller.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <dynamic_reconfigure/server.h>
#include "mppi_controller/MPPIControllerConfig.h"

namespace mppi_controller
{

using namespace mppi;  // NOLINT

struct NavTolerances
{
  double xy_goal_tolerance;
  double yaw_goal_tolerance;
  double theta_stopped_vel;
  double trans_stopped_vel;
};

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */
class MPPIController : public mbf_costmap_core::CostmapController
{
private:
  static constexpr auto LOGNAME = "Controller";

public:
  /**
   * @brief Constructor for mppi::MPPIController
   */
  MPPIController() = default;
  ~MPPIController();

  /**
   * @brief Configure controller
   * @param name Name of plugin
   * @param tf TF buffer to use
   * @param costmap_ros Costmap2DROS object of environment
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * @brief Reset the controller state between tasks
   */
  void reset();

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param pose the current pose of the robot.
   * @param velocity the current velocity of the robot.
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
   * @param message Optional more detailed outcome as a string.
   * @return Result code as described on ExePath action result:
   *         SUCCESS         = 0
   *         1..9 are reserved as plugin specific non-error results
   *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
   *         CANCELED        = 101
   *         NO_VALID_CMD    = 102
   *         PAT_EXCEEDED    = 103
   *         COLLISION       = 104
   *         OSCILLATION     = 105
   *         ROBOT_STUCK     = 106
   *         MISSED_GOAL     = 107
   *         MISSED_PATH     = 108
   *         BLOCKED_PATH    = 109
   *         INVALID_PATH    = 110
   *         TF_ERROR        = 111
   *         NOT_INITIALIZED = 112
   *         INVALID_PLUGIN  = 113
   *         INTERNAL_ERROR  = 114
   *         121..149 are reserved as plugin specific errors
   */
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity,
                                   geometry_msgs::TwistStamped& cmd_vel, std::string& message) override;

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;

  bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

  void updateTolerances(double dist_tolerance, double angle_tolerance);

  bool isInitialized()
  {
    return initialized_;
  }

  /**
   * @brief Requests the planner to cancel; not implemented for this planner
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  bool cancel() override
  {
    return false;
  };

protected:
  /**
   * @brief Visualize trajectories
   * @param transformed_plan Transformed input plan
   */
  void visualize(nav_msgs::Path transformed_plan);

  void reconfigureCB(const mppi_controller::MPPIControllerConfig& config, uint32_t level);

  std::string name_;
  ros::NodeHandle parent_nh_;
  ros::NodeHandle pnh_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer* tf_buffer_;
  NavTolerances latest_limits_;
  base_local_planner::LocalPlannerUtil planner_util_;
  base_local_planner::OdometryHelperRos odom_helper_;

  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;
  std::unique_ptr<dynamic_reconfigure::Server<MPPIControllerConfig>> dsrv_;

  bool initialized_{ false };
  bool visualize_{ false };
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__CONTROLLER_HPP_
