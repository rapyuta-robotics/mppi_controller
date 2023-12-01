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

#include <stdint.h>
#include <chrono>
#include "mppi_controller/controller.hpp"
#include "mppi_controller/tools/utils.hpp"
#include <mbf_msgs/ExePathResult.h>
#include <base_local_planner/goal_functions.h>

// #define BENCHMARK_TESTING

namespace mppi_controller
{

void MPPIController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  parent_nh_ = ros::NodeHandle("~");
  pnh_ = ros::NodeHandle(parent_nh_, name);
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;

  // Configure composed objects
  optimizer_.initialize(parent_nh_, name_, costmap_ros_, MPPIControllerConfig());
  path_handler_.initialize(parent_nh_, name_, costmap_ros_, tf_buffer_);
  planner_util_.initialize(tf, costmap_ros_->getCostmap(), costmap_ros_->getGlobalFrameID());

  dsrv_ = std::make_unique<dynamic_reconfigure::Server<MPPIControllerConfig>>(pnh_);
  dsrv_->setCallback(boost::bind(&MPPIController::reconfigureCB, this, _1, _2));

  std::string odom_topic;
  pnh_.param<std::string>("odom_topic", odom_topic, "odom");
  odom_helper_.setOdomTopic(odom_topic);

  initialized_ = true;

  trajectory_visualizer_.on_configure(parent_nh_, name_, costmap_ros_->getGlobalFrameID());

  ROS_INFO_NAMED("MPPIController", "Configured MPPI Controller: %s", name_.c_str());
}

MPPIController::~MPPIController()
{
  optimizer_.shutdown();
}

void MPPIController::reset()
{
  optimizer_.reset();
}

uint32_t MPPIController::computeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                                                 const geometry_msgs::TwistStamped& robot_speed,
                                                 geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif
  nav_msgs::Path transformed_plan;
  if (auto error = path_handler_.transformPath(robot_pose, transformed_plan); error != mbf_msgs::ExePathResult::SUCCESS)
  {
    return error;
  }

  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  if (uint32_t error = optimizer_.evalControl(robot_pose, robot_speed.twist, transformed_plan, cmd_vel);
      error != mbf_msgs::ExePathResult::SUCCESS)
  {
    return error;
  }

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_)
  {
    visualize(std::move(transformed_plan));
  }

  return mbf_msgs::ExePathResult::SUCCESS;
}

void MPPIController::visualize(nav_msgs::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

bool MPPIController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  nav_msgs::Path path;
  path.poses = orig_global_plan;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = ros::Time::now();
  path_handler_.setPath(path);
  return true;
}

bool MPPIController::isGoalReached(double dist_tolerance, double angle_tolerance)
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  geometry_msgs::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  updateTolerances(dist_tolerance, angle_tolerance);

  // copy over the odometry information
  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  // we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if (!planner_util_.getGoal(goal_pose))
  {
    return false;
  }

  // check to see if we've reached the goal position
  const double xy_goal_tolerance = planner_util_.getCurrentLimits().xy_goal_tolerance;
  if (mbf_utility::distance(goal_pose, robot_pose) <= xy_goal_tolerance)
  {
    ROS_DEBUG_NAMED("MPPI", "Goal position reached");

    const double yaw_goal_tolerance = planner_util_.getCurrentLimits().yaw_goal_tolerance;
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
    if (std::abs(angles::shortest_angular_distance(robot_yaw, goal_yaw)) <= yaw_goal_tolerance)
    {
      ROS_DEBUG_NAMED("MPPI", "Goal orientation reached");

      // make sure that we're actually stopped before returning success
      const double theta_stopped_vel = planner_util_.getCurrentLimits().theta_stopped_vel;
      const double trans_stopped_vel = planner_util_.getCurrentLimits().trans_stopped_vel;
      if (base_local_planner::stopped(base_odom, theta_stopped_vel, trans_stopped_vel))
      {
        ROS_DEBUG_NAMED("MPPI", "Goal reached and robot stopped");
        return true;
      }
      ROS_DEBUG_NAMED("MPPI", "Goal reached but robot still in motion");
      return false;
    }
    ROS_DEBUG_NAMED("MPPI", "Goal position reached, but orientation not reached yet");
    return false;
  }

  ROS_DEBUG_NAMED("MPPI", "Robot position (%.2f, %.2f) is still %.2fm away from goal (%.2f, %.2f)",
                  robot_pose.pose.position.x, robot_pose.pose.position.y, mbf_utility::distance(goal_pose, robot_pose),
                  goal_pose.pose.position.x, goal_pose.pose.position.y);
  return false;
}

void MPPIController::updateTolerances(double dist_tolerance, double angle_tolerance)
{
  bool reconfigure_needed = false;
  auto limits = planner_util_.getCurrentLimits();

  if (dist_tolerance > 0)
  {
    if (dist_tolerance != limits.xy_goal_tolerance)
    {
      // change limits if dist_tolerance is set, and different from the current values
      ROS_INFO_STREAM("updating xy_goal_tolerance to tolerance from action: " << dist_tolerance);
      limits.xy_goal_tolerance = dist_tolerance;
      reconfigure_needed = true;
    }
  }
  else if (latest_limits_.xy_goal_tolerance != limits.xy_goal_tolerance)
  {
    // change limits if dist_tolerance is not set, and different from latest values set by dynamic reconfigure
    ROS_INFO_STREAM(
        "updating xy_goal_tolerance to tolerance from previous dyn reconfig: " << latest_limits_.xy_goal_tolerance);
    limits.xy_goal_tolerance = latest_limits_.xy_goal_tolerance;
    reconfigure_needed = true;
  }

  if (angle_tolerance > 0)
  {
    if (angle_tolerance != limits.yaw_goal_tolerance)
    {
      // change limits if angle_tolerance is set, and different from the current values;
      ROS_INFO_STREAM("updating yaw_goal_tolerance to tolerance from action: " << angle_tolerance);
      limits.yaw_goal_tolerance = angle_tolerance;
      reconfigure_needed = true;
    }
  }
  else if (latest_limits_.yaw_goal_tolerance != limits.yaw_goal_tolerance)
  {
    // change limits if angle_tolerance is not set, and different from latest values set by dynamic reconfigure
    ROS_INFO_STREAM(
        "updating yaw_goal_tolerance to tolerance from previous dyn reconfig: " << latest_limits_.yaw_goal_tolerance);
    limits.yaw_goal_tolerance = latest_limits_.yaw_goal_tolerance;
    reconfigure_needed = true;
  }

  if (reconfigure_needed)
  {
    planner_util_.reconfigureCB(limits, false);
  }
}

void MPPIController::reconfigureCB(const mppi_controller::MPPIControllerConfig& config, uint32_t level)
{
  visualize_ = config.visualize;
  optimizer_.setParams(config);
}

}  // namespace mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi_controller::MPPIController, mbf_costmap_core::CostmapController)
