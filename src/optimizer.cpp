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

#include "mppi_controller/optimizer.hpp"

#include <mbf_msgs/ExePathResult.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void Optimizer::initialize(const ros::NodeHandle& parent_nh, costmap_2d::Costmap2DROS* costmap_ros,
                           const mppi_controller::MPPIControllerConfig& config)
{
  parent_nh_ = parent_nh;

  costmap_ros_ = costmap_ros;

  critic_manager_.on_configure(parent_nh_, costmap_ros_);

  models::OptimizerSettings default_settings;
  noise_generator_.initialize(parent_nh_, default_settings, false);
  setParams(config);
}

void Optimizer::shutdown()
{
}

void Optimizer::setParams(const mppi_controller::MPPIControllerConfig& config)
{
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    auto& s = settings_;
    s.model_dt = config.model_dt;
    s.time_steps = config.time_steps;
    s.batch_size = config.batch_size;
    s.iteration_count = config.iteration_count;
    s.temperature = config.temperature;
    s.gamma = config.gamma;
    s.retry_attempt_limit = config.retry_attempt_limit;
    s.base_constraints.vx_max = config.vx_max;
    s.base_constraints.vx_min = config.vx_min;
    s.base_constraints.vy = config.vy_max;
    s.base_constraints.wz = config.wz_max;
    s.sampling_std.vx = config.vx_std;
    s.sampling_std.vy = config.vy_std;
    s.sampling_std.wz = config.wz_std;
    s.constraints = s.base_constraints;

    std_dev_ = { s.sampling_std.wz, config.__getMax__().wz_std, std::hypot(config.vx_max, config.vy_max) };

    ROS_DEBUG_NAMED("Optimizer",
                    "Updating setting params: model_dt: %f, time_steps: %d, batch_size: %d, "
                    "iteration_count: %d, temperature: %f, gamma: %f, retry_attempt_limit: %d, "
                    "vx_max: %f, vx_min: %f, vy_max: %f, wz_max: %f, vx_std: %f, vy_std: %f, "
                    "wz_std: %f",
                    s.model_dt, s.time_steps, s.batch_size, s.iteration_count, s.temperature, s.gamma,
                    s.retry_attempt_limit, s.base_constraints.vx_max, s.base_constraints.vx_min, s.base_constraints.vy,
                    s.base_constraints.wz, s.sampling_std.vx, s.sampling_std.vy, s.sampling_std.wz);
  }

  setMotionModel(config.motion_model);
  setOffset(config.controller_frequency);
  reset();
}

void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
  constexpr double eps = 1e-6;

  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    if ((controller_period + eps) < settings_.model_dt)
    {
      ROS_WARN("Controller period is less then model dt, consider setting it equal");
    }
    else if (abs(controller_period - settings_.model_dt) < eps)
    {
      ROS_INFO("Controller period is equal to model dt. Control sequence shifting is ON");
      settings_.shift_control_sequence = true;
    }
    else
    {
      ROS_WARN("Controller period is more then model dt, we ignore and consider it equal");
      settings_.shift_control_sequence = true;
    }
  }
}

void Optimizer::reset()
{
  mppi::models::OptimizerSettings settings_copy;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    settings_copy = settings_;
  }

  state_.reset(settings_copy.batch_size, settings_copy.time_steps);
  control_sequence_.reset(settings_copy.time_steps);
  control_history_[0] = { 0.0, 0.0, 0.0 };
  control_history_[1] = { 0.0, 0.0, 0.0 };
  control_history_[2] = { 0.0, 0.0, 0.0 };
  control_history_[3] = { 0.0, 0.0, 0.0 };

  costs_ = xt::zeros<float>({ settings_copy.batch_size });
  generated_trajectories_.reset(settings_copy.batch_size, settings_copy.time_steps);

  critic_manager_.updateConstraints(settings_copy.constraints);
  ROS_INFO("Optimizer reset");
}

uint32_t Optimizer::evalControl(const geometry_msgs::PoseStamped& robot_pose, const geometry_msgs::Twist& robot_speed,
                                const nav_msgs::Path& plan, geometry_msgs::TwistStamped& cmd_vel)
{
  prepare(robot_pose, robot_speed, plan);

  while (true)
  {
    optimize();

    if (uint32_t error = mbf_msgs::ExePathResult::SUCCESS; !fallback(critics_data_.fail_flag, error))
    {
      if (error != mbf_msgs::ExePathResult::SUCCESS)
      {
        return error;
      }
      break;
    }
  }

  mppi::models::OptimizerSettings settings;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    settings = settings_;
  }

  utils::savitskyGolayFilter(control_sequence_, control_history_, settings);
  cmd_vel = getControlFromSequenceAsTwist(plan.header);

  if (settings.shift_control_sequence)
  {
    shiftControlSequence();
  }

  return mbf_msgs::ExePathResult::SUCCESS;
}

void Optimizer::optimize()
{
  int iteration_count;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    iteration_count = settings_.iteration_count;
  }

  for (size_t i = 0; i < iteration_count; ++i)
  {
    generateNoisedTrajectories();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    updateControlSequence();
  }
}

bool Optimizer::fallback(bool fail, uint32_t& error)
{
  static size_t counter = 0;

  if (!fail)
  {
    counter = 0;
    return false;
  }

  reset();

  int retry_attempt_limit;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    retry_attempt_limit = settings_.retry_attempt_limit;
  }

  if (++counter > retry_attempt_limit)
  {
    counter = 0;
    ROS_ERROR_NAMED("Optimizer", "Optimizer fail to compute path");
    error = mbf_msgs::ExePathResult::NO_VALID_CMD;
    return false;
  }

  return true;
}

void Optimizer::prepare(const geometry_msgs::PoseStamped& robot_pose, const geometry_msgs::Twist& robot_speed,
                        const nav_msgs::Path& plan)
{
  state_.pose = robot_pose;
  state_.speed = robot_speed;
  path_ = utils::toTensor(plan);
  costs_.fill(0);

  critics_data_.fail_flag = false;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
  scaleStdDeviation(robot_speed, 1.0);
}

void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT
  control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
  control_sequence_.wz = xt::roll(control_sequence_.wz, -1);

  xt::view(control_sequence_.vx, -1) = xt::view(control_sequence_.vx, -2);

  xt::view(control_sequence_.wz, -1) = xt::view(control_sequence_.wz, -2);

  if (isHolonomic())
  {
    control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
    xt::view(control_sequence_.vy, -1) = xt::view(control_sequence_.vy, -2);
  }
}

void Optimizer::scaleStdDeviation(const geometry_msgs::Twist& robot_speed, double scaling_factor)
{
  scaling_factor = std::clamp(scaling_factor, 0.0, 1.0);
  if (scaling_factor == 0.0)
  {
    return;
  }

  double max_wz_std = std::clamp(scaling_factor * std_dev_.max_wz_std, std_dev_.wz_std, std_dev_.max_wz_std);
  const double speed = std::hypot(robot_speed.linear.x, robot_speed.linear.y);

  /*
  // logistic function
  const double wz_std = max_wz_std + (std_dev_.wz_std - max_wz_std) / (1 + exp(2 * -(speed - std_dev_.max_speed / 2)));
  */

  const double wz_std = speed < 0.1 ? max_wz_std : std_dev_.wz_std;

  ROS_DEBUG_NAMED("Optimizer", "Scaling std deviation: wz_std: %f", wz_std);
  // settings_.sampling_std.wz = wz_std;
  noise_generator_.generateNoisedControls(settings_, isHolonomic());
}

void Optimizer::generateNoisedTrajectories()
{
  noise_generator_.setNoisedControls(state_, control_sequence_);
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

bool Optimizer::isHolonomic() const
{
  return motion_model_->isHolonomic();
}

void Optimizer::applyControlSequenceConstraints()
{
  mppi::models::ControlConstraints constraints;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    constraints = settings_.constraints;
  }

  if (isHolonomic())
  {
    control_sequence_.vy = xt::clip(control_sequence_.vy, -constraints.vy, constraints.vy);
  }

  control_sequence_.vx = xt::clip(control_sequence_.vx, constraints.vx_min, constraints.vx_max);
  control_sequence_.wz = xt::clip(control_sequence_.wz, -constraints.wz, constraints.wz);

  motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(models::State& state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(models::State& state) const
{
  xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
  xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

  if (isHolonomic())
  {
    xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(models::State& state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(xt::xtensor<float, 2>& trajectory, const xt::xtensor<float, 2>& sequence) const
{
  double model_dt;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    model_dt = settings_.model_dt;
  }

  float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  const auto vx = xt::view(sequence, xt::all(), 0);
  const auto vy = xt::view(sequence, xt::all(), 2);
  const auto wz = xt::view(sequence, xt::all(), 1);

  auto traj_x = xt::view(trajectory, xt::all(), 0);
  auto traj_y = xt::view(trajectory, xt::all(), 1);
  auto traj_yaws = xt::view(trajectory, xt::all(), 2);

  xt::noalias(traj_yaws) = xt::cumsum(wz * model_dt, 0) + initial_yaw;

  auto&& yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
  auto&& yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

  const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

  xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
  xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

  auto&& dx = xt::eval(vx * yaw_cos);
  auto&& dy = xt::eval(vx * yaw_sin);

  if (isHolonomic())
  {
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * model_dt, 0);
  xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * model_dt, 0);
}

void Optimizer::integrateStateVelocities(models::Trajectories& trajectories, const models::State& state) const
{
  double model_dt;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    model_dt = settings_.model_dt;
  }

  const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

  xt::noalias(trajectories.yaws) = xt::cumsum(state.wz * model_dt, 1) + initial_yaw;

  const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

  auto&& yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  auto&& yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
  xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

  auto&& dx = xt::eval(state.vx * yaw_cos);
  auto&& dy = xt::eval(state.vx * yaw_sin);

  if (isHolonomic())
  {
    dx = dx - state.vy * yaw_sin;
    dy = dy + state.vy * yaw_cos;
  }

  xt::noalias(trajectories.x) = state.pose.pose.position.x + xt::cumsum(dx * model_dt, 1);
  xt::noalias(trajectories.y) = state.pose.pose.position.y + xt::cumsum(dy * model_dt, 1);
}

xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
  int time_steps;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    time_steps = settings_.time_steps;
  }

  auto&& sequence =
      xt::xtensor<float, 2>::from_shape({ static_cast<long unsigned int>(time_steps), isHolonomic() ? 3u : 2u });
  auto&& trajectories = xt::xtensor<float, 2>::from_shape({ static_cast<long unsigned int>(time_steps), 3 });

  xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
  xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

  if (isHolonomic())
  {
    xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
  mppi::models::OptimizerSettings settings_copy;
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    settings_copy = settings_;
  }

  auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
  auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
  xt::noalias(costs_) +=
      settings_copy.gamma / powf(settings_copy.sampling_std.vx, 2) *
      xt::sum(xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
  xt::noalias(costs_) +=
      settings_copy.gamma / powf(settings_copy.sampling_std.wz, 2) *
      xt::sum(xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

  if (isHolonomic())
  {
    auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
    xt::noalias(costs_) +=
        settings_copy.gamma / powf(settings_copy.sampling_std.vy, 2) *
        xt::sum(xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy, 1, immediate);
  }

  auto&& costs_normalized = costs_ - xt::amin(costs_, immediate);
  auto&& exponents = xt::eval(xt::exp(-1 / settings_copy.temperature * costs_normalized));
  auto&& softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  auto&& softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

  xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
  if (isHolonomic())
  {
    xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  }

  applyControlSequenceConstraints();
}

geometry_msgs::TwistStamped Optimizer::getControlFromSequenceAsTwist(const std_msgs::Header& header)
{
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  if (isHolonomic())
  {
    auto vy = control_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, header);
  }

  return utils::toTwistStamped(vx, wz, header);
}

void Optimizer::setMotionModel(const int model)
{
  switch (model)
  {
    case mppi_controller::MPPIController_DiffDrive:
      motion_model_ = std::make_shared<DiffDriveMotionModel>();
      break;
    case mppi_controller::MPPIController_Omni:
      motion_model_ = std::make_shared<OmniMotionModel>();
      break;
    case mppi_controller::MPPIController_Ackermann:
      motion_model_ = std::make_shared<AckermannMotionModel>(parent_nh_);
      break;
    default:
      ROS_WARN_NAMED("Optimizer",
                     "Model %d is not valid! Valid options are DiffDrive, Omni, or Ackermann; defaulting to DiffDrive",
                     model);
      motion_model_ = std::make_shared<DiffDriveMotionModel>();
      break;
  }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  auto& s = settings_;
  if (speed_limit == 0.0)
  {
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  }
  else
  {
    if (percentage)
    {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
    else
    {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

models::Trajectories& Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
