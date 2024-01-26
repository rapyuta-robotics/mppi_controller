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

  (*critics_data_.writeAccess()).fail_flag = false;

  (*critic_manager_.writeAccess()).on_configure(parent_nh_, costmap_ros_);

  models::OptimizerSettings default_settings;
  (*noise_generator_.writeAccess()).initialize(parent_nh_, default_settings, false);
  setParams(config);
}

void Optimizer::shutdown()
{
  (*noise_generator_.writeAccess()).shutdown();
}

void Optimizer::setParams(const mppi_controller::MPPIControllerConfig& config)
{
  (*settings_.writeAccess()).model_dt = config.model_dt;
  (*settings_.writeAccess()).time_steps = config.time_steps;
  (*settings_.writeAccess()).batch_size = config.batch_size;
  (*settings_.writeAccess()).iteration_count = config.iteration_count;
  (*settings_.writeAccess()).temperature = config.temperature;
  (*settings_.writeAccess()).gamma = config.gamma;
  (*settings_.writeAccess()).retry_attempt_limit = config.retry_attempt_limit;
  (*settings_.writeAccess()).base_constraints.vx_max = config.vx_max;
  (*settings_.writeAccess()).base_constraints.vx_min = config.vx_min;
  (*settings_.writeAccess()).base_constraints.vy = config.vy_max;
  (*settings_.writeAccess()).base_constraints.wz = config.wz_max;
  (*settings_.writeAccess()).sampling_std.vx = config.vx_std;
  (*settings_.writeAccess()).sampling_std.vy = config.vy_std;
  (*settings_.writeAccess()).sampling_std.wz = config.wz_std;

  const auto base_constraints = (*settings_.readAccess()).base_constraints;
  (*settings_.writeAccess()).constraints = base_constraints;

  ROS_DEBUG_NAMED("Optimizer",
                  "Updating setting params: model_dt: %f, time_steps: %d, batch_size: %d, "
                  "iteration_count: %d, temperature: %f, gamma: %f, retry_attempt_limit: %d, "
                  "vx_max: %f, vx_min: %f, vy_max: %f, wz_max: %f, vx_std: %f, vy_std: %f, "
                  "wz_std: %f",
                  config.model_dt, config.time_steps, config.batch_size, config.iteration_count, config.temperature,
                  config.gamma, config.retry_attempt_limit, config.vx_max, config.vx_min, config.vy_max, config.wz_max,
                  config.vx_std, config.vy_std, config.wz_std);

  setMotionModel(config.motion_model);
  setOffset(config.controller_frequency);
  reset();

  (*noise_generator_.writeAccess()).setParams(config);
}

void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
  const double model_dt = (*settings_.readAccess()).model_dt;
  constexpr double eps = 1e-6;

  if ((controller_period + eps) < model_dt)
  {
    ROS_WARN("Controller period is less then model dt, consider setting it equal");
  }
  else if (abs(controller_period - model_dt) < eps)
  {
    ROS_INFO("Controller period is equal to model dt. Control sequence shifting is ON");
    (*settings_.writeAccess()).shift_control_sequence = true;
  }
    else
    {
      ROS_WARN("Controller period is more then model dt, we ignore and consider it equal");
      (*settings_.writeAccess()).shift_control_sequence = true;
    }
}

void Optimizer::reset()
{
  const int batch_size = (*settings_.readAccess()).batch_size;
  const int time_steps = (*settings_.readAccess()).time_steps;
  const auto constraints = (*settings_.readAccess()).constraints;
  (*state_.writeAccess())->reset(batch_size, time_steps);
  (*control_sequence_.writeAccess()).reset(time_steps);

  (*control_history_.writeAccess())[0] = { 0.0, 0.0, 0.0 };
  (*control_history_.writeAccess())[1] = { 0.0, 0.0, 0.0 };
  (*control_history_.writeAccess())[2] = { 0.0, 0.0, 0.0 };
  (*control_history_.writeAccess())[3] = { 0.0, 0.0, 0.0 };

  *(*costs_.writeAccess()) = xt::zeros<float>({ batch_size });

  (*generated_trajectories_.writeAccess())->reset(batch_size, time_steps);
  (*noise_generator_.writeAccess()).reset(batch_size, time_steps, isHolonomic());
  (*critic_manager_.writeAccess()).updateConstraints(constraints);
  ROS_INFO("Optimizer reset");
}

uint32_t Optimizer::evalControl(const geometry_msgs::PoseStamped& robot_pose, const geometry_msgs::Twist& robot_speed,
                                const nav_msgs::Path& plan, geometry_msgs::TwistStamped& cmd_vel)
{
  prepare(robot_pose, robot_speed, plan);

  while (true)
  {
    optimize();

    const bool fail_flag = (*critics_data_.readAccess()).fail_flag;
    if (uint32_t error = mbf_msgs::ExePathResult::SUCCESS; !fallback(fail_flag, error))
    {
      if (error != mbf_msgs::ExePathResult::SUCCESS)
      {
        return error;
      }
      break;
    }
  }

  utils::savitskyGolayFilter((*control_sequence_.writeAccess()), (*control_history_.writeAccess()),
                             (*settings_.readAccess()));

  cmd_vel = getControlFromSequenceAsTwist(plan.header);

  if ((*settings_.readAccess()).shift_control_sequence)
  {
    shiftControlSequence();
  }

  return mbf_msgs::ExePathResult::SUCCESS;
}

void Optimizer::optimize()
{
  const int iteration_count = (*settings_.readAccess()).iteration_count;

  for (size_t i = 0; i < iteration_count; ++i)
  {
    generateNoisedTrajectories();
    (*critic_manager_.writeAccess()).evalTrajectoriesScores(*critics_data_.writeAccess());
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

  const int retry_attempt_limit = (*settings_.readAccess()).retry_attempt_limit;

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
  (*state_.writeAccess())->pose = robot_pose;
  (*state_.writeAccess())->speed = robot_speed;

  path_ = utils::toTensor(plan);

  (*costs_.writeAccess())->fill(0);
  (*critics_data_.writeAccess()).fail_flag = false;
  (*critics_data_.writeAccess()).furthest_reached_path_point.reset();
  (*critics_data_.writeAccess()).path_pts_valid.reset();
}

void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT

  {
    const auto vx = xt::roll((*control_sequence_.readAccess()).vx, -1);
    const auto wz = xt::roll((*control_sequence_.readAccess()).wz, -1);
    (*control_sequence_.writeAccess()).vx = vx;
    (*control_sequence_.writeAccess()).wz = wz;
  }

  {
    const auto vx = xt::roll((*control_sequence_.readAccess()).vx, -2);
    const auto wz = xt::roll((*control_sequence_.readAccess()).wz, -2);
    xt::view((*control_sequence_.writeAccess()).vx, -1) = vx;
    xt::view((*control_sequence_.writeAccess()).wz, -1) = wz;
  }

  if (isHolonomic())
  {
    const auto vy = xt::roll((*control_sequence_.readAccess()).vy, -1);
    const auto vy2 = xt::roll((*control_sequence_.readAccess()).vy, -2);
    (*control_sequence_.writeAccess()).vy = vy;
    xt::view((*control_sequence_.writeAccess()).vy, -1) = vy2;
  }
}

void Optimizer::generateNoisedTrajectories()
{
  (*noise_generator_.writeAccess()).setNoisedControls(*(*state_.writeAccess()), *control_sequence_.readAccess());
  (*noise_generator_.writeAccess()).generateNextNoises();
  updateStateVelocities(*(*state_.writeAccess()));
  integrateStateVelocities(*(*generated_trajectories_.writeAccess()), *(*state_.writeAccess()));
}

bool Optimizer::isHolonomic() const
{
  return (*motion_model_.readAccess())->isHolonomic();
}

void Optimizer::applyControlSequenceConstraints()
{
  mppi::models::ControlConstraints constraints = (*settings_.readAccess()).constraints;
  const auto vy = (*control_sequence_.readAccess()).vy;
  const auto vx = (*control_sequence_.readAccess()).vx;
  const auto wz = (*control_sequence_.readAccess()).wz;

  if (isHolonomic())
  {
    (*control_sequence_.writeAccess()).vy = xt::clip(vy, -constraints.vy, constraints.vy);
  }

  (*control_sequence_.writeAccess()).vx = xt::clip(vx, constraints.vx_min, constraints.vx_max);
  (*control_sequence_.writeAccess()).wz = xt::clip(wz, -constraints.wz, constraints.wz);
  (*motion_model_.writeAccess())->applyConstraints(*control_sequence_.writeAccess());
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
  (*motion_model_.readAccess())->predict(state);
}

void Optimizer::integrateStateVelocities(xt::xtensor<float, 2>& trajectory, const xt::xtensor<float, 2>& sequence) const
{
  const double model_dt = (*settings_.readAccess()).model_dt;
  const geometry_msgs::PoseStamped& pose = (*state_.readAccess())->pose;

  float initial_yaw = tf2::getYaw(pose.pose.orientation);

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

  xt::noalias(traj_x) = pose.pose.position.x + xt::cumsum(dx * model_dt, 0);
  xt::noalias(traj_y) = pose.pose.position.y + xt::cumsum(dy * model_dt, 0);
}

void Optimizer::integrateStateVelocities(models::Trajectories& trajectories, const models::State& state) const
{
  const double model_dt = (*settings_.readAccess()).model_dt;
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
  const int time_steps = (*settings_.readAccess()).time_steps;
  const auto vx = (*control_sequence_.readAccess()).vx;
  const auto vy = (*control_sequence_.readAccess()).vy;
  const auto wz = (*control_sequence_.readAccess()).wz;

  auto&& sequence =
      xt::xtensor<float, 2>::from_shape({ static_cast<long unsigned int>(time_steps), isHolonomic() ? 3u : 2u });
  auto&& trajectories = xt::xtensor<float, 2>::from_shape({ static_cast<long unsigned int>(time_steps), 3 });

  xt::noalias(xt::view(sequence, xt::all(), 0)) = vx;
  xt::noalias(xt::view(sequence, xt::all(), 1)) = wz;

  if (isHolonomic())
  {
    xt::noalias(xt::view(sequence, xt::all(), 2)) = vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
  const mppi::models::SamplingStd sampling_std = (*settings_.readAccess()).sampling_std;
  const double temperature = (*settings_.readAccess()).temperature;
  const double gamma = (*settings_.readAccess()).gamma;
  const auto cvx = (*state_.readAccess())->cvx;
  const auto cvy = (*state_.readAccess())->cvy;
  const auto cwz = (*state_.readAccess())->cwz;
  const auto vx = (*control_sequence_.readAccess()).vx;
  const auto vy = (*control_sequence_.readAccess()).vy;
  const auto wz = (*control_sequence_.readAccess()).wz;

  const auto bounded_noises_vx = cvx - vx;
  const auto bounded_noises_wz = cwz - wz;

  xt::noalias(*costs_.writeAccess()) +=
      gamma / powf(sampling_std.vx, 2) *
      xt::sum(xt::view(vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
  xt::noalias(*costs_.writeAccess()) +=
      gamma / powf(sampling_std.wz, 2) *
      xt::sum(xt::view(wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

  if (isHolonomic())
  {
    const auto bounded_noises_vy = cvy - vy;
    xt::noalias(*costs_.writeAccess()) +=
        gamma / powf(sampling_std.vy, 2) *
        xt::sum(xt::view(vy, xt::newaxis(), xt::all()) * bounded_noises_vy, 1, immediate);
  }

  auto&& costs_normalized = *(*costs_.readAccess()) - xt::amin(*(*costs_.readAccess()), immediate);
  auto&& exponents = xt::eval(xt::exp(-1 / temperature * costs_normalized));
  auto&& softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  auto&& softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

  xt::noalias((*control_sequence_.writeAccess()).vx) = xt::sum(cvx * softmaxes_extened, 0, immediate);
  xt::noalias((*control_sequence_.writeAccess()).wz) = xt::sum(cwz * softmaxes_extened, 0, immediate);
  if (isHolonomic())
  {
    xt::noalias((*control_sequence_.writeAccess()).vy) = xt::sum(cvy * softmaxes_extened, 0, immediate);
  }

  applyControlSequenceConstraints();
}

geometry_msgs::TwistStamped Optimizer::getControlFromSequenceAsTwist(const std_msgs::Header& header)
{
  const unsigned int offset = (*settings_.readAccess()).shift_control_sequence ? 1 : 0;
  const auto vx = (*control_sequence_.readAccess()).vx(offset);
  const auto wz = (*control_sequence_.readAccess()).wz(offset);

  if (isHolonomic())
  {
    const auto vy = (*control_sequence_.readAccess()).vy(offset);
    return utils::toTwistStamped(vx, vy, wz, header);
  }
  return utils::toTwistStamped(vx, wz, header);
}

void Optimizer::setMotionModel(const int model)
{
  switch (model)
  {
    case mppi_controller::MPPIController_DiffDrive:
      (*motion_model_.writeAccess()) = std::make_shared<DiffDriveMotionModel>();
      break;
    case mppi_controller::MPPIController_Omni:
      (*motion_model_.writeAccess()) = std::make_shared<OmniMotionModel>();
      break;
    case mppi_controller::MPPIController_Ackermann:
      (*motion_model_.writeAccess()) = std::make_shared<AckermannMotionModel>(parent_nh_);
      break;
    default:
      ROS_WARN_NAMED("Optimizer",
                     "Model %d is not valid! Valid options are DiffDrive, Omni, or Ackermann; defaulting to DiffDrive",
                     model);
      (*motion_model_.writeAccess()) = std::make_shared<DiffDriveMotionModel>();
      break;
  }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  const auto vx_max = (*settings_.readAccess()).base_constraints.vx_max;
  const auto vx_min = (*settings_.readAccess()).base_constraints.vx_min;
  const auto vy = (*settings_.readAccess()).base_constraints.vy;
  const auto wz = (*settings_.readAccess()).base_constraints.wz;

  if (speed_limit == 0.0)
  {
    (*settings_.writeAccess()).constraints.vx_max = vx_max;
    (*settings_.writeAccess()).constraints.vx_min = vx_min;
    (*settings_.writeAccess()).constraints.vy = vy;
    (*settings_.writeAccess()).constraints.wz = wz;
  }
  else
  {
    if (percentage)
    {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      (*settings_.writeAccess()).constraints.vx_max = vx_max * ratio;
      (*settings_.writeAccess()).constraints.vx_min = vx_min * ratio;
      (*settings_.writeAccess()).constraints.vy = vy * ratio;
      (*settings_.writeAccess()).constraints.wz = wz * ratio;
    }
    else
    {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / vx_max;
      (*settings_.writeAccess()).constraints.vx_max = vx_max * ratio;
      (*settings_.writeAccess()).constraints.vx_min = vx_min * ratio;
      (*settings_.writeAccess()).constraints.vy = vy * ratio;
      (*settings_.writeAccess()).constraints.wz = wz * ratio;
    }
  }
}

models::Trajectories Optimizer::getGeneratedTrajectories()
{
  return *(generated_trajectories_.copy());
}

}  // namespace mppi
