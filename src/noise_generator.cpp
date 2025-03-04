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

#include "mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

void NoiseGenerator::shutdown()
{
  active_ = false;

  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }

  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::initialize()
{
  // Recompute the noises on reset, initialization, and fallback
  ROS_INFO_STREAM("Recomputing noises with batch size: " << settings_.batch_size
                                                         << " and time steps: " << settings_.time_steps);
  xt::noalias(noises_vx_) = xt::zeros<float>({ settings_.batch_size, settings_.time_steps });
  xt::noalias(noises_vy_) = xt::zeros<float>({ settings_.batch_size, settings_.time_steps });
  xt::noalias(noises_wz_) = xt::zeros<float>({ settings_.batch_size, settings_.time_steps });

  batch_size_ = settings_.batch_size;
  time_steps_ = settings_.time_steps;

  std::unique_lock<std::mutex> guard(noise_lock_);
  ready_ = true;
}

void NoiseGenerator::noiseThread()
{
  do {
    {
      std::unique_lock<std::mutex> guard(noise_lock_);
      noise_cond_.wait(guard, [this]() { return ready_; });
      ready_ = false;
    }

    if (active_)
    {
      generateNoisedControls();
    }
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto& s = settings_;

  if (batch_size_ != s.batch_size || time_steps_ != s.time_steps)
  {
    initialize();
  }

  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);
  if (is_holonomic_) {
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }
}

void NoiseGenerator::setParams(const mppi_controller::MPPIControllerConfig& config)
{
  if (config.regenerate_noises)
  {
    if (regenerate_noises_)
    {
      return;
    }
    regenerate_noises_ = true;
    active_ = true;
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
    return;
  }

  shutdown();
  generateNoisedControls();
}

}  // namespace mppi
