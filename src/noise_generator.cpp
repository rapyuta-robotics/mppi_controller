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

void NoiseGenerator::initialize(const ros::NodeHandle& parent_nh, mppi::models::OptimizerSettings& settings,
                                bool is_holonomic)
{
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::generateNoisedControls(const mppi::models::OptimizerSettings& settings, bool is_holonomic)
{
  xt::noalias(noises_vx_) = xt::zeros<float>({ settings.batch_size, settings.time_steps });
  xt::noalias(noises_vy_) = xt::zeros<float>({ settings.batch_size, settings.time_steps });
  xt::noalias(noises_wz_) = xt::zeros<float>({ settings.batch_size, settings.time_steps });

  xt::noalias(noises_vx_) =
      xt::random::randn<float>({ settings.batch_size, settings.time_steps }, 0.0f, settings.sampling_std.vx);
  xt::noalias(noises_wz_) =
      xt::random::randn<float>({ settings.batch_size, settings.time_steps }, 0.0f, settings.sampling_std.wz);

  if (is_holonomic)
  {
    xt::noalias(noises_vy_) =
        xt::random::randn<float>({ settings.batch_size, settings.time_steps }, 0.0f, settings.sampling_std.vy);
  }
}

}  // namespace mppi
