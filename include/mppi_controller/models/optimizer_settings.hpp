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

#ifndef MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
#define MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_

#include <cstddef>
#include "mppi_controller/models/constraints.hpp"

namespace mppi::models
{

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 */
struct OptimizerSettings
{
  models::ControlConstraints base_constraints{ 0, 0, 0, 0, 0 };
  models::ControlConstraints constraints{ 0, 0, 0, 0, 0};
  models::SamplingStd sampling_std{ 0, 0, 0 };
  double model_dt{ 0 };
  double temperature{ 0 };
  double gamma{ 0 };
  int batch_size{ 0 };
  int time_steps{ 0 };
  int iteration_count{ 0 };
  bool shift_control_sequence{ false };
  int retry_attempt_limit{ 0 };
};

}  // namespace mppi::models

#endif  // MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
