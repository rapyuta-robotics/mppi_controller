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

#ifndef MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_
#define MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_

#include <vector>

#include "mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class VelocityDeadbandCritic : public CriticFunction<>
{
public:
  void initialize() override;
  void score(CriticData& data) override;

protected:
  std::vector<float> deadband_velocities_{0.0f, 0.0f, 0.0f};
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_
