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

#ifndef MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>
#include <string>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xview.hpp>

#include <dynamic_reconfigure/server.h>

#include "mppi_controller/AckermannConfig.h"
#include "mppi_controller/models/control_sequence.hpp"
#include "mppi_controller/models/state.hpp"
#include "mppi_controller/models/constraints.hpp"

namespace mppi {

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel {
 public:
  /**
   * @brief Constructor for mppi::MotionModel
   */
  MotionModel() = default;

  /**
   * @brief Destructor for mppi::MotionModel
   */
  virtual ~MotionModel() = default;

  /**
    * @brief Initialize motion model on bringup and set required variables
    * @param control_constraints Constraints on control
    * @param model_dt duration of a time step
    */
  void initialize(const models::ControlConstraints & control_constraints, float model_dt)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle
   * velocities
   */
  virtual void predict(models::State& state) {
    const bool is_holo = isHolonomic();
    double max_delta_vx = model_dt_ * control_constraints_.ax_max;
    double min_delta_vx = model_dt_ * control_constraints_.ax_min;
    double max_delta_vy = model_dt_ * control_constraints_.ay_max;
    double max_delta_wz = model_dt_ * control_constraints_.az_max;

    for (unsigned int i = 0; i != state.vx.shape(0); i++) {
      float vx_last = state.vx(i, 0);
      float vy_last = state.vy(i, 0);
      float wz_last = state.wz(i, 0);
      for (unsigned int j = 1; j != state.vx.shape(1); j++) {
        double cvx_curr = state.cvx(i, j - 1);
        cvx_curr = std::clamp(cvx_curr, vx_last + min_delta_vx, vx_last + max_delta_vx);
        state.vx(i, j) = cvx_curr;
        vx_last = cvx_curr;

        float & cwz_curr = state.cwz(i, j - 1);
        cwz_curr = std::clamp(cvx_curr, wz_last - max_delta_wz, wz_last + max_delta_wz);
        state.wz(i, j) = cwz_curr;
        wz_last = cwz_curr;

        if (is_holo) {
          float & cvy_curr = state.cvy(i, j - 1);
          cvy_curr = std::clamp(cvx_curr, vy_last - max_delta_vy, vy_last + max_delta_vy);
          state.vy(i, j) = cvy_curr;
          vy_last = cvy_curr;
        }
      }
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;


  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence& /*control_sequence*/) {
  }

  protected:
    float model_dt_{0.0};
  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f};
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel {
 public:
  /**
   * @brief Constructor for mppi::AckermannMotionModel
   */
   explicit AckermannMotionModel(const ros::NodeHandle& parent_nh)
   {
     ros::NodeHandle pnh(parent_nh, "ackermann");
     dsrv_ = std::make_unique<dynamic_reconfigure::Server<mppi_controller::AckermannConfig>>(pnh);
     dsrv_->setCallback(boost::bind(&AckermannMotionModel::reconfigureCB, this, _1, _2));
   }

   /**
    * @brief Whether the motion model is holonomic, using Y axis
    * @return Bool If holonomic
    */
   bool isHolonomic() override
   {
     return false;
   }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence& control_sequence) override {
    double min_turning_r;
    {
      std::lock_guard<std::mutex> lock(param_mtx_);
      min_turning_r = min_turning_r_;
    }

    auto& vx = control_sequence.vx;
    auto& wz = control_sequence.wz;

    auto view = xt::masked_view(wz, xt::fabs(vx) / xt::fabs(wz) < min_turning_r);
    view = xt::sign(wz) * vx / min_turning_r;
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius()
  {
    std::lock_guard<std::mutex> lock(param_mtx_);
    return static_cast<float>(min_turning_r_);
  }

 private:
   void reconfigureCB(const mppi_controller::AckermannConfig& config, uint32_t level)
   {
     std::lock_guard<std::mutex> lock(param_mtx_);
     min_turning_r_ = config.min_turning_r;
   }

  double min_turning_r_{0};
  std::mutex param_mtx_;
  std::unique_ptr<dynamic_reconfigure::Server<mppi_controller::AckermannConfig>> dsrv_;
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel {
 public:
  /**
   * @brief Constructor for mppi::DiffDriveMotionModel
   */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override { return false; }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel {
 public:
  /**
   * @brief Constructor for mppi::OmniMotionModel
   */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override { return true; }
};

}  // namespace mppi

#endif  // MPPI_CONTROLLER__MOTION_MODELS_HPP_
