#include <algorithm>
#include <cmath>

#include "mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{

void VelocityDeadbandCritic::initialize()
{
  std::vector<double> deadband_velocities;
  pnh_.param("deadband_velocities", deadband_velocities, std::vector<double>{0.0, 0.0, 0.0});
  deadband_velocities_.assign(3u, 0.0f);
  for (size_t i = 0; i < std::min<size_t>(3u, deadband_velocities.size()); ++i)
  {
    deadband_velocities_[i] = static_cast<float>(deadband_velocities[i]);
  }

  ROS_INFO_STREAM_NAMED(
    name_, "VelocityDeadbandCritic instantiated with " << power_ << " power, " << weight_
      << " weight, deadband_velocity [" << deadband_velocities_[0] << ","
      << deadband_velocities_[1] << "," << deadband_velocities_[2] << "]");
}

void VelocityDeadbandCritic::score(CriticData& data)
{
  if (!enabled_)
  {
    return;
  }

  const bool holonomic = data.motion_model->isHolonomic();
  for (size_t i = 0; i < data.costs.shape(0); ++i)
  {
    double penalty = 0.0;
    for (size_t j = 0; j < data.state.vx.shape(1); ++j)
    {
      penalty += std::max(0.0, std::fabs(static_cast<double>(deadband_velocities_[0])) - std::fabs(data.state.vx(i, j)));
      penalty += std::max(0.0, std::fabs(static_cast<double>(deadband_velocities_[2])) - std::fabs(data.state.wz(i, j)));
      if (holonomic)
      {
        penalty += std::max(0.0, std::fabs(static_cast<double>(deadband_velocities_[1])) - std::fabs(data.state.vy(i, j)));
      }
    }

    penalty *= data.model_dt * weight_;
    data.costs(i) += static_cast<float>(power_ > 1.0 ? std::pow(penalty, power_) : penalty);
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::VelocityDeadbandCritic, mppi::critics::CriticBase)
