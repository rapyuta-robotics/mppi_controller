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

#include "mppi_controller/critic_manager.hpp"

namespace mppi
{

CriticManager::CriticManager() : loader_("mppi_controller", "mppi::critics::CriticBase")
{
}

void CriticManager::on_configure(const ros::NodeHandle& parent_nh, costmap_2d::Costmap2DROS* costmap_ros)
{
  parent_nh_ = parent_nh;
  costmap_ros_ = costmap_ros;
  getParams();
  critics_stats_publisher_ = parent_nh_.advertise<mppi_controller::CriticsStats>("critics_stats", 1);
}

void CriticManager::getParams()
{
  const XmlRpc::XmlRpcValue plugins = parent_nh_.param<XmlRpc::XmlRpcValue>("critics", XmlRpc::XmlRpcValue());
  visualize_ = parent_nh_.param("visualize", false);
  if (plugins.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_FATAL("'critics' parameter does not define a valid list");
    ros::shutdown();
    return;
  }

  for (int i = 0; i < plugins.size(); ++i)
  {
    const auto& plugin = plugins[i];
    if (plugin.getType() != XmlRpc::XmlRpcValue::TypeStruct || !plugin.hasMember("name") || !plugin.hasMember("type"))
    {
      ROS_ERROR_STREAM("Critic at index " << i << " does not define a valid struct with 'name' and 'type' fields");
      continue;
    }

    try
    {
      const auto& critic = critics_.emplace_back(loader_.createInstance(plugin["type"]));
      critic->on_configure(parent_nh_, plugin["name"], costmap_ros_);
    }
    catch (const pluginlib::PluginlibException& e)
    {
      ROS_ERROR_STREAM("Failed to load plugin. " << e.what());
    }
  }
}

void CriticManager::evalTrajectoriesScores(CriticData& data)
{
  if (visualize_)
  {
    critic_costs_.clear();
    critic_costs_.reserve(critics_.size());
  }

  const bool publish_stats = visualize_ && critics_stats_publisher_.getNumSubscribers() > 0;
  mppi_controller::CriticsStats stats_msg;
  if (publish_stats)
  {
    stats_msg.critics.reserve(critics_.size());
    stats_msg.changed.reserve(critics_.size());
    stats_msg.costs_sum.reserve(critics_.size());
  }

  for (size_t q = 0; q < critics_.size(); q++)
  {
    if (data.fail_flag)
    {
      break;
    }

    xt::xtensor<float, 1> costs_before;
    if (visualize_)
    {
      costs_before = data.costs;
    }

    critics_[q]->score(data);

    if (visualize_)
    {
      auto cost_diff = data.costs - costs_before;
      critic_costs_.emplace_back(critics_[q]->getName(), cost_diff);

      if (publish_stats)
      {
        const float costs_sum = xt::sum(cost_diff)();
        stats_msg.critics.push_back(critics_[q]->getName());
        stats_msg.changed.push_back(costs_sum != 0.0f);
        stats_msg.costs_sum.push_back(costs_sum);
      }
    }
  }

  if (publish_stats)
  {
    stats_msg.stamp = ros::Time::now();
    critics_stats_publisher_.publish(stats_msg);
  }
}

void CriticManager::updateConstraints(const models::ControlConstraints& constraints)
{
  for (size_t q = 0; q < critics_.size(); q++)
  {
    critics_[q]->updateConstraints(constraints);
  }
}
}  // namespace mppi
