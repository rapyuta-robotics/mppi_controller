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

void CriticManager::on_configure(const ros::NodeHandle& parent_nh, const std::string& name,
                                 costmap_2d::Costmap2DROS* costmap_ros)
{
  parent_nh_ = parent_nh;
  costmap_ros_ = costmap_ros;
  name_ = name;
  getParams();
}

void CriticManager::getParams()
{
  const XmlRpc::XmlRpcValue plugins = parent_nh_.param<XmlRpc::XmlRpcValue>("critics", XmlRpc::XmlRpcValue());
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

void CriticManager::evalTrajectoriesScores(CriticData& data) const
{
  for (size_t q = 0; q < critics_.size(); q++)
  {
    if (data.fail_flag)
    {
      break;
    }
    critics_[q]->score(data);
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
