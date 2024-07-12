/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gz/msgs/twist.pb.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "VelocityControl.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::VelocityControlPrivate
{
  /// \brief Callback for model velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Twist &_msg);

  /// \brief Callback for link velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnLinkCmdVel(const msgs::Twist &_msg,
    const transport::MessageInfo &_info);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update link velocity.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateLinkVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Angular velocity of a model
  public: math::Vector3d angularVelocity{0, 0, 0};

  /// \brief Linear velocity of a model
  public: math::Vector3d linearVelocity{0, 0, 0};

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief A mutex to protect the model velocity command.
  public: std::mutex mutex;

  /// \brief Link names
  public: std::vector<std::string> linkNames;

  /// \brief Link entities in a model
  public: std::unordered_map<std::string, Entity> links;

  /// \brief Angular velocities of links
  public: std::unordered_map<std::string, math::Vector3d> angularVelocities;

  /// \brief Linear velocities of links
  public: std::unordered_map<std::string, math::Vector3d> linearVelocities;

  /// \brief All link velocites
  public: std::unordered_map<std::string, msgs::Twist> linkVels;
};

//////////////////////////////////////////////////
VelocityControl::VelocityControl()
  : dataPtr(std::make_unique<VelocityControlPrivate>())
{
}

//////////////////////////////////////////////////
void VelocityControl::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "VelocityControl plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("initial_linear"))
  {
    this->dataPtr->linearVelocity = _sdf->Get<math::Vector3d>("initial_linear");
    msgs::Set(this->dataPtr->targetVel.mutable_linear(),
        this->dataPtr->linearVelocity);
    gzmsg << "Linear velocity initialized to ["
           << this->dataPtr->linearVelocity << "]" << std::endl;
  }

  if (_sdf->HasElement("initial_angular"))
  {
    this->dataPtr->angularVelocity =
        _sdf->Get<math::Vector3d>("initial_angular");
    msgs::Set(this->dataPtr->targetVel.mutable_angular(),
        this->dataPtr->angularVelocity);
    gzmsg << "Angular velocity initialized to ["
           << this->dataPtr->angularVelocity << "]" << std::endl;
  }

  // Subscribe to model commands
  std::vector<std::string> modelTopics;
  if (_sdf->HasElement("topic"))
  {
    modelTopics.push_back(_sdf->Get<std::string>("topic"));
  }
  modelTopics.push_back(
    "/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
  auto modelTopic = validTopic(modelTopics);
  this->dataPtr->node.Subscribe(
    modelTopic, &VelocityControlPrivate::OnCmdVel, this->dataPtr.get());
  gzmsg << "VelocityControl subscribing to twist messages on ["
         << modelTopic << "]"
         << std::endl;

  if (!_sdf->HasElement("link_name"))
    return;

  auto sdfElem = _sdf->FindElement("link_name");
  while (sdfElem)
  {
    this->dataPtr->linkNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("link_name");
  }

  // Subscribe to link commands
  for (const auto &linkName : this->dataPtr->linkNames)
  {
    std::string linkTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
                             "/link/" + linkName + "/cmd_vel"};
    linkTopic = transport::TopicUtils::AsValidTopic(linkTopic);
    this->dataPtr->node.Subscribe(
        linkTopic, &VelocityControlPrivate::OnLinkCmdVel, this->dataPtr.get());
    gzmsg << "VelocityControl subscribing to twist messages on ["
           << linkTopic << "]"
           << std::endl;
  }
}

//////////////////////////////////////////////////
void VelocityControl::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("VelocityControl::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // update angular velocity of model
  _ecm.SetComponentData<components::AngularVelocityCmd>(
    this->dataPtr->model.Entity(), {this->dataPtr->angularVelocity});

  // update linear velocity of model
  _ecm.SetComponentData<components::LinearVelocityCmd>(
    this->dataPtr->model.Entity(), {this->dataPtr->linearVelocity});

  // If there are links, create link components
  // If the link hasn't been identified yet, look for it
  auto modelName = this->dataPtr->model.Name(_ecm);

  if (this->dataPtr->linkNames.empty())
    return;

  // find all the link entity ids
  if (this->dataPtr->links.size() != this->dataPtr->linkNames.size())
  {
    for (const auto &linkName : this->dataPtr->linkNames)
    {
      if (this->dataPtr->links.find(linkName) == this->dataPtr->links.end())
      {
        Entity link = this->dataPtr->model.LinkByName(_ecm, linkName);
        if (link != kNullEntity)
          this->dataPtr->links.insert({linkName, link});
        else
        {
          gzwarn << "Failed to find link [" << linkName
                << "] for model [" << modelName << "]" << std::endl;
        }
      }
    }
  }

  // update link velocities
  for (const auto& [linkName, angularVel] : this->dataPtr->angularVelocities)
  {
    auto it = this->dataPtr->links.find(linkName);
    if (it != this->dataPtr->links.end())
    {
      _ecm.SetComponentData<components::AngularVelocityCmd>(
        it->second, {angularVel});
    }
    else
    {
      gzwarn << "No link found for angular velocity cmd ["
              << linkName << "]" << std::endl;
    }
  }

  for (const auto& [linkName, linearVel] : this->dataPtr->linearVelocities)
  {
    auto it = this->dataPtr->links.find(linkName);
    if (it != this->dataPtr->links.end())
    {
      _ecm.SetComponentData<components::LinearVelocityCmd>(
        it->second, {linearVel});
    }
    else
    {
      gzwarn << "No link found for linear velocity cmd ["
              << linkName << "]" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void VelocityControl::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("VelocityControl::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // update model velocities
  this->dataPtr->UpdateVelocity(_info, _ecm);
  // update link velocities
  this->dataPtr->UpdateLinkVelocity(_info, _ecm);
}

//////////////////////////////////////////////////
void VelocityControlPrivate::UpdateVelocity(
    const UpdateInfo &/*_info*/,
    const EntityComponentManager &/*_ecm*/)
{
  GZ_PROFILE("VeocityControl::UpdateVelocity");

  std::lock_guard<std::mutex> lock(this->mutex);
  this->linearVelocity = msgs::Convert(this->targetVel.linear());
  this->angularVelocity = msgs::Convert(this->targetVel.angular());
}

//////////////////////////////////////////////////
void VelocityControlPrivate::UpdateLinkVelocity(
    const UpdateInfo &/*_info*/,
    const EntityComponentManager &/*_ecm*/)
{
  GZ_PROFILE("VelocityControl::UpdateLinkVelocity");

  std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto& [linkName, msg] : this->linkVels)
  {
    auto linearVel = msgs::Convert(msg.linear());
    auto angularVel = msgs::Convert(msg.angular());
    this->linearVelocities[linkName] = linearVel;
    this->angularVelocities[linkName] = angularVel;
  }
  this->linkVels.clear();
}

//////////////////////////////////////////////////
void VelocityControlPrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

//////////////////////////////////////////////////
void VelocityControlPrivate::OnLinkCmdVel(const msgs::Twist &_msg,
                                      const transport::MessageInfo &_info)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto &linkName : this->linkNames)
  {
    if (_info.Topic().find("/" + linkName + "/cmd_vel") != std::string::npos)
    {
      this->linkVels.insert({linkName, _msg});
      break;
    }
  }
}

GZ_ADD_PLUGIN(VelocityControl,
                    System,
                    VelocityControl::ISystemConfigure,
                    VelocityControl::ISystemPreUpdate,
                    VelocityControl::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(VelocityControl,
                          "gz::sim::systems::VelocityControl")
