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

#include <mutex>
#include <string>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocityCmd.hh"
#include "ignition/gazebo/Model.hh"

#include "VelocityControl.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::VelocityControlPrivate
{
  /// \brief Callback for model velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Twist &_msg);

  /// \brief Callback for link velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnLinkCmdVel(const ignition::msgs::Twist &_msg,
                        const ignition::transport::MessageInfo &_info);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update upper link velocity.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateUpperLinkVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Update lower link velocity.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateLowerLinkVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Upper link entity of pendulum
  public: Entity upperLink{kNullEntity};

  /// \brief Lower link entity of pendulum
  public: Entity lowerLink{kNullEntity};

  /// \brief Upper link name
  public: std::string upperLinkName;

  /// \brief Lower link name
  public: std::string lowerLinkName;

  /// \brief Angular velocity of a model
  public: math::Vector3d angularVelocity{0, 0, 0};

  /// \brief Linear velocity of a model
  public: math::Vector3d linearVelocity{0, 0, 0};

    /// \brief Angular velocity of upper link
  public: math::Vector3d upperAngularVelocity{0, 0, 0};

  /// \brief Linear velocity of a upper link
  public: math::Vector3d upperLinearVelocity{0, 0, 0};

    /// \brief Angular velocity of lower link
  public: math::Vector3d lowerAngularVelocity{0, 0, 0};

  /// \brief Linear velocity of a lower link
  public: math::Vector3d lowerLinearVelocity{0, 0, 0};

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief Last upper link velocity requested.
  public: msgs::Twist upperLinkVel;

  /// \brief Last lower link velocity requested.
  public: msgs::Twist lowerLinkVel;

  /// \brief A mutex to protect the velocity command.
  public: std::mutex mutex;
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
    ignerr << "VelocityControl plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Subscribe to model commands
  std::string modelTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
  if (_sdf->HasElement("topic"))
    modelTopic = _sdf->Get<std::string>("topic");
  this->dataPtr->node.Subscribe(
    modelTopic, &VelocityControlPrivate::OnCmdVel, this->dataPtr.get());
  ignmsg << "VelocityControl subscribing to twist messages on ["
         << modelTopic << "]"
         << std::endl;

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element *>(_sdf.get());

  if (!ptr->HasElement("link1_name") || !ptr->HasElement("link2_name"))
    return;

  sdf::ElementPtr sdfElem = ptr->GetElement("link1_name");
  while (sdfElem)
  {
    this->dataPtr->upperLinkName = sdfElem->Get<std::string>();
    sdfElem = sdfElem->GetNextElement("link1_name");
  }

  sdfElem = ptr->GetElement("link2_name");
  while (sdfElem)
  {
    this->dataPtr->lowerLinkName = sdfElem->Get<std::string>();
    sdfElem = sdfElem->GetNextElement("link2_name");
  }

  // Subscribe to link commands
  std::string upperLinkTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
                             "/link/" + this->dataPtr->upperLinkName + "/cmd_vel"};
  this->dataPtr->node.Subscribe(
    upperLinkTopic, &VelocityControlPrivate::OnLinkCmdVel, this->dataPtr.get());
    ignmsg << "VelocityControl subscribing to twist messages on ["
           << upperLinkTopic << "]"
           << std::endl;

  std::string lowerLinkTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
                             "/link/" + this->dataPtr->lowerLinkName + "/cmd_vel"};
  this->dataPtr->node.Subscribe(
    lowerLinkTopic, &VelocityControlPrivate::OnLinkCmdVel, this->dataPtr.get());
    ignmsg << "VelocityControl subscribing to twist messages on ["
           << lowerLinkTopic << "]"
           << std::endl;
}

//////////////////////////////////////////////////
void VelocityControl::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("VelocityControl::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // update angular velocity of model
  auto modelAngularVel =
    _ecm.Component<components::AngularVelocityCmd>(
      this->dataPtr->model.Entity());

  if (modelAngularVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->model.Entity(),
      components::AngularVelocityCmd({this->dataPtr->angularVelocity}));
  }
  else
  {
    *modelAngularVel =
      components::AngularVelocityCmd({this->dataPtr->angularVelocity});
  }

  // update linear velocity of model
  auto modelLinearVel =
    _ecm.Component<components::LinearVelocityCmd>(
      this->dataPtr->model.Entity());

  if (modelLinearVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->model.Entity(),
      components::LinearVelocityCmd({this->dataPtr->linearVelocity}));
  }
  else
  {
    *modelLinearVel =
      components::LinearVelocityCmd({this->dataPtr->linearVelocity});
  }

  // If there are links, create link components
  // If the link hasn't been identified yet, look for it
  auto modelName = this->dataPtr->model.Name(_ecm);

  if (this->dataPtr->upperLinkName.empty() &&
      this->dataPtr->lowerLinkName.empty())
        return;
  if (this->dataPtr->upperLink == kNullEntity ||
      this->dataPtr->lowerLink == kNullEntity)
  {
    Entity link = this->dataPtr->model.LinkByName(
                    _ecm, this->dataPtr->upperLinkName);
    if (link != kNullEntity)
      this->dataPtr->upperLink = link;
    else
    {
      ignwarn << "Failed to find upper link [" << this->dataPtr->upperLinkName
              << "] for model [" << modelName << "]" << std::endl;
    }
    link = this->dataPtr->model.LinkByName(_ecm, this->dataPtr->lowerLinkName);
    if (link != kNullEntity)
      this->dataPtr->lowerLink = link;
    else
    {
      ignwarn << "Failed to find lower link [" << this->dataPtr->lowerLinkName
              << "] for model [" << modelName << "]" << std::endl;
    }
  }

  if (this->dataPtr->upperLink == kNullEntity || this->dataPtr->lowerLink == kNullEntity)
    return;

  // update upper link velocity
  auto upperAngularVel =
    _ecm.Component<components::AngularVelocityCmd>(this->dataPtr->upperLink);

  if (upperAngularVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->upperLink,
      components::AngularVelocityCmd({this->dataPtr->upperAngularVelocity}));
  }
  else
  {
    *upperAngularVel =
      components::AngularVelocityCmd({this->dataPtr->upperAngularVelocity});
  }

  auto upperLinearVel =
    _ecm.Component<components::LinearVelocityCmd>(this->dataPtr->upperLink);

  if (upperLinearVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->upperLink,
      components::LinearVelocityCmd({this->dataPtr->upperLinearVelocity}));
  }
  else
  {
    *upperLinearVel =
      components::LinearVelocityCmd({this->dataPtr->upperLinearVelocity});
  }

  // update lower link velocity
  auto lowerAngularVel =
    _ecm.Component<components::AngularVelocityCmd>(this->dataPtr->lowerLink);

  if (lowerAngularVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->lowerLink,
      components::AngularVelocityCmd({this->dataPtr->lowerAngularVelocity}));
  }
  else
  {
    *lowerAngularVel =
      components::AngularVelocityCmd({this->dataPtr->lowerAngularVelocity});
  }

  auto lowerLinearVel =
    _ecm.Component<components::LinearVelocityCmd>(this->dataPtr->lowerLink);

  if (lowerLinearVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->lowerLink,
      components::LinearVelocityCmd({this->dataPtr->lowerLinearVelocity}));
  }
  else
  {
    *lowerLinearVel =
      components::LinearVelocityCmd({this->dataPtr->lowerLinearVelocity});
  }
}

//////////////////////////////////////////////////
void VelocityControl::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("VelocityControl::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateUpperLinkVelocity(_info, _ecm);
  this->dataPtr->UpdateLowerLinkVelocity(_info, _ecm);
}


//////////////////////////////////////////////////
void VelocityControlPrivate::UpdateVelocity(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("VeocityControl::UpdateVelocity");

  double linVel;
  double angVel;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    angVel = this->targetVel.angular().z();
  }

  this->linearVelocity = math::Vector3d(
    linVel, this->targetVel.linear().y(), this->targetVel.linear().z());
  this->angularVelocity = math::Vector3d(
    this->targetVel.angular().x(), this->targetVel.angular().y(), angVel);
}

//////////////////////////////////////////////////
void VelocityControlPrivate::UpdateUpperLinkVelocity(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  this->upperLinearVelocity = math::Vector3d(
    this->upperLinkVel.linear().x(),
    this->upperLinkVel.linear().y(),
    this->upperLinkVel.linear().z());
  this->upperAngularVelocity = math::Vector3d(
    this->upperLinkVel.angular().x(),
    this->upperLinkVel.angular().y(),
    this->upperLinkVel.angular().z());
}

//////////////////////////////////////////////////
void VelocityControlPrivate::UpdateLowerLinkVelocity(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  this->lowerLinearVelocity = math::Vector3d(
    this->lowerLinkVel.linear().x(),
    this->lowerLinkVel.linear().y(),
    this->lowerLinkVel.linear().z());
  this->lowerAngularVelocity = math::Vector3d(
    this->lowerLinkVel.angular().x(),
    this->lowerLinkVel.angular().y(),
    this->lowerLinkVel.angular().z());
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
  if (_info.Topic().find(this->upperLinkName) != std::string::npos)
  {
    this->upperLinkVel = _msg;
  }
  else if (_info.Topic().find(this->lowerLinkName) != std::string::npos)
  {
    this->lowerLinkVel = _msg;
  }
}

IGNITION_ADD_PLUGIN(VelocityControl,
                    ignition::gazebo::System,
                    VelocityControl::ISystemConfigure,
                    VelocityControl::ISystemPreUpdate,
                    VelocityControl::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(VelocityControl,
                          "ignition::gazebo::systems::VelocityControl")
