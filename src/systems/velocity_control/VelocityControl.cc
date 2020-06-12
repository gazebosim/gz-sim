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
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Twist &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Angular velocity of a model
  public: math::Vector3d angularVelocity{0, 0, 0};

  /// \brief Linear velocity of a model
  public: math::Vector3d linearVelocity{0, 0, 0};

  /// \brief Model interface
  public: Model model{kNullEntity};
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

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");
  this->dataPtr->node.Subscribe(
    topic, &VelocityControlPrivate::OnCmdVel, this->dataPtr.get());

  ignmsg << "VelocityControl subscribing to twist messages on [" << topic << "]"
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
  auto angularVel =
    _ecm.Component<components::AngularVelocityCmd>(
      this->dataPtr->model.Entity());

  if (angularVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->model.Entity(),
      components::AngularVelocityCmd({this->dataPtr->angularVelocity}));
  }
  else
  {
    *angularVel =
      components::AngularVelocityCmd({this->dataPtr->angularVelocity});
  }

  // update linear velocity of model
  auto linearVel =
    _ecm.Component<components::LinearVelocityCmd>(
      this->dataPtr->model.Entity());

  if (linearVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->model.Entity(),
      components::LinearVelocityCmd({this->dataPtr->linearVelocity}));
  }
  else
  {
    *linearVel =
      components::LinearVelocityCmd({this->dataPtr->linearVelocity});
  }
}

//////////////////////////////////////////////////
void VelocityControlPrivate::OnCmdVel(const msgs::Twist &_msg)
{
  this->linearVelocity = math::Vector3d(
    _msg.linear().x(), _msg.linear().y(), _msg.linear().z());
  this->angularVelocity = math::Vector3d(
    _msg.angular().x(), _msg.angular().y(), _msg.angular().z());
}

IGNITION_ADD_PLUGIN(VelocityControl,
                    ignition::gazebo::System,
                    VelocityControl::ISystemConfigure,
                    VelocityControl::ISystemPreUpdate,
                    VelocityControl::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(VelocityControl,
                          "ignition::gazebo::systems::VelocityControl")
