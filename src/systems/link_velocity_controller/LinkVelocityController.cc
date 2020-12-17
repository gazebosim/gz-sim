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

#include <ignition/msgs/vector3d.pb.h>
#include <ignition/math/PID.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocityCmd.hh"
#include "ignition/gazebo/Model.hh"

#include "LinkVelocityController.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LinkVelocityControllerPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Twist &_msg);

  /// \brief Update link angular velocity.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Link entity
  public: Entity linkEntity;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Angular velocity of link
  public: math::Vector3d linkAngularVel {0, 0, 0};

  /// \brief Mutex to protect link commands
  public: std::mutex mutex;

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
LinkVelocityController::LinkVelocityController()
  : dataPtr(std::make_unique<LinkVelocityControllerPrivate>())
{
}

//////////////////////////////////////////////////
void LinkVelocityController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "LinkVelocityController plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  // TODO: add link2_name
  this->dataPtr->linkName = _sdf->Get<std::string>("link1_name");
  if (this->dataPtr->linkName == "")
  {
    ignerr << "LinkVelocityController found an empty link1Name parameter. "
           << "Failed to initialize.";
    return;
  }

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) +
                    "/link/" + this->dataPtr->linkName + "/cmd_vel"};
  this->dataPtr->node.Subscribe(
    topic, &LinkVelocityControllerPrivate::OnCmdVel, this->dataPtr.get());
  ignmsg << "LinkVelocityController subscribing to twist messages on [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void LinkVelocityController::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // If the link hasn't been identified yet, look for it
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
      this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  if (this->dataPtr->linkEntity == kNullEntity)
    return;

  // update angular velocity of link
  auto angularVel =
    _ecm.Component<components::AngularVelocityCmd>(this->dataPtr->linkEntity);
  if (angularVel == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->linkEntity,
      components::AngularVelocityCmd({this->dataPtr->linkAngularVel}));
  }
  else
  {
    *angularVel =
      components::AngularVelocityCmd({this->dataPtr->linkAngularVel});
  }
}

//////////////////////////////////////////////////
void LinkVelocityController::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
}

//////////////////////////////////////////////////
void LinkVelocityControllerPrivate::UpdateVelocity(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  double angVel;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    angVel = this->targetVel.angular().z();
  }

  this->linkAngularVel = math::Vector3d(
    this->targetVel.angular().x(), this->targetVel.angular().y(), angVel);
}

//////////////////////////////////////////////////
void LinkVelocityControllerPrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

IGNITION_ADD_PLUGIN(LinkVelocityController,
                    ignition::gazebo::System,
                    LinkVelocityController::ISystemConfigure,
                    LinkVelocityController::ISystemPreUpdate,
                    LinkVelocityController::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LinkVelocityController,
                          "ignition::gazebo::systems::LinkVelocityController")
