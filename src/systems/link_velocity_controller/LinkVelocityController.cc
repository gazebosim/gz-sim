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
#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
// #include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/Model.hh"

#include "LinkVelocityController.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LinkVelocityControllerPrivate
{
  /// \brief Callback for Velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Vector3d &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Link 1 name
  public: std::string link1Name;

  /// \brief Link 2 name
  public: std::string link2Name;

  // /// \brief Commanded Link Linear Velocity
  // public: math::Vector3d linkLinearVelCmd;

  /// \brief Commanded Link Angular Velocity
  public: math::Vector3d linkAngularVelCmd;

  /// \brief mutex to protect Link commands
  public: std::mutex linkCmdMutex;

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
  this->dataPtr->link1Name = _sdf->Get<std::string>("link1_name");
  // ignerr << _sdf->ToString("") << std::endl;
  // // if (_sdf->HasElement("link"))
  // //   this->dataPtr->linkName = _sdf->GetAttribute("link");
  if (this->dataPtr->link1Name == "")
  {
    ignerr << "LinkVelocityController found an empty link1Name parameter. "
           << "Failed to initialize.";
    return;
  }

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) +
                    "/link/" + this->dataPtr->link1Name + "/cmd_vel"};
  this->dataPtr->node.Subscribe(
    topic, &LinkVelocityControllerPrivate::OnCmdVel, this->dataPtr.get());
  igndbg << "Topic: ["      << topic     << "]"            << std::endl;
}

//////////////////////////////////////////////////
void LinkVelocityController::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("LinkVelocityController::PreUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }
  // If the link hasn't been identified yet, look for it
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
      this->dataPtr->model.LinkByName(_ecm, this->dataPtr->link1Name);
  }

  if (this->dataPtr->linkEntity == kNullEntity)
    return;
  
  // Nothing left to do if paused
  if (_info.paused)
    return;
  
  // // Create link velocity componenet if one doesn't exist
  // auto linearVelocityComp =
  //   _ecm.Component<components::LinearVelocity>(this->dataPtr->linkEntity);
  // if (linearVelocityComp == nullptr)
  // {
  //   _ecm.CreateComponent(
  //     this->dataPtr->linkEntity, components::LinearVelocity());
  // }
  auto angularVelocityComp =
    _ecm.Component<components::AngularVelocity>(this->dataPtr->linkEntity);
  if (angularVelocityComp == nullptr)
  {
    _ecm.CreateComponent(
      this->dataPtr->linkEntity, components::AngularVelocity());
  }
  if (angularVelocityComp == nullptr)
    return;
}

//////////////////////////////////////////////////
void LinkVelocityControllerPrivate::OnCmdVel(const msgs::Vector3d &_msg)
{
  std::lock_guard<std::mutex> lock(this->linkCmdMutex);
  // this->linkLinearVelCmd = _msg.data();
  std::cout << _msg.x() << ", " << _msg.y() << ", " << _msg.z() << std::endl;
  this->linkAngularVelCmd = math::Vector3d(_msg.x(), _msg.y(), _msg.z());
}

IGNITION_ADD_PLUGIN(LinkVelocityController,
                    ignition::gazebo::System,
                    LinkVelocityController::ISystemConfigure,
                    LinkVelocityController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LinkVelocityController,
                          "ignition::gazebo::systems::LinkVelocityController")
