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

#include <ignition/msgs/double.pb.h>
#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
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
  public: void OnCmdVel(const ignition::msgs::Double &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Link name
  public: std::string linkName;

  // /// \brief Commanded Link Linear Velocity
  // public: math::Vector3d linkLinearVelCmd;

  /// \brief Commanded Link Angular Velocity
  public: double linkAngularVelCmd;

  /// \brief mutex to protect Link commands
  public: std::mutex linkCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Link index to be used.
  public: unsigned int linkIndex = 0u;
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
  this->dataPtr->linkName = _sdf->Get<std::string>("link_name");
  if (this->dataPtr->linkName == "")
  {
    ignerr << "LinkVelocityController found an empty linkName parameter. "
           << "Failed to initialize.";
    return;
  }

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) +
                    "/link/" + this->dataPtr->linkName + "/" +
                    std::to_string(this->dataPtr->linkIndex) + "/cmd_vel"};
  this->dataPtr->node.Subscribe(
    topic, &linkVelocityControllerPrivate::OnCmdVel, this->dataPtr.get());
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
      this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
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

  // Sanity Check: make sure the link index is valid
  if (this->dataPtr->linkIndex >= angularVelocityComp->Data().size())
  {
    static bool invalidLinkReported = false;
    if (!invalidLinkReported)
    {
      ignerr << "[LinkPositionController]: Detected an invalid <link_index> "
             << "parameter. The index specified is ["
             << this->dataPtr->linkIndex << "] but the link only has ["
             << angularVelocityComp->Data().size() << "] index[es]. "
             << "This controller will be ignored" << std::endl;
      invalidLinkReported = true;
    }
    return;
  }
}

//////////////////////////////////////////////////
void LinkVelocityControllerPrivate::OnCmdPos(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->linkCmdMutex);
  // this->linkLinearVelCmd = _msg.data();
  this->linkAngularVelCmd = msg.data();
}

IGNITION_ADD_PLUGIN(LinkVelocityController,
                    ignition::gazebo::System,
                    LinkVelocityController::ISystemConfigure,
                    LinkVelocityController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LinkVelocityController,
                          "ignition::gazebo::systems::LinkVelocityController")
