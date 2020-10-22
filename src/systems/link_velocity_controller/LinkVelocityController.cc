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

#include "ignition/gazebo/Model.hh"

#include "LinkVelocityController.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LinkVelocityControllerPrivate
{
  /// \brief Callback for Velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdPos(const ignition::msgs::Double &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Commanded Link Velocity
  public: double linkPosCmd;

  /// \brief mutex to protect Link commands
  public: std::mutex linkCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Velocity PID controller.
  public: ignition::math::PID posPid;

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
}

//////////////////////////////////////////////////
void LinkVelocityController::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("JointVelocityController::PreUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }
}

//////////////////////////////////////////////////
void LinkVelocityControllerPrivate::OnCmdPos(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->linkCmdMutex);
  this->linkPosCmd = _msg.data();
}

IGNITION_ADD_PLUGIN(LinkVelocityController,
                    ignition::gazebo::System,
                    LinkVelocityController::ISystemConfigure,
                    LinkVelocityController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LinkVelocityController,
                          "ignition::gazebo::systems::LinkVelocityController")
