/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

//#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/PID.hh>
#include <ignition/msgs/twist.pb.h>

#include <sdf/sdf.hh>

// #include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
// #include "ignition/gazebo/components/JointAxis.hh"
// #include "ignition/gazebo/components/JointVelocity.hh"
// #include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
// #include "ignition/gazebo/components/ParentLinkName.hh"
// #include "ignition/gazebo/components/Pose.hh"
// #include "ignition/gazebo/components/Wind.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "MulticopterVelocityControl.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::MulticopterVelocityControlPrivate
{
  public: void OnTwist(const ignition::msgs::Twist &_msg);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Link name
  public: std::string linkName;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Topic namespace.
  public: std::string robotNamespace;

  /// \brief Topic for twist commands.
  public: std::string commandSubTopic;

  /// \brief Ignition communication node.
  public: transport::Node node;

  public: math::PID rollPid, pitchPid, yawPid;
};

//////////////////////////////////////////////////
MulticopterVelocityControl::MulticopterVelocityControl()
  : dataPtr(std::make_unique<MulticopterVelocityControlPrivate>())
{
}

//////////////////////////////////////////////////
void MulticopterVelocityControl::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "MulticopterVelocityControl plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  this->dataPtr->robotNamespace.clear();
  if (sdfClone->HasElement("robotNamespace"))
  {
    this->dataPtr->robotNamespace =
        sdfClone->Get<std::string>("robotNamespace");
  }
  else
  {
    ignerr << "Please specify a robotNamespace.\n";
  }

  sdfClone->Get<std::string>("commandSubTopic",
      this->dataPtr->commandSubTopic, this->dataPtr->commandSubTopic);

  if (sdfClone->HasElement("linkName"))
    this->dataPtr->linkName = sdfClone->Get<std::string>("linkName");

  if (this->dataPtr->linkName.empty())
  {
    ignerr << "found an empty linkName parameter. Failed to initialize.\n";
    return;
  }


  // Subscribe to actuator command messages
  std::string topic{this->dataPtr->robotNamespace + "/"
    + this->dataPtr->commandSubTopic};

  this->dataPtr->node.Subscribe(topic,
      &MulticopterVelocityControlPrivate::OnTwist, this->dataPtr.get());
}

//////////////////////////////////////////////////
void MulticopterVelocityControl::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("MulticopterVelocityControl::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Get the link entity
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }
  if (!this->dataPtr->linkEntity == kNullEntity)
    return;

  // Create a world linear velocity component if one is not present.
  if (!_ecm.Component<components::WorldLinearVelocity>(
      this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity,
        components::WorldLinearVelocity());
  }
  else
  {
    Link link(this->dataPtr->linkEntity);
    const std::optional<math::Vector3d> worldLinearVel =
      link.WorldLinearVelocity(_ecm);
    std::cout << "Linear Vel[" << *worldLinearVel << "]\n";
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;
}

//////////////////////////////////////////////////
void MulticopterVelocityControlPrivate::OnTwist(
    const ignition::msgs::Twist &_msg)
{

}


IGNITION_ADD_PLUGIN(MulticopterVelocityControl,
                    ignition::gazebo::System,
                    MulticopterVelocityControl::ISystemConfigure,
                    MulticopterVelocityControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MulticopterVelocityControl,
                          "ignition::gazebo::systems::MulticopterVelocityControl")
