/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include "ConveyorBelt.hh"

#include <gz/msgs/double.pb.h>

#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Joint.hh"
#include "gz/sim/Model.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ConveyorBeltPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Double &_msg);

    /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Joint interface
  public: Joint joint;

  /// \brief Commanded joint velocity
  public: double jointVelCmd{0.0};

  /// \brief Joint upper limit
  public: double jointLimit{0.0};

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
ConveyorBelt::ConveyorBelt()
  : dataPtr(std::make_unique<ConveyorBeltPrivate>())
{
}

//////////////////////////////////////////////////
void ConveyorBelt::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "ConveyorBelt plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto jointElem = _sdf->FindElement("joint_name");
  if (jointElem)
  {
    this->dataPtr->jointName = jointElem->Get<std::string>();
  }
  else
  {
    gzerr << "<joint_name> not specified." << std::endl;
    return;
  }

  // Subscribe to commands
  std::string topic = "conveyor_belt/cmd";
  if (_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      gzerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
             << "]" << " for joint [" << this->dataPtr->jointName
             << "]" << std::endl;
      return;
    }
  }

  this->dataPtr->node.Subscribe(topic,
    &ConveyorBeltPrivate::OnCmdVel,
    this->dataPtr.get());

  gzmsg << "ConveyorBelt subscribing to Double messages on [" << topic
       << "]" << std::endl;
}

//////////////////////////////////////////////////
void ConveyorBelt::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ConveyorBelt::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }


  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity = this->dataPtr->model.JointByName(
        _ecm, this->dataPtr->jointName);

    if (this->dataPtr->jointEntity == kNullEntity)
    {
      gzerr << "Unable to find joint " << this->dataPtr->jointName << std::endl;
      return;
    }

    this->dataPtr->joint = Joint(this->dataPtr->jointEntity);
    this->dataPtr->joint.EnableVelocityCheck(_ecm, true);
    this->dataPtr->joint.EnablePositionCheck(_ecm, true);

    std::optional<std::vector<sdf::JointAxis>> axes = this->dataPtr->joint.Axis(_ecm);
    if (axes.has_value() && !axes.value().empty())
    {
      this->dataPtr->jointLimit = axes.value()[0].Upper();
    }
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  double targetVel = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex);
    targetVel = this->dataPtr->jointVelCmd;
  }

  this->dataPtr->joint.SetVelocity(_ecm, {targetVel});

  double position = 0.0;
  auto positions = this->dataPtr->joint.Position(_ecm);
  if (positions.has_value() && !positions.value().empty())
  {
    position = positions.value()[0];
  }

  if (position >= this->dataPtr->jointLimit)
  {
    std::vector<double> resetPos{0};
    this->dataPtr->joint.ResetPosition(_ecm, resetPos);
  }
}

//////////////////////////////////////////////////
void ConveyorBeltPrivate::OnCmdVel(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  this->jointVelCmd = _msg.data();
}


GZ_ADD_PLUGIN(ConveyorBelt,
                    System,
                    ConveyorBelt::ISystemConfigure,
                    ConveyorBelt::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ConveyorBelt,
                    "gz::sim::systems::ConveyorBelt")
