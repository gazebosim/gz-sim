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

#include "ApplyJointForce.hh"

#include <gz/msgs/double.pb.h>

#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ApplyJointForcePrivate
{
  /// \brief Callback for joint force subscription
  /// \param[in] _msg Joint force message
  public: void OnCmdForce(const msgs::Double &_msg);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  //! [jointEntityDeclaration]
  public: Entity jointEntity;
  //! [jointEntityDeclaration]

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Commanded joint force
  //! [forceDeclaration]
  public: double jointForceCmd;
  //! [forceDeclaration]

  /// \brief mutex to protect jointForceCmd
  public: std::mutex jointForceCmdMutex;

  /// \brief Model interface
  //! [modelDeclaration]
  public: Model model{kNullEntity};
  //! [modelDeclaration]
};

//////////////////////////////////////////////////
ApplyJointForce::ApplyJointForce()
  : dataPtr(std::make_unique<ApplyJointForcePrivate>())
{
}

//////////////////////////////////////////////////
//! [Configure]
void ApplyJointForce::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  //! [Configure]

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "ApplyJointForce plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  // Get params from SDF
  auto sdfElem = sdfClone->GetElement("joint_name");
  if (sdfElem)
  {
    this->dataPtr->jointName = sdfElem->Get<std::string>();
  }

  if (this->dataPtr->jointName == "")
  {
    gzerr << "ApplyJointForce found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  // Subscribe to commands
  //! [cmdTopic]
  auto topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/joint/" + this->dataPtr->jointName +
      "/cmd_force");
  //! [cmdTopic]
  if (topic.empty())
  {
    gzerr << "Failed to create valid topic for [" << this->dataPtr->jointName
           << "]" << std::endl;
    return;
  }
  //! [cmdSub]
  this->dataPtr->node.Subscribe(topic, &ApplyJointForcePrivate::OnCmdForce,
                                this->dataPtr.get());
  //! [cmdSub]

  gzmsg << "ApplyJointForce subscribing to Double messages on [" << topic
         << "]" << std::endl;
}

//////////////////////////////////////////////////
void ApplyJointForce::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ApplyJointForce::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // If the joint hasn't been identified yet, look for it
  //! [findJoint]
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }
  //! [findJoint]

  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Update joint force
  //! [jointForceComponent]
  auto force = _ecm.Component<components::JointForceCmd>(
      this->dataPtr->jointEntity);
  //! [jointForceComponent]

  std::lock_guard<std::mutex> lock(this->dataPtr->jointForceCmdMutex);

  //! [modifyComponent]
  if (force == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity,
        components::JointForceCmd({this->dataPtr->jointForceCmd}));
  }
  else
  {
    force->Data()[0] += this->dataPtr->jointForceCmd;
  }
  //! [modifyComponent]
}

//////////////////////////////////////////////////
//! [setForce]
void ApplyJointForcePrivate::OnCmdForce(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointForceCmdMutex);
  this->jointForceCmd = _msg.data();
}
//! [setForce]

GZ_ADD_PLUGIN(ApplyJointForce,
                    System,
                    ApplyJointForce::ISystemConfigure,
                    ApplyJointForce::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ApplyJointForce,
                          "gz::sim::systems::ApplyJointForce")
