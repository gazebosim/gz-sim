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
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/Model.hh"

#include "RotorsMotorModel.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::RotorsMotorModelPrivate
{
  /// \brief Callback for joint force subscription
  /// \param[in] _msg Joint force message
  public: void OnCmdForce(const ignition::msgs::Double &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Commanded joint force
  public: double jointForceCmd;

  /// \brief mutex to protect jointForceCmd
  public: std::mutex jointForceCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
RotorsMotorModel::RotorsMotorModel()
  : dataPtr(std::make_unique<RotorsMotorModelPrivate>())
{
}

//////////////////////////////////////////////////
void RotorsMotorModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "RotorsMotorModel plugin should be attached to a model entity. "
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
    ignerr << "RotorsMotorModel found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/joint/" +
                    this->dataPtr->jointName + "/cmd_force"};
  this->dataPtr->node.Subscribe(topic, &RotorsMotorModelPrivate::OnCmdForce,
                                this->dataPtr.get());

  ignmsg << "RotorsMotorModel subscribing to Double messages on [" << topic
         << "]" << std::endl;
}

//////////////////////////////////////////////////
void RotorsMotorModel::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // If the joint hasn't been identified yet, look for it
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }

  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Update joint force
  auto force = _ecm.Component<components::JointForceCmd>(
      this->dataPtr->jointEntity);

  std::lock_guard<std::mutex> lock(this->dataPtr->jointForceCmdMutex);

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
}

//////////////////////////////////////////////////
void RotorsMotorModelPrivate::OnCmdForce(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointForceCmdMutex);
  this->jointForceCmd = _msg.data();
}

IGNITION_ADD_PLUGIN(RotorsMotorModel,
                    ignition::gazebo::System,
                    RotorsMotorModel::ISystemConfigure,
                    RotorsMotorModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RotorsMotorModel,
                          "ignition::gazebo::systems::RotorsMotorModel")
