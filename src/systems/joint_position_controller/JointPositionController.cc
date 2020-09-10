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

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/Model.hh"

#include "JointPositionController.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::JointPositionControllerPrivate
{
  /// \brief Callback for position subscription
  /// \param[in] _msg Position message
  public: void OnCmdPos(const ignition::msgs::Double &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Commanded joint position
  public: double jointPosCmd;

  /// \brief mutex to protect joint commands
  public: std::mutex jointCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Position PID controller.
  public: ignition::math::PID posPid;

  /// \brief Joint index to be used.
  public: unsigned int jointIndex = 0u;
};

//////////////////////////////////////////////////
JointPositionController::JointPositionController()
  : dataPtr(std::make_unique<JointPositionControllerPrivate>())
{
}

//////////////////////////////////////////////////
void JointPositionController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "JointPositionController plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  this->dataPtr->jointName = _sdf->Get<std::string>("joint_name");

  if (this->dataPtr->jointName == "")
  {
    ignerr << "JointPositionController found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (_sdf->HasElement("joint_index"))
  {
    this->dataPtr->jointIndex = _sdf->Get<unsigned int>("joint_index");
  }

  // PID parameters
  double p         =  1;
  double i         =  0.1;
  double d         =  0.01;
  double iMax      =  1;
  double iMin      = -1;
  double cmdMax    =  1000;
  double cmdMin    = -1000;
  double cmdOffset =  0;

  if (_sdf->HasElement("p_gain"))
  {
    p = _sdf->Get<double>("p_gain");
  }
  if (_sdf->HasElement("i_gain"))
  {
    i = _sdf->Get<double>("i_gain");
  }
  if (_sdf->HasElement("d_gain"))
  {
    d = _sdf->Get<double>("d_gain");
  }
  if (_sdf->HasElement("i_max"))
  {
    iMax = _sdf->Get<double>("i_max");
  }
  if (_sdf->HasElement("i_min"))
  {
    iMin = _sdf->Get<double>("i_min");
  }
  if (_sdf->HasElement("cmd_max"))
  {
    cmdMax = _sdf->Get<double>("cmd_max");
  }
  if (_sdf->HasElement("cmd_min"))
  {
    cmdMin = _sdf->Get<double>("cmd_min");
  }
  if (_sdf->HasElement("cmd_offset"))
  {
    cmdOffset = _sdf->Get<double>("cmd_offset");
  }

  this->dataPtr->posPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) +
                    "/joint/" + this->dataPtr->jointName + "/" +
                    std::to_string(this->dataPtr->jointIndex) + "/cmd_pos"};
  this->dataPtr->node.Subscribe(
      topic, &JointPositionControllerPrivate::OnCmdPos, this->dataPtr.get());

  igndbg << "[JointPositionController] system parameters:" << std::endl;
  igndbg << "p_gain: ["     << p         << "]"            << std::endl;
  igndbg << "i_gain: ["     << i         << "]"            << std::endl;
  igndbg << "d_gain: ["     << d         << "]"            << std::endl;
  igndbg << "i_max: ["      << iMax      << "]"            << std::endl;
  igndbg << "i_min: ["      << iMin      << "]"            << std::endl;
  igndbg << "cmd_max: ["    << cmdMax    << "]"            << std::endl;
  igndbg << "cmd_min: ["    << cmdMin    << "]"            << std::endl;
  igndbg << "cmd_offset: [" << cmdOffset << "]"            << std::endl;
  igndbg << "Topic: ["      << topic     << "]"            << std::endl;
}

//////////////////////////////////////////////////
void JointPositionController::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("JointPositionController::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

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

  // Create joint position component if one doesn't exist
  auto jointPosComp =
      _ecm.Component<components::JointPosition>(this->dataPtr->jointEntity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity, components::JointPosition());
  }
  if (jointPosComp == nullptr)
    return;

  // Sanity check: Make sure that the joint index is valid.
  if (this->dataPtr->jointIndex >= jointPosComp->Data().size())
  {
    static bool invalidJointReported = false;
    if (!invalidJointReported)
    {
      ignerr << "[JointPositionController]: Detected an invalid <joint_index> "
             << "parameter. The index specified is ["
             << this->dataPtr->jointIndex << "] but the joint only has ["
             << jointPosComp->Data().size() << "] index[es]. "
             << "This controller will be ignored" << std::endl;
      invalidJointReported = true;
    }
    return;
  }

  // Update force command.
  double error;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointCmdMutex);
    error = jointPosComp->Data().at(this->dataPtr->jointIndex) -
            this->dataPtr->jointPosCmd;
  }

  double force = this->dataPtr->posPid.Update(error, _info.dt);

  auto forceComp =
      _ecm.Component<components::JointForceCmd>(this->dataPtr->jointEntity);
  if (forceComp == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity,
                         components::JointForceCmd({force}));
  }
  else
  {
    forceComp->Data()[this->dataPtr->jointIndex] = force;
  }
}

//////////////////////////////////////////////////
void JointPositionControllerPrivate::OnCmdPos(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointCmdMutex);
  this->jointPosCmd = _msg.data();
}

IGNITION_ADD_PLUGIN(JointPositionController,
                    ignition::gazebo::System,
                    JointPositionController::ISystemConfigure,
                    JointPositionController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(JointPositionController,
                          "ignition::gazebo::systems::JointPositionController")
