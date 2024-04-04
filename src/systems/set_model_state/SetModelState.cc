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

#include "SetModelState.hh"

#include <string>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Angle.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::SetModelStatePrivate
{
  /// \brief Model interface
  //! [modelDeclaration]
  public: Model model{kNullEntity};
  //! [modelDeclaration]
};

//////////////////////////////////////////////////
SetModelState::SetModelState()
  : dataPtr(std::make_unique<SetModelStatePrivate>())
{
}

//////////////////////////////////////////////////
//! [Configure]
void SetModelState::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  const std::string modelName = this->dataPtr->model.Name(_ecm);
  //! [Configure]

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "SetModelState plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  auto modelStateElem = sdfClone->FindElement("model_state");
  if (!modelStateElem)
  {
    gzerr << "No <model_state> specified; the model state is unchanged.\n";
    return;
  }

  for (auto jointStateElem = modelStateElem->FindElement("joint_state");
       jointStateElem != nullptr;
       jointStateElem = jointStateElem->GetNextElement("joint_state"))
  {
    std::pair<std::string, bool> namePair =
        jointStateElem->Get<std::string>("name", "");
    if (!namePair.second)
    {
      gzerr << "No name specified for joint_state, skipping.\n";
      continue;
    }
    const auto &jointName = namePair.first;

    Entity jointEntity = this->dataPtr->model.JointByName(_ecm, jointName);
    if (jointEntity == kNullEntity)
    {
      gzerr << "Unable to find joint with name [" << jointName << "] "
            << "in model with name [" << modelName << "], skipping.\n";
      continue;
    }

    if (!_ecm.EntityHasComponentType(jointEntity,
                                     components::JointAxis::typeId))
    {
      gzerr << "Joint with name [" << jointName << "] "
            << "in model with name [" << modelName << "] "
            << "has no JointAxis component (is it a fixed joint?), "
            << "skipping.\n";
      continue;
    }

    bool jointPositionSet = false;
    bool jointVelocitySet = false;
    std::vector<double> jointPosition;
    std::vector<double> jointVelocity;

    auto parseScalarWithDegrees =
        [](math::Angle &_scalar, sdf::ElementPtr _elem) -> bool
    {
      if (_elem)
      {
        // parse degrees attribute, default false
        std::pair<bool, bool> degreesPair = _elem->Get<bool>("degrees", false);
        // parse element scalar value, default 0.0
        std::pair<double, bool> scalarPair = _elem->Get<double>("", 0.0);
        if (scalarPair.second)
        {
          if (degreesPair.first)
          {
            _scalar.SetDegree(scalarPair.first);
          }
          else
          {
            _scalar.SetRadian(scalarPair.first);
          }
          return true;
        }
      }
      return false;
    };

    {
      auto axisElem = jointStateElem->FindElement("axis_state");
      if (axisElem)
      {
        auto positionElem = axisElem ->FindElement("position");
        if (positionElem)
        {
          math::Angle position;
          bool scalarParsed = parseScalarWithDegrees(position, positionElem);
          jointPositionSet = scalarParsed || jointPositionSet;
          jointPosition.push_back(position.Radian());
        }

        auto velocityElem = axisElem ->FindElement("velocity");
        if (velocityElem)
        {
          math::Angle velocity;
          bool scalarParsed = parseScalarWithDegrees(velocity, velocityElem);
          jointVelocitySet = scalarParsed || jointVelocitySet;
          jointVelocity.push_back(velocity.Radian());
        }
      }
      // else
      // {
      //   // <axis_state> not found
      // }
    }

    // If joint entity has a JointAxis2 component,
    // then try to parse <axis2_state>
    if (_ecm.EntityHasComponentType(jointEntity,
                                    components::JointAxis2::typeId))
    {
      auto axisElem = jointStateElem->FindElement("axis2_state");
      if (axisElem)
      {
        auto positionElem = axisElem ->FindElement("position");
        if (positionElem)
        {
          math::Angle position;
          bool scalarParsed = parseScalarWithDegrees(position, positionElem);
          jointPositionSet = scalarParsed || jointPositionSet;
          // check if //axis_state/position was set; if not,
          // push 0.0 for first axis before pushing //axis2_state/position
          if (jointPosition.empty())
          {
            jointPosition.push_back(0.0);
          }
          jointPosition.push_back(position.Radian());
        }

        auto velocityElem = axisElem ->FindElement("velocity");
        if (velocityElem)
        {
          math::Angle velocity;
          bool scalarParsed = parseScalarWithDegrees(velocity, velocityElem);
          jointVelocitySet = scalarParsed || jointVelocitySet;
          jointVelocity.push_back(velocity.Radian());
          // check if //axis_state/velocity was set; if not,
          // push 0.0 for first axis before pushing //axis2_state/velocity
          if (jointVelocity.empty())
          {
            jointVelocity.push_back(0.0);
          }
          jointVelocity.push_back(velocity.Radian());
        }
      }
      // else
      // {
      //   // <axis2_state> not found
      // }
    }

    if (jointPositionSet)
    {
      _ecm.SetComponentData<components::JointPositionReset>(jointEntity,
                                                            jointPosition);
    }

    if (jointVelocitySet)
    {
      _ecm.SetComponentData<components::JointVelocityReset>(jointEntity,
                                                            jointVelocity);
    }
  }
}

//////////////////////////////////////////////////
void SetModelState::Reset(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("SetModelState::Reset");

  // // \TODO(anyone) Support rewind
  // if (_info.dt < std::chrono::steady_clock::duration::zero())
  // {
  //   gzwarn << "Detected jump back in time ["
  //       << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
  //       << "s]. System may not work properly." << std::endl;
  // }

  // // If the joint hasn't been identified yet, look for it
  // //! [findJoint]
  // if (this->dataPtr->jointEntity == kNullEntity)
  // {
  //   this->dataPtr->jointEntity =
  //       this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  // }
  // //! [findJoint]

  // if (this->dataPtr->jointEntity == kNullEntity)
  //   return;

  // // Nothing left to do if paused.
  // if (_info.paused)
  //   return;

  // // Update joint force
  // //! [jointForceComponent]
  // auto force = _ecm.Component<components::JointForceCmd>(
  //     this->dataPtr->jointEntity);
  // //! [jointForceComponent]

  // std::lock_guard<std::mutex> lock(this->dataPtr->jointForceCmdMutex);

  // //! [modifyComponent]
  // if (force == nullptr)
  // {
  //   _ecm.CreateComponent(
  //       this->dataPtr->jointEntity,
  //       components::JointForceCmd({this->dataPtr->jointForceCmd}));
  // }
  // else
  // {
  //   force->Data()[0] += this->dataPtr->jointForceCmd;
  // }
  // //! [modifyComponent]
}

GZ_ADD_PLUGIN(SetModelState,
                  System,
                  SetModelState::ISystemConfigure,
                  SetModelState::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(SetModelState,
                    "gz::sim::systems::SetModelState")
