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
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/LinearVelocityReset.hh"
#include "gz/sim/components/AngularVelocityReset.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

namespace
{
  bool parseScalarWithDegrees(math::Angle &_scalar, sdf::ElementConstPtr _elem)
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
  }

  bool parseVectorWithDegree(math::Vector3d &vector, sdf::ElementConstPtr _elem)
  {
    if(_elem)
    {
      // parse degree attribute, default false
      std::pair<bool, bool> degreesPair = _elem->Get<bool>("degrees", false);
      // parse element vector, default math::Vector3d::Zero
      std::pair<math::Vector3d, bool> vectorPair = _elem->Get<math::Vector3d>
                                                  ("", math::Vector3d::Zero);
      if(vectorPair.second)
      {
        if(degreesPair.first)
        {
          vector.Set(GZ_DTOR(vectorPair.first.X()),
                 GZ_DTOR(vectorPair.first.Y()), GZ_DTOR(vectorPair.first.Z()));
        }
        else
        {
          vector = vectorPair.first;
        }
        return true;
      }
    }
    return false;
  }
}

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

  auto modelStateElem = _sdf->FindElement("model_state");
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
  for (auto linkStateElem = modelStateElem->FindElement("link_state");
       linkStateElem != nullptr;
       linkStateElem = linkStateElem->GetNextElement("link_state"))
  {

    std::pair<std::string, bool> namePair =
         linkStateElem->Get<std::string>("name", "");
    if (!namePair.second)
    {
      gzerr << "No name specified for link_state, skipping.\n";
      continue;
    }
    const auto &linkName = namePair.first;

    Entity linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
    if (linkEntity == kNullEntity)
    {
      gzerr << "Unable to find link with name [" << linkName << "] "
            << "in model with name [" << modelName << "], skipping.\n";
      continue;
    }

    math::Vector3d linearVelocity;
    auto linearVelocityElem = linkStateElem->FindElement("linear_velocity");

    if(linearVelocityElem)
    {
     std::pair<math::Vector3d, bool> vectorPair =
     linearVelocityElem->Get<math::Vector3d>("", math::Vector3d::Zero);
      if (vectorPair.second)
      {
        linearVelocity = vectorPair.first;
        _ecm.SetComponentData<components::WorldLinearVelocityReset>(
                                          linkEntity, linearVelocity);
      }
    }

    math::Vector3d angularVelocity;
    auto angularVelocityElem =
                     linkStateElem->FindElement("angular_velocity");

    if(angularVelocityElem){
      bool parsedVector = parseVectorWithDegree(angularVelocity,
                                                angularVelocityElem);
      if(parsedVector)
      {
         _ecm.SetComponentData<components::WorldAngularVelocityReset>(
                                           linkEntity, angularVelocity);
      }
    }
  }
  // \TODO(yaswanth1701) set reset velocity components in body frame.
}

//////////////////////////////////////////////////
void SetModelState::Reset(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("SetModelState::Reset");

  // \TODO(scpeters) Set the same state as in Configure
}

GZ_ADD_PLUGIN(SetModelState,
                  System,
                  SetModelState::ISystemConfigure,
                  SetModelState::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(SetModelState,
                    "gz::sim::systems::SetModelState")
