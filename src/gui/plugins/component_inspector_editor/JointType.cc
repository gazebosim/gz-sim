/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <sdf/Joint.hh>

#include <gz/common/Console.hh>
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Recreate.hh"
#include <gz/sim/EntityComponentManager.hh>

#include "JointType.hh"
#include "ComponentInspectorEditor.hh"
#include "Types.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
JointType::JointType(ComponentInspectorEditor *_inspector)
{
  _inspector->Context()->setContextProperty("JointTypeImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::JointType>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;

    const sdf::JointType joint = comp->Data();

    _item->setData(QString("JointType"),
        ComponentsModel::RoleNames().key("dataType"));

    QString jointType;
    if (joint == sdf::JointType::BALL)
      jointType = "Ball";
    else if (joint == sdf::JointType::CONTINUOUS)
      jointType = "Continuous";
    else if (joint == sdf::JointType::FIXED)
      jointType = "Fixed";
    else if (joint == sdf::JointType::GEARBOX)
      jointType = "Gearbox";
    else if (joint == sdf::JointType::PRISMATIC)
      jointType = "Prismatic";
    else if (joint == sdf::JointType::REVOLUTE)
      jointType = "Revolute";
    else if (joint == sdf::JointType::REVOLUTE2)
      jointType = "Revolute2";
    else if (joint == sdf::JointType::SCREW)
      jointType = "Screw";
    else if (joint == sdf::JointType::UNIVERSAL)
      jointType = "Universal";

    _item->setData(jointType, ComponentsModel::RoleNames().key("data"));
  };

  this->inspector->RegisterComponentCreator(
      components::JointType::typeId, creator);
}

/////////////////////////////////////////////////
Q_INVOKABLE void JointType::OnJointType(QString _jointType)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    components::JointType *comp =
      _ecm.Component<components::JointType>(this->inspector->GetEntity());

    components::ParentEntity *parentComp =
      _ecm.Component<components::ParentEntity>(this->inspector->GetEntity());

    if (comp && parentComp)
    {
      if (_jointType == "Ball")
        comp->Data() = sdf::JointType::BALL;
      else if (_jointType == "Continuous")
        comp->Data() = sdf::JointType::CONTINUOUS;
      else if (_jointType == "Fixed")
        comp->Data() = sdf::JointType::FIXED;
      else if (_jointType == "Gearbox")
        comp->Data() = sdf::JointType::GEARBOX;
      else if (_jointType == "Prismatic")
        comp->Data() = sdf::JointType::PRISMATIC;
      else if (_jointType == "Revolute")
        comp->Data() = sdf::JointType::REVOLUTE;
      else if (_jointType == "Revolute2")
        comp->Data() = sdf::JointType::REVOLUTE2;
      else if (_jointType == "Screw")
        comp->Data() = sdf::JointType::SCREW;
      else if (_jointType == "Universal")
        comp->Data() = sdf::JointType::UNIVERSAL;

      // Make sure to mark the parent as needing recreation. This will
      // tell the server to rebuild the model with the new link.
      _ecm.CreateComponent(parentComp->Data(), components::Recreate());
    }
    else if (!comp)
    {
      gzerr << "Unable to get the joint type component.\n";
    }
    else
    {
      gzerr << "Unable to get the joint's parent entity component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}
