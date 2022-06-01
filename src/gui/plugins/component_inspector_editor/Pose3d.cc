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
#include <gz/math/Pose3.hh>

#include <gz/common/Console.hh>
#include <gz/sim/Util.hh>
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Recreate.hh"
#include <gz/sim/EntityComponentManager.hh>

#include "ComponentInspectorEditor.hh"
#include "Pose3d.hh"
#include "Types.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
Pose3d::Pose3d(ComponentInspectorEditor *_inspector)
{
  _inspector->Context()->setContextProperty("Pose3dImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::Pose>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;
    math::Pose3d pose = comp->Data();

    _item->setData(QString("Pose3d"),
        ComponentsModel::RoleNames().key("dataType"));
    _item->setData(QList({
      QVariant(pose.Pos().X()),
      QVariant(pose.Pos().Y()),
      QVariant(pose.Pos().Z()),
      QVariant(pose.Rot().Roll()),
      QVariant(pose.Rot().Pitch()),
      QVariant(pose.Rot().Yaw())
    }), ComponentsModel::RoleNames().key("data"));

    // Notify the qml component when the pose has changed, such as when
    // a new entity is selected.
    if (this->currentPose != pose)
    {
      this->currentPose = pose;
      this->poseChanged();
    }
  };

  this->inspector->RegisterComponentCreator(components::Pose::typeId, creator);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Pose3d::PoseUpdate(
    double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Pose>(this->inspector->GetEntity());
    if (comp)
    {
      comp->Data().Set(_x, _y, _z, _roll, _pitch, _yaw);
      // Recreate the model. Changing link poses can cause the kinematic
      // tree to need regreneration.
      Entity modelEntity = topLevelModel(this->inspector->GetEntity(), _ecm);
      _ecm.CreateComponent(modelEntity, components::Recreate());
    }
    else
      gzerr << "Unable to get the pose component.\n";
  };
  this->inspector->AddUpdateCallback(cb);
}
