/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gz/math/Inertial.hh>
#include <gz/sim/components/Inertial.hh>

#include "Inertial.hh"

using namespace gz;
using namespace sim;
using namespace inspector;

/////////////////////////////////////////////////
Inertial::Inertial(ComponentInspector *_inspector)
{
  _inspector->Context()->setContextProperty("InertialImpl", this);
  this->inspector = _inspector;

  this->inspector->AddUpdateViewCb(components::Inertial::typeId,
      std::bind(&Inertial::UpdateView, this,
      std::placeholders::_1, std::placeholders::_2));
}

/////////////////////////////////////////////////
void Inertial::UpdateView(const EntityComponentManager &_ecm,
    QStandardItem *_item)
{
  auto comp = _ecm.Component<components::Inertial>(
      this->inspector->GetEntity());
  if (nullptr == _item || nullptr == comp)
    return;

  auto inertial = comp->Data();

  _item->setData(QString("Inertial"),
      ComponentsModel::RoleNames().key("dataType"));

  QList<QVariant> dataList = {
    QVariant(inertial.Pose().Pos().X()),
    QVariant(inertial.Pose().Pos().Y()),
    QVariant(inertial.Pose().Pos().Z()),
    QVariant(inertial.Pose().Rot().Roll()),
    QVariant(inertial.Pose().Rot().Pitch()),
    QVariant(inertial.Pose().Rot().Yaw()),
    QVariant(inertial.MassMatrix().Mass()),
    QVariant(inertial.MassMatrix().Ixx()),
    QVariant(inertial.MassMatrix().Ixy()),
    QVariant(inertial.MassMatrix().Ixz()),
    QVariant(inertial.MassMatrix().Iyy()),
    QVariant(inertial.MassMatrix().Iyz()),
    QVariant(inertial.MassMatrix().Izz())
  };

  if (inertial.FluidAddedMass().has_value())
  {
    dataList.append(QVariant(inertial.FluidAddedMass().value()(0, 0)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(0, 1)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(0, 2)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(0, 3)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(0, 4)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(0, 5)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(1, 1)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(1, 2)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(1, 3)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(1, 4)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(1, 5)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(2, 2)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(2, 3)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(2, 4)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(2, 5)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(3, 3)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(3, 4)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(3, 5)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(4, 4)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(4, 5)));
    dataList.append(QVariant(inertial.FluidAddedMass().value()(5, 5)));
  }

  _item->setData(dataList, ComponentsModel::RoleNames().key("data"));
}
