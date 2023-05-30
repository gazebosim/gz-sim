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
#include <sdf/Altimeter.hh>

#include <gz/common/Console.hh>
#include <gz/sim/components/Altimeter.hh>

#include "Altimeter.hh"
#include "ComponentInspectorEditor.hh"
#include "Types.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
Altimeter::Altimeter(ComponentInspectorEditor *_inspector)
{
  _inspector->Context()->setContextProperty("AltimeterImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::Altimeter>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;
    const sdf::Altimeter *alt = comp->Data().AltimeterSensor();

    _item->setData(QString("Altimeter"),
        ComponentsModel::RoleNames().key("dataType"));
    _item->setData(QList({
      QVariant(alt->VerticalPositionNoise().Mean()),
      QVariant(alt->VerticalPositionNoise().BiasMean()),
      QVariant(alt->VerticalPositionNoise().StdDev()),
      QVariant(alt->VerticalPositionNoise().BiasStdDev()),
      QVariant(alt->VerticalPositionNoise().DynamicBiasStdDev()),
      QVariant(alt->VerticalPositionNoise().DynamicBiasCorrelationTime()),

      QVariant(alt->VerticalVelocityNoise().Mean()),
      QVariant(alt->VerticalVelocityNoise().BiasMean()),
      QVariant(alt->VerticalVelocityNoise().StdDev()),
      QVariant(alt->VerticalVelocityNoise().BiasStdDev()),
      QVariant(alt->VerticalVelocityNoise().DynamicBiasStdDev()),
      QVariant(alt->VerticalVelocityNoise().DynamicBiasCorrelationTime()),
    }), ComponentsModel::RoleNames().key("data"));
  };

  this->inspector->RegisterComponentCreator(
      components::Altimeter::typeId, creator);
}
/////////////////////////////////////////////////
Q_INVOKABLE void Altimeter::OnAltimeterPositionNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Altimeter>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Altimeter *altimeter = comp->Data().AltimeterSensor();
      if (altimeter)
      {
        sdf::Noise noise = altimeter->VerticalPositionNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        altimeter->SetVerticalPositionNoise(noise);
      }
      else
        gzerr << "Unable to get the altimeter data.\n";
    }
    else
    {
      gzerr << "Unable to get the altimeter component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Altimeter::OnAltimeterVelocityNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Altimeter>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Altimeter *altimeter = comp->Data().AltimeterSensor();
      if (altimeter)
      {
        sdf::Noise noise = altimeter->VerticalVelocityNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        altimeter->SetVerticalVelocityNoise(noise);
      }
      else
        gzerr << "Unable to get the altimeter data.\n";
    }
    else
    {
      gzerr << "Unable to get the altimeter component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}
