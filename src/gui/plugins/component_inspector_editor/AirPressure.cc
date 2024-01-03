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
#include <sdf/AirPressure.hh>

#include <gz/common/Console.hh>
#include <gz/sim/components/AirPressureSensor.hh>
#include <gz/sim/EntityComponentManager.hh>

#include "AirPressure.hh"
#include "ComponentInspectorEditor.hh"
#include "Types.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
AirPressure::AirPressure(ComponentInspectorEditor *_inspector)
{
  _inspector->Context()->setContextProperty("AirPressureImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::AirPressureSensor>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;
    const sdf::AirPressure *air = comp->Data().AirPressureSensor();

    _item->setData(QString("AirPressure"),
        ComponentsModel::RoleNames().key("dataType"));
    _item->setData(QList({
      QVariant(air->ReferenceAltitude()),
      QVariant(air->PressureNoise().Mean()),
      QVariant(air->PressureNoise().BiasMean()),
      QVariant(air->PressureNoise().StdDev()),
      QVariant(air->PressureNoise().BiasStdDev()),
      QVariant(air->PressureNoise().DynamicBiasStdDev()),
      QVariant(air->PressureNoise().DynamicBiasCorrelationTime()),
    }), ComponentsModel::RoleNames().key("data"));
  };

  this->inspector->RegisterComponentCreator(
      components::AirPressureSensor::typeId, creator);
}

/////////////////////////////////////////////////
Q_INVOKABLE void AirPressure::OnAirPressureNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::AirPressureSensor>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::AirPressure *airpressure = comp->Data().AirPressureSensor();
      if (airpressure)
      {
        sdf::Noise noise = airpressure->PressureNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        airpressure->SetPressureNoise(noise);
      }
      else
        gzerr << "Unable to get the air pressure data.\n";
    }
    else
    {
      gzerr << "Unable to get the air pressure component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void AirPressure::OnAirPressureReferenceAltitude(
    double _referenceAltitude)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::AirPressureSensor>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::AirPressure *airpressure = comp->Data().AirPressureSensor();
      if (airpressure)
      {
        airpressure->SetReferenceAltitude(_referenceAltitude);
      }
      else
        gzerr << "Unable to get the air pressure data.\n";
    }
    else
    {
      gzerr << "Unable to get the air pressure component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}
