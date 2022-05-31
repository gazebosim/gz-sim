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
#include <sdf/Magnetometer.hh>

#include <gz/common/Console.hh>
#include <gz/sim/components/Magnetometer.hh>

#include "ComponentInspectorEditor.hh"
#include "Magnetometer.hh"
#include "Types.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
Magnetometer::Magnetometer(ComponentInspectorEditor *_inspector)
{
  _inspector->Context()->setContextProperty("MagnetometerImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::Magnetometer>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;
    const sdf::Magnetometer *mag = comp->Data().MagnetometerSensor();

    _item->setData(QString("Magnetometer"),
        ComponentsModel::RoleNames().key("dataType"));
    _item->setData(QList({
      QVariant(mag->XNoise().Mean()),
      QVariant(mag->XNoise().BiasMean()),
      QVariant(mag->XNoise().StdDev()),
      QVariant(mag->XNoise().BiasStdDev()),
      QVariant(mag->XNoise().DynamicBiasStdDev()),
      QVariant(mag->XNoise().DynamicBiasCorrelationTime()),

      QVariant(mag->YNoise().Mean()),
      QVariant(mag->YNoise().BiasMean()),
      QVariant(mag->YNoise().StdDev()),
      QVariant(mag->YNoise().BiasStdDev()),
      QVariant(mag->YNoise().DynamicBiasStdDev()),
      QVariant(mag->YNoise().DynamicBiasCorrelationTime()),

      QVariant(mag->ZNoise().Mean()),
      QVariant(mag->ZNoise().BiasMean()),
      QVariant(mag->ZNoise().StdDev()),
      QVariant(mag->ZNoise().BiasStdDev()),
      QVariant(mag->ZNoise().DynamicBiasStdDev()),
      QVariant(mag->ZNoise().DynamicBiasCorrelationTime()),

    }), ComponentsModel::RoleNames().key("data"));
  };

  this->inspector->RegisterComponentCreator(
      components::Magnetometer::typeId, creator);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Magnetometer::OnMagnetometerXNoise(
    double _mean, double _meanBias, double _stdDev,
      double _stdDevBias, double _dynamicBiasStdDev,
      double _dynamicBiasCorrelationTime)
{
  UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Magnetometer>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Magnetometer *mag = comp->Data().MagnetometerSensor();
      if (mag)
      {
        sdf::Noise noise = mag->XNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        mag->SetXNoise(noise);
      }
      else
        gzerr << "Unable to get the magnetometer data.\n";
    }
    else
    {
      gzerr << "Unable to get the magnetometer component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Magnetometer::OnMagnetometerYNoise(
    double _mean, double _meanBias, double _stdDev,
      double _stdDevBias, double _dynamicBiasStdDev,
      double _dynamicBiasCorrelationTime)
{
  UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Magnetometer>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Magnetometer *mag = comp->Data().MagnetometerSensor();
      if (mag)
      {
        sdf::Noise noise = mag->YNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        mag->SetYNoise(noise);
      }
      else
        gzerr << "Unable to get the magnetometer data.\n";
    }
    else
    {
      gzerr << "Unable to get the magnetometer component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Magnetometer::OnMagnetometerZNoise(
    double _mean, double _meanBias, double _stdDev,
      double _stdDevBias, double _dynamicBiasStdDev,
      double _dynamicBiasCorrelationTime)
{
  UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Magnetometer>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Magnetometer *mag = comp->Data().MagnetometerSensor();
      if (mag)
      {
        sdf::Noise noise = mag->ZNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        mag->SetZNoise(noise);
      }
      else
        gzerr << "Unable to get the magnetometer data.\n";
    }
    else
    {
      gzerr << "Unable to get the magnetometer component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}
