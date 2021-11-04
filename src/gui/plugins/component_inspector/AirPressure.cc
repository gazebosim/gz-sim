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

#include <ignition/common/Console.hh>
#include <ignition/gazebo/components/AirPressureSensor.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include "AirPressure.hh"

//////////////////////////////////////////////////
template<>
void ignition::gazebo::setData(QStandardItem *_item,
    const sdf::AirPressure &_airpressure)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("AirPressure"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QList({
    QVariant(_airpressure.PressureNoise().Mean()),
    QVariant(_airpressure.PressureNoise().BiasMean()),
    QVariant(_airpressure.PressureNoise().StdDev()),
    QVariant(_airpressure.PressureNoise().BiasStdDev()),
    QVariant(_airpressure.PressureNoise().DynamicBiasStdDev()),
    QVariant(_airpressure.PressureNoise().DynamicBiasCorrelationTime()),
  }), ComponentsModel::RoleNames().key("data"));
}

/////////////////////////////////////////////////
ignition::gazebo::UpdateCallback ignition::gazebo::onAirPressureNoise(
    Entity _entity, double _mean, double _meanBias, double _stdDev,
      double _stdDevBias, double _dynamicBiasStdDev,
      double _dynamicBiasCorrelationTime)
{
  ignition::gazebo::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::AirPressureSensor>(_entity);
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
        ignerr << "Unable to get the air pressure data.\n";
    }
    else
    {
      ignerr << "Unable to get the air pressure component.\n";
    }
  };
  return cb;
}

/////////////////////////////////////////////////
ignition::gazebo::UpdateCallback
ignition::gazebo::onAirPressureReferenceAltitude(
    Entity _entity, double _referenceAltitude)
{
  ignition::gazebo::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::AirPressureSensor>(_entity);
    if (comp)
    {
      sdf::AirPressure *airpressure = comp->Data().AirPressureSensor();
      if (airpressure)
      {
        airpressure->SetReferenceAltitude(_referenceAltitude);
      }
      else
        ignerr << "Unable to get the air pressure data.\n";
    }
    else
    {
      ignerr << "Unable to get the air pressure component.\n";
    }
  };
  return cb;
}

