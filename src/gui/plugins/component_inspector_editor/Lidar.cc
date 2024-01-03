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
#include <sdf/Lidar.hh>

#include <gz/common/Console.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/EntityComponentManager.hh>

#include "ComponentInspectorEditor.hh"
#include "Lidar.hh"
#include "Types.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
Lidar::Lidar(ComponentInspectorEditor *_inspector)
{
  _inspector->Context()->setContextProperty("LidarImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::GpuLidar>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;
    const sdf::Lidar *lidar = comp->Data().LidarSensor();

    _item->setData(QString("Lidar"),
        ComponentsModel::RoleNames().key("dataType"));
    _item->setData(QList({
      QVariant(lidar->LidarNoise().Mean()),
      QVariant(lidar->LidarNoise().BiasMean()),
      QVariant(lidar->LidarNoise().StdDev()),
      QVariant(lidar->LidarNoise().BiasStdDev()),
      QVariant(lidar->LidarNoise().DynamicBiasStdDev()),
      QVariant(lidar->LidarNoise().DynamicBiasCorrelationTime()),

      QVariant(lidar->HorizontalScanSamples()),
      QVariant(lidar->HorizontalScanResolution()),
      QVariant(lidar->HorizontalScanMinAngle().Radian()),
      QVariant(lidar->HorizontalScanMaxAngle().Radian()),

      QVariant(lidar->VerticalScanSamples()),
      QVariant(lidar->VerticalScanResolution()),
      QVariant(lidar->VerticalScanMinAngle().Radian()),
      QVariant(lidar->VerticalScanMaxAngle().Radian()),

      QVariant(lidar->RangeMin()),
      QVariant(lidar->RangeMax()),
      QVariant(lidar->RangeResolution()),

    }), ComponentsModel::RoleNames().key("data"));
  };

  this->inspector->RegisterComponentCreator(
      components::GpuLidar::typeId, creator);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Lidar::OnLidarNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::GpuLidar>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Lidar *lidar = comp->Data().LidarSensor();
      if (lidar)
      {
        sdf::Noise noise = lidar->LidarNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        lidar->SetLidarNoise(noise);
      }
      else
        gzerr << "Unable to get the lidar noise data.\n";
    }
    else
    {
      gzerr << "Unable to get the lidar component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Lidar::OnLidarChange(
    double _rangeMin, double _rangeMax,
    double _rangeResolution, double _horizontalScanSamples,
    double _horizontalScanResolution,
    double _horizontalScanMinAngle,
    double _horizontalScanMaxAngle, double _verticalScanSamples,
    double _verticalScanResolution, double _verticalScanMinAngle,
    double _verticalScanMaxAngle)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::GpuLidar>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Lidar *lidar = comp->Data().LidarSensor();
      if (lidar)
      {
        lidar->SetRangeMin(_rangeMin);
        lidar->SetRangeMax(_rangeMax);
        lidar->SetRangeResolution(_rangeResolution);

        lidar->SetHorizontalScanSamples(_horizontalScanSamples);
        lidar->SetHorizontalScanResolution(_horizontalScanResolution);
        lidar->SetHorizontalScanMinAngle(_horizontalScanMinAngle);
        lidar->SetHorizontalScanMaxAngle(_horizontalScanMaxAngle);

        lidar->SetVerticalScanSamples(_verticalScanSamples);
        lidar->SetVerticalScanResolution(_verticalScanResolution);
        lidar->SetVerticalScanMinAngle(_verticalScanMinAngle);
        lidar->SetVerticalScanMaxAngle(_verticalScanMaxAngle);
      }
      else
        gzerr << "Unable to get the lidar data.\n";
    }
    else
    {
      gzerr << "Unable to get the lidar component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}
