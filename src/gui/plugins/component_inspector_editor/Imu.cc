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
#include <sdf/Imu.hh>

#include <gz/common/Console.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/EntityComponentManager.hh>

#include "ComponentInspectorEditor.hh"
#include "Imu.hh"
#include "Types.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
Imu::Imu(ComponentInspectorEditor *_inspector)
{
  _inspector->Context()->setContextProperty("ImuImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::Imu>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;
    const sdf::Imu *imu = comp->Data().ImuSensor();

    _item->setData(QString("Imu"),
        ComponentsModel::RoleNames().key("dataType"));
    _item->setData(QList({
      QVariant(imu->LinearAccelerationXNoise().Mean()),
      QVariant(imu->LinearAccelerationXNoise().BiasMean()),
      QVariant(imu->LinearAccelerationXNoise().StdDev()),
      QVariant(imu->LinearAccelerationXNoise().BiasStdDev()),
      QVariant(imu->LinearAccelerationXNoise().DynamicBiasStdDev()),
      QVariant(imu->LinearAccelerationXNoise().DynamicBiasCorrelationTime()),

      QVariant(imu->LinearAccelerationYNoise().Mean()),
      QVariant(imu->LinearAccelerationYNoise().BiasMean()),
      QVariant(imu->LinearAccelerationYNoise().StdDev()),
      QVariant(imu->LinearAccelerationYNoise().BiasStdDev()),
      QVariant(imu->LinearAccelerationYNoise().DynamicBiasStdDev()),
      QVariant(imu->LinearAccelerationYNoise().DynamicBiasCorrelationTime()),

      QVariant(imu->LinearAccelerationZNoise().Mean()),
      QVariant(imu->LinearAccelerationZNoise().BiasMean()),
      QVariant(imu->LinearAccelerationZNoise().StdDev()),
      QVariant(imu->LinearAccelerationZNoise().BiasStdDev()),
      QVariant(imu->LinearAccelerationZNoise().DynamicBiasStdDev()),
      QVariant(imu->LinearAccelerationZNoise().DynamicBiasCorrelationTime()),

      QVariant(imu->AngularVelocityXNoise().Mean()),
      QVariant(imu->AngularVelocityXNoise().BiasMean()),
      QVariant(imu->AngularVelocityXNoise().StdDev()),
      QVariant(imu->AngularVelocityXNoise().BiasStdDev()),
      QVariant(imu->AngularVelocityXNoise().DynamicBiasStdDev()),
      QVariant(imu->AngularVelocityXNoise().DynamicBiasCorrelationTime()),

      QVariant(imu->AngularVelocityYNoise().Mean()),
      QVariant(imu->AngularVelocityYNoise().BiasMean()),
      QVariant(imu->AngularVelocityYNoise().StdDev()),
      QVariant(imu->AngularVelocityYNoise().BiasStdDev()),
      QVariant(imu->AngularVelocityYNoise().DynamicBiasStdDev()),
      QVariant(imu->AngularVelocityYNoise().DynamicBiasCorrelationTime()),

      QVariant(imu->AngularVelocityZNoise().Mean()),
      QVariant(imu->AngularVelocityZNoise().BiasMean()),
      QVariant(imu->AngularVelocityZNoise().StdDev()),
      QVariant(imu->AngularVelocityZNoise().BiasStdDev()),
      QVariant(imu->AngularVelocityZNoise().DynamicBiasStdDev()),
      QVariant(imu->AngularVelocityZNoise().DynamicBiasCorrelationTime()),

    }), ComponentsModel::RoleNames().key("data"));
  };

  this->inspector->RegisterComponentCreator(
      components::Imu::typeId, creator);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Imu::OnLinearAccelerationXNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Imu>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Imu *imu = comp->Data().ImuSensor();
      if (imu)
      {
        sdf::Noise noise = imu->LinearAccelerationXNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        imu->SetLinearAccelerationXNoise(noise);
      }
      else
        gzerr << "Unable to get the imu linear acceleration x noise data.\n";
    }
    else
    {
      gzerr << "Unable to get the imu component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Imu::OnLinearAccelerationYNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Imu>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Imu *imu = comp->Data().ImuSensor();
      if (imu)
      {
        sdf::Noise noise = imu->LinearAccelerationYNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        imu->SetLinearAccelerationYNoise(noise);
      }
      else
        gzerr << "Unable to get the imu linear acceleration y noise data.\n";
    }
    else
    {
      gzerr << "Unable to get the imu component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Imu::OnLinearAccelerationZNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Imu>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Imu *imu = comp->Data().ImuSensor();
      if (imu)
      {
        sdf::Noise noise = imu->LinearAccelerationZNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        imu->SetLinearAccelerationZNoise(noise);
      }
      else
        gzerr << "Unable to get the imu linear acceleration z noise data.\n";
    }
    else
    {
      gzerr << "Unable to get the imu component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Imu::OnAngularVelocityXNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Imu>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Imu *imu = comp->Data().ImuSensor();
      if (imu)
      {
        sdf::Noise noise = imu->AngularVelocityXNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        imu->SetAngularVelocityXNoise(noise);
      }
      else
        gzerr << "Unable to get the imu angular velocity x noise data.\n";
    }
    else
    {
      gzerr << "Unable to get the imu component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Imu::OnAngularVelocityYNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Imu>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Imu *imu = comp->Data().ImuSensor();
      if (imu)
      {
        sdf::Noise noise = imu->AngularVelocityYNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        imu->SetAngularVelocityYNoise(noise);
      }
      else
        gzerr << "Unable to get the imu angular velocity y noise data.\n";
    }
    else
    {
      gzerr << "Unable to get the imu component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Imu::OnAngularVelocityZNoise(
    double _mean, double _meanBias, double _stdDev,
    double _stdDevBias, double _dynamicBiasStdDev,
    double _dynamicBiasCorrelationTime)
{
  gz::sim::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Imu>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Imu *imu = comp->Data().ImuSensor();
      if (imu)
      {
        sdf::Noise noise = imu->AngularVelocityZNoise();

        setNoise(noise, _mean, _meanBias, _stdDev, _stdDevBias,
            _dynamicBiasStdDev, _dynamicBiasCorrelationTime);

        imu->SetAngularVelocityZNoise(noise);
      }
      else
        gzerr << "Unable to get the imu angular velocity z noise data.\n";
    }
    else
    {
      gzerr << "Unable to get the imu component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}
