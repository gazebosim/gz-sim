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
#include <sdf/Camera.hh>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/components/Camera.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include "ComponentInspector.hh"
#include "Camera.hh"
#include "Types.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
Camera::Camera(ComponentInspector *_inspector)
{
  _inspector->Context()->setContextProperty("CameraImpl", this);
  this->inspector = _inspector;

  ComponentCreator creator =
    [=](EntityComponentManager &_ecm, Entity _entity, QStandardItem *_item)
  {
    auto comp = _ecm.Component<components::Camera>(_entity);
    if (nullptr == _item || nullptr == comp)
      return;
    const sdf::Camera *camera = comp->Data().CameraSensor();

    _item->setData(QString("Camera"),
        ComponentsModel::RoleNames().key("dataType"));
    _item->setData(QList({
      QVariant(camera->ImageWidth()),
      QVariant(camera->ImageHeight()),
      QVariant(camera->HorizontalFov().Radian()),
      QVariant(camera->NearClip()),
      QVariant(camera->FarClip()),

      QVariant(camera->ImageNoise().Mean()),
      QVariant(camera->ImageNoise().StdDev()),

      QVariant(camera->DistortionK1()),
      QVariant(camera->DistortionK2()),
      QVariant(camera->DistortionK3()),
      QVariant(camera->DistortionP1()),
      QVariant(camera->DistortionP2()),

      QVariant(camera->DistortionCenter().X()),
      QVariant(camera->DistortionCenter().Y()),

      QVariant(camera->LensC1()),
      QVariant(camera->LensC2()),
      QVariant(camera->LensC3()),
      QVariant(camera->LensFocalLength()),
      QVariant(camera->LensCutoffAngle().Radian()),
      QVariant(camera->LensEnvironmentTextureSize()),
      QVariant(camera->LensIntrinsicsFx()),
      QVariant(camera->LensIntrinsicsFy()),
      QVariant(camera->LensIntrinsicsCx()),
      QVariant(camera->LensIntrinsicsCy()),
      QVariant(camera->LensIntrinsicsSkew()),

    }), ComponentsModel::RoleNames().key("data"));
  };

  this->inspector->RegisterComponentCreator(
      components::Camera::typeId, creator);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Camera::OnImageNoise(double _mean, double _stdDev)
{
  ignition::gazebo::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Camera>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Camera *camera = comp->Data().CameraSensor();
      if (camera)
      {
        sdf::Noise noise = camera->ImageNoise();

        setNoise(noise, _mean, noise.BiasMean(), _stdDev, noise.BiasStdDev(),
            noise.DynamicBiasStdDev(), noise.DynamicBiasCorrelationTime());

        camera->SetImageNoise(noise);
      }
      else
        ignerr << "Unable to get the camera image noise data.\n";
    }
    else
    {
      ignerr << "Unable to get the camera image component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Camera::OnImageSizeChange(int _width, int _height)
{
  ignition::gazebo::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Camera>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Camera *camera = comp->Data().CameraSensor();
      if (camera)
      {
        camera->SetImageWidth(_width);
        camera->SetImageHeight(_height);
      }
      else
        ignerr << "Unable to get the camera image size data.\n";
    }
    else
    {
      ignerr << "Unable to get the camera image component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Camera::OnClipChange(double _near, double _far)
{
  ignition::gazebo::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Camera>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Camera *camera = comp->Data().CameraSensor();
      if (camera)
      {
        camera->SetNearClip(_near);
        camera->SetFarClip(_far);
      }
      else
        ignerr << "Unable to get the camera clip distance data.\n";
    }
    else
    {
      ignerr << "Unable to get the camera image component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}

/////////////////////////////////////////////////
Q_INVOKABLE void Camera::OnHorizontalFovChange(double _hov)
{
  ignition::gazebo::UpdateCallback cb =
      [=](EntityComponentManager &_ecm)
  {
    auto comp = _ecm.Component<components::Camera>(
        this->inspector->GetEntity());
    if (comp)
    {
      sdf::Camera *camera = comp->Data().CameraSensor();
      if (camera)
      {
        camera->SetHorizontalFov(_hov);
      }
      else
        ignerr << "Unable to get the camera horizontal field of view data.\n";
    }
    else
    {
      ignerr << "Unable to get the camera image component.\n";
    }
  };
  this->inspector->AddUpdateCallback(cb);
}
