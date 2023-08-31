/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <memory>

#include <gz/common/Console.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/rendering/ArrowVisual.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/rendering/WrenchVisualizer.hh"

using namespace gz;
using namespace sim;
using namespace detail;

/// Private data for the WrenchVisualizer class
class gz::sim::detail::WrenchVisualizer::Implementation
{
  /// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene{nullptr};
};

/////////////////////////////////////////////////
WrenchVisualizer::WrenchVisualizer() :
  dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

/////////////////////////////////////////////////
WrenchVisualizer::~WrenchVisualizer() = default;

/////////////////////////////////////////////////
bool WrenchVisualizer::Init(rendering::ScenePtr _scene)
{
  if (!_scene)
  {
    return false;
  }
  this->dataPtr->scene = _scene;
  return true;
}

/////////////////////////////////////////////////
rendering::ArrowVisualPtr WrenchVisualizer::CreateForceVisual(
  rendering::MaterialPtr _material)
{
  if (!this->dataPtr->scene)
  {
    gzwarn << "WrenchVisualizer not initialized" << std::endl;
    return rendering::ArrowVisualPtr();
  }
  rendering::ArrowVisualPtr forceVisual =
    this->dataPtr->scene->CreateArrowVisual();
  forceVisual->SetMaterial(_material);
  forceVisual->ShowArrowHead(true);
  forceVisual->ShowArrowShaft(true);
  forceVisual->ShowArrowRotation(false);
  return forceVisual;
}

/////////////////////////////////////////////////
rendering::VisualPtr WrenchVisualizer::CreateTorqueVisual(
  rendering::MaterialPtr _material)
{
  if (!this->dataPtr->scene)
  {
    gzwarn << "WrenchVisualizer not initialized" << std::endl;
    return rendering::ArrowVisualPtr();
  }
  auto meshMgr = common::MeshManager::Instance();
  std::string meshName{"torque_tube"};
  if (!meshMgr->HasMesh(meshName))
    meshMgr->CreateTube(meshName, 0.28f, 0.3f, 0.2f, 1, 32);
  auto torqueTube = this->dataPtr->scene->CreateVisual();
  torqueTube->AddGeometry(this->dataPtr->scene->CreateMesh(meshName));
  torqueTube->SetOrigin(0, 0, -0.9f);
  torqueTube->SetLocalPosition(0, 0, 0);

  auto cylinder = this->dataPtr->scene->CreateVisual();
  cylinder->AddGeometry(this->dataPtr->scene->CreateCylinder());
  cylinder->SetOrigin(0, 0, -0.5);
  cylinder->SetLocalScale(0.01, 0.01, 0.8);
  cylinder->SetLocalPosition(0, 0, 0);

  auto torqueVisual = this->dataPtr->scene->CreateVisual();
  torqueVisual->AddChild(torqueTube);
  torqueVisual->AddChild(cylinder);
  torqueVisual->SetMaterial(_material);
  return torqueVisual;
}

/////////////////////////////////////////////////
void WrenchVisualizer::UpdateVectorVisual(rendering::VisualPtr _visual,
                                          const math::Vector3d &_direction,
                                          const math::Vector3d &_position,
                                          const double _size,
                                          const bool _tip)
{
  math::Quaterniond quat;
  quat.SetFrom2Axes(math::Vector3d::UnitZ, _direction);
  if (_tip)
  {
    _visual->SetLocalPosition(
      _position - 0.75 * _size * _direction.Normalized());
  }
  else
  {
    _visual->SetLocalPosition(_position);
  }
  _visual->SetLocalRotation(quat);
  _visual->SetLocalScale(_size);
}
