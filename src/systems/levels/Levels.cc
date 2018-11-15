/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <algorithm>
#include <iostream>
#include <random>

#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Box.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>
#include <ignition/plugin/Register.hh>

// SDF
#include <sdf/Collision.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
// Components
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/NameList.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/RenderState.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include "Levels.hh"

using namespace ignition::gazebo::systems;
namespace components = ignition::gazebo::components;


// Private data class.
class ignition::gazebo::systems::LevelsPrivate
{
  public: void UpdateLevels(EntityComponentManager &_ecm,
                            const math::AxisAlignedBox &_performerVolume,
                            std::set<EntityId> &_activeLevels);
};

//////////////////////////////////////////////////
Levels::Levels() : System(), dataPtr(std::make_unique<LevelsPrivate>())
{
}

//////////////////////////////////////////////////
Levels::~Levels()
{
}

//////////////////////////////////////////////////
void Levels::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;


  // create a list of active models
  std::set<EntityId> activeLevels;
  _ecm.Each<components::Performer, components::Geometry,
            components::ParentEntity>(
      [&](const EntityId &,
          const components::Performer *,
          const components::Geometry *_geometry,
          const components::ParentEntity * _parent) -> bool
      {
        auto pose = _ecm.Component<components::Pose>(_parent->Data());
        // We assume the geometry contains a box.
        auto box = _geometry->Data().BoxShape();

        math::AxisAlignedBox performerVolume{
             pose->Data().Pos() - box->Size() / 2,
             pose->Data().Pos() + box->Size() / 2};

        this->dataPtr->UpdateLevels(_ecm, performerVolume, activeLevels);
        return true;
      });

  // create a list of active models
  std::set<std::string> activeModels;
  std::set<std::string> inactiveModels;
  _ecm.Each<components::Level, components::NameList>(
      [&](const EntityId &_entity, const components::Level *,
          const components::NameList *_modelNames) -> bool
      {
        if (activeLevels.find(_entity) != activeLevels.end())
        {
          activeModels.insert(_modelNames->Data().begin(),
                              _modelNames->Data().end());
        }
        else
        {
          inactiveModels.insert(_modelNames->Data().begin(),
                                _modelNames->Data().end());
        }
        return true;
      });


  _ecm.Each<components::Model, components::Name, components::RenderState>(
      [&](const EntityId &,
          const components::Model *,
          const components::Name *_name,
          components::RenderState *_renderState)->bool
      {
        if (activeModels.find(_name->Data()) != activeModels.end())
        {
          *_renderState = components::RenderState(true);
        }
        else if (inactiveModels.find(_name->Data()) != inactiveModels.end())
        {
          *_renderState = components::RenderState(false);
        }

        return true;
      });
}

void LevelsPrivate::UpdateLevels(EntityComponentManager &_ecm,
                                 const math::AxisAlignedBox &_performerVolume,
                                 std::set<EntityId> &_activeLevels)
{
  _ecm.Each<components::Level, components::Pose, components::Geometry>(
      [&](const EntityId &_entity, const components::Level *,
          const components::Pose *_pose,
          const components::Geometry *_geometry) -> bool
      {
        // Check if the performer is in this level
        // assume a box for now
        auto box = _geometry->Data().BoxShape();
        auto center = _pose->Data().Pos();
        math::AxisAlignedBox region{center - box->Size() / 2,
                                    center + box->Size() / 2};

        if (region.Intersects(_performerVolume))
        {
          _activeLevels.insert(_entity);
        }
        return true;
      });

}
IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Levels,
                    ignition::gazebo::System,
                    Levels::ISystemPreUpdate)
