/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "SimplifiedPhysicsModel.hh"

#include <string>

#include <gz/common/Console.hh>
#include <sdf/Element.hh>

#include "PhysicsMode.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/SupportedPhysicsFeatures.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace systems::simplified_physics;

namespace
{
/// \brief Keep a top-level model on the default Physics system.
///
/// The world can name another provider as its default owner. A model that
/// chose `full` (or whose request was refused) must not follow that default,
/// so it gets an empty PhysicsOwner, which takes precedence over the world's.
/// It also drops the features that the world's provider may have listed for it
/// (see SimplifiedPhysics): Physics supports everything, and a model without
/// the component is taken to do so.
///
/// \param[in] _ecm Mutable reference to the ECM.
/// \param[in] _model Model entity.
void KeepOnPhysics(EntityComponentManager &_ecm, const Entity &_model)
{
  // Nested models follow their top-level model.
  if (topLevelModel(_model, _ecm) != _model)
    return;

  // Nothing to undo, and nothing added: an empty PhysicsOwner on every model
  // would make Physics resolve ownership one entity at a time.
  if (physicsOwner(_model, _ecm).empty())
    return;

  _ecm.SetComponentData<components::PhysicsOwner>(_model, "");
  _ecm.RemoveComponent<components::SupportedPhysicsFeatures>(_model);
}
}

//////////////////////////////////////////////////
SimplifiedPhysicsModel::SimplifiedPhysicsModel() = default;

//////////////////////////////////////////////////
SimplifiedPhysicsModel::~SimplifiedPhysicsModel() = default;

//////////////////////////////////////////////////
void SimplifiedPhysicsModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  const auto name = _ecm.ComponentData<components::Name>(_entity).value_or("");

  if (!_ecm.Component<components::Model>(_entity))
  {
    gzerr << "SimplifiedPhysicsModel must be attached to a model, but it is "
          << "attached to entity [" << _entity << "]. The plugin is ignored."
          << std::endl;
    return;
  }

  PhysicsMode mode{PhysicsMode::Full};
  if (_sdf->HasElement("mode"))
  {
    const auto value = _sdf->Get<std::string>("mode");
    if (!ParseMode(value, mode))
    {
      gzwarn << "Model [" << name << "] has unknown physics mode [" << value
             << "]. Known modes are [full], [planar_kinematic] and "
             << "[planar_dynamic]. The model stays on [full] physics."
             << std::endl;
      KeepOnPhysics(_ecm, _entity);
      return;
    }
  }

  if (mode == PhysicsMode::Full)
  {
    KeepOnPhysics(_ecm, _entity);
    return;
  }

  if (mode == PhysicsMode::PlanarDynamic)
  {
    gzwarn << "Model [" << name << "] asks for physics mode ["
           << ModeName(mode) << "], which is not implemented yet. The model "
           << "stays on [full] physics." << std::endl;
    KeepOnPhysics(_ecm, _entity);
    return;
  }

  if (topLevelModel(_entity, _ecm) != _entity)
  {
    gzwarn << "Model [" << name << "] is nested in another model. Only a "
           << "top-level model can choose its physics mode, so the model "
           << "stays on [full] physics." << std::endl;
    return;
  }

  // A static model never moves, so there is nothing for a mode to simulate.
  const auto isStatic = _ecm.ComponentData<components::Static>(_entity);
  if (isStatic.has_value() && *isStatic)
  {
    gzwarn << "Model [" << name << "] is static, so physics mode ["
           << ModeName(mode) << "] has no effect. The model stays on [full] "
           << "physics." << std::endl;
    KeepOnPhysics(_ecm, _entity);
    return;
  }

  // Both components must exist before the next step: the default Physics
  // system reads the owner when it creates the model, and the system manager
  // reads the features when it activates the model's other plugins.
  _ecm.SetComponentData<components::PhysicsOwner>(_entity, kProviderName);
  _ecm.SetComponentData<components::SupportedPhysicsFeatures>(_entity,
      SupportedFeatures(mode));
}
