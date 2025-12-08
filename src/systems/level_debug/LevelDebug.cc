/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "LevelDebug.hh"

#include <cmath>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Performer.hh"
#include "gz/sim/components/PerformerAffinity.hh"
#include "gz/sim/components/PerformerLevels.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LevelDebugPrivate
{
  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
LevelDebug::LevelDebug()
    : System(), dataPtr(std::make_unique<LevelDebugPrivate>())
{
}

//////////////////////////////////////////////////
void LevelDebug::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The LevelDebug system should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void LevelDebug::PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
  GZ_PROFILE("LevelDebug::PreUpdate");

  //! @todo - all of these checks could be run at load
  // and re-run only of the affinity changes. The expensive
  // check for ChildrenByComponents only needs to run at configure/load.

  std::string modelName = this->dataPtr->model.Name(_ecm);

  if (_info.secondaryNamespace.empty())
  {
    gzmsg << "System is not a NetworkManagerSecondary"
          <<  std::endl;
    return;
  }

  // Is this model a performer?
  //! @note expensive - returns a std::vector by value. Move to Configure.
  auto performerEntities = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::Performer());

  if (performerEntities.empty())
  {
    gzmsg << "Model: [" << modelName << "] "
          << "is not a performer" <<  std::endl;
    return;
  }

  if (performerEntities.size() > 1)
  {
    gzerr << "Model: [" << modelName << "] "
          << "is associated with performers" <<  std::endl;
    return;
  }

  auto performerEntity = performerEntities[0];
  auto performerAffinity =
      _ecm.Component<components::PerformerAffinity>(performerEntity);
  auto performerRef =
      _ecm.Component<components::PerformerRef>(performerEntity);
  auto PerformerLevels =
      _ecm.Component<components::PerformerLevels>(performerEntity);
  auto performerName =
      _ecm.Component<components::Name>(performerEntity);

  if (performerName == nullptr || performerRef == nullptr)
  {
    gzerr << "Model: [" << modelName << "] "
          << "has an incorrectly configured performer"
          << std::endl;
    return;
  }

  if (performerAffinity == nullptr ||
      performerAffinity->Data() != _info.secondaryNamespace)
  {
    gzmsg << "Model: [" << modelName << "] "
          << "is not updated by Secondary: ["
          << _info.secondaryNamespace << "]"
          << std::endl;
    return;
  }

  {
    gzmsg << "Name: [" << performerName->Data() << "] "
          << std::endl;
  }

  {
    gzmsg << "Ref: [" << performerRef->Data() << "] "
          << std::endl;
  }

  {
    gzmsg << "Affinity: [" << performerAffinity->Data() << "] "
          << std::endl;
  }

  // List all the model components in this ECM instance
  gzmsg << "All Models:" << std::endl;
  auto modelEntities = _ecm.EntitiesByComponents(components::Model());
  for (const auto &entity : modelEntities)
  {
    auto model = Model(entity);
    auto name = model.Name(_ecm);
    gzmsg << "[" << entity << "] " << name << std::endl;
  }
}

GZ_ADD_PLUGIN(LevelDebug,
              System,
              LevelDebug::ISystemConfigure,
              LevelDebug::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LevelDebug, "gz::sim::systems::LevelDebug")
