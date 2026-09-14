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
#ifndef GZ_SIM_UNIT_TEST_UTIL_HH_
#define GZ_SIM_UNIT_TEST_UTIL_HH_

#include <string>

#include <gtest/gtest.h>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

namespace gz::sim::test
{
/////////////////////////////////////////////////
/// \brief Load a world from an SDF string and create entities
/// \param[in] _sdfString The SDF string to load
/// \param[in] _ecm Immutable reference to ECM.
/// \param[in] _eventMgr Event manager used while creating entities.
/// \param[out] _entity The entity corresponding to the world
/// \param[out] _pluginSdf Optional copy of the world's first plugin SDF.
/// If nullptr, no plugin will be stored
inline void LoadWorldContext(
  const std::string &_sdfString,
  EntityComponentManager &_ecm,
  EventManager &_eventMgr,
  Entity &_entity,
  sdf::Plugin *_pluginSdf = nullptr)
{
  sdf::Root root;
  root.LoadSdfString(_sdfString);

  ASSERT_EQ(1u, root.WorldCount());
  const sdf::World *worldSdf = root.WorldByIndex(0);
  ASSERT_NE(nullptr, worldSdf);

  if (_pluginSdf)
  {
    ASSERT_FALSE(worldSdf->Plugins().empty());
    *_pluginSdf = worldSdf->Plugins()[0];
  }

  SdfEntityCreator entityCreator(_ecm, _eventMgr);

  _entity = entityCreator.CreateEntities(worldSdf);
  ASSERT_NE(kNullEntity, _entity);
}

/////////////////////////////////////////////////
/// \brief Load a model from an SDF string and create entities
/// \param[in] _sdfString The SDF string to load
/// \param[in] _ecm Immutable reference to ECM.
/// \param[in] _eventMgr Event manager used while creating entities.
/// \param[out] _entity The entity corresponding to the model
/// \param[out] _pluginSdf Optional copy of the model's first plugin SDF.
/// If nullptr, no plugin will be stored
inline void LoadModelContext(
  const std::string &_sdfString,
  EntityComponentManager &_ecm,
  EventManager &_eventMgr,
  Entity &_entity,
  sdf::Plugin *_pluginSdf = nullptr)
{
  sdf::Root root;
  root.LoadSdfString(_sdfString);

  ASSERT_EQ(1u, root.WorldCount());
  const sdf::World *worldSdf = root.WorldByIndex(0);
  ASSERT_NE(nullptr, worldSdf);

  ASSERT_EQ(1u, worldSdf->ModelCount());
  const sdf::Model *modelSdf = worldSdf->ModelByIndex(0);
  ASSERT_NE(nullptr, modelSdf);

  if (_pluginSdf)
  {
    ASSERT_FALSE(modelSdf->Plugins().empty());
    *_pluginSdf = modelSdf->Plugins()[0];
  }

  SdfEntityCreator entityCreator(_ecm, _eventMgr);
  _entity = entityCreator.CreateEntitiesWithoutLoadingPlugins(modelSdf);
  ASSERT_NE(kNullEntity, _entity);
}
} // namespace gz::sim::test

#endif  // GZ_SIM_UNIT_TEST_UTIL_HH_
