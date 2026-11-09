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

#include <gtest/gtest.h>

#include <QQmlContext>
#include <QQmlEngine>
#include <QStandardItemModel>

#include <gz/common/Filesystem.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Loader.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/gui/GuiSystem.hh"
#include "test_config.hh"
#include "../../../../test/helpers/EnvTestFixture.hh"

int g_argc = 1;
char *g_argv[] =
{
  reinterpret_cast<char *>(const_cast<char *>("EntityTree_TEST")),
};

using namespace gz;

/// \brief Tests for the EntityTree GUI plugin
class EntityTreeTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Verify that pre-existing entities are added to the tree on the initial
// update regardless of the order in which they are processed from the ECM.
TEST_F(EntityTreeTest, InitialEntitiesPopulated)
{
  auto app = std::make_unique<gui::Application>(g_argc, g_argv);
  ASSERT_NE(nullptr, app);

  common::SystemPaths systemPaths;
  systemPaths.AddPluginPaths(common::joinPaths(PROJECT_BINARY_PATH, "lib"));
  auto pathToLib = systemPaths.FindSharedLibrary("EntityTree");
  ASSERT_FALSE(pathToLib.empty());

  plugin::Loader pluginLoader;
  ASSERT_FALSE(pluginLoader.LoadLib(pathToLib).empty());

  auto pluginPtr = pluginLoader.Instantiate("gz::sim::EntityTree");
  ASSERT_FALSE(pluginPtr.IsEmpty());
  auto *entityTree =
      static_cast<sim::GuiSystem *>(pluginPtr->QueryInterface<gui::Plugin>());
  ASSERT_NE(nullptr, entityTree);

  sim::EntityComponentManager ecm;
  auto worldEntity = ecm.CreateEntity();
  ecm.CreateComponent(worldEntity, sim::components::World());
  ecm.CreateComponent(worldEntity, sim::components::Name("default"));

  auto modelEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelEntity, sim::components::Model());
  ecm.CreateComponent(modelEntity, sim::components::Name("model_1"));
  ecm.CreateComponent(modelEntity,
      sim::components::ParentEntity(worldEntity));

  entityTree->Update(sim::UpdateInfo(), ecm);
  QCoreApplication::processEvents();

  auto *treeModel = qobject_cast<QStandardItemModel *>(
      app->Engine()->rootContext()->contextProperty("_EntityTreeModel")
          .value<QObject *>());
  ASSERT_NE(nullptr, treeModel);
  ASSERT_EQ(1, treeModel->rowCount());
  EXPECT_EQ(QString("model_1"),
      treeModel->data(treeModel->index(0, 0)).toString());

  pluginPtr.Clear();
}
