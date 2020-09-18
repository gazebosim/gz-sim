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

#include <gtest/gtest.h>
#include <sdf/Actor.hh>
#include <sdf/Light.hh>

#include "ignition/gazebo/components/Actor.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TEST(UtilTest, ScopedName)
{
  EntityComponentManager ecm;

  // world
  //  - lightA
  //  - modelB
  //    - linkB
  //      - lightB
  //      - sensorB
  //  - modelC
  //    - linkC
  //      - collisionC
  //      - visualC
  //    - jointC
  //    - modelCC
  //      - linkCC
  //  - actorD

  // World
  auto worldEntity = ecm.CreateEntity();
  EXPECT_EQ(kNullEntity, gazebo::worldEntity(worldEntity, ecm));
  ecm.CreateComponent(worldEntity, components::World());
  ecm.CreateComponent(worldEntity, components::Name("world_name"));

  // Light A
  auto lightAEntity = ecm.CreateEntity();
  ecm.CreateComponent(lightAEntity, components::Light(sdf::Light()));
  ecm.CreateComponent(lightAEntity, components::Name("lightA_name"));
  ecm.CreateComponent(lightAEntity, components::ParentEntity(worldEntity));

  // Model B
  auto modelBEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelBEntity, components::Model());
  ecm.CreateComponent(modelBEntity, components::Name("modelB_name"));
  ecm.CreateComponent(modelBEntity, components::ParentEntity(worldEntity));

  // Link B
  auto linkBEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkBEntity, components::Link());
  ecm.CreateComponent(linkBEntity, components::Name("linkB_name"));
  ecm.CreateComponent(linkBEntity, components::ParentEntity(modelBEntity));

  // Light B
  auto lightBEntity = ecm.CreateEntity();
  ecm.CreateComponent(lightBEntity, components::Light(sdf::Light()));
  ecm.CreateComponent(lightBEntity, components::Name("lightB_name"));
  ecm.CreateComponent(lightBEntity, components::ParentEntity(linkBEntity));

  // Sensor B
  auto sensorBEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorBEntity, components::Sensor());
  ecm.CreateComponent(sensorBEntity, components::Name("sensorB_name"));
  ecm.CreateComponent(sensorBEntity, components::ParentEntity(linkBEntity));

  // Model C
  auto modelCEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelCEntity, components::Model());
  ecm.CreateComponent(modelCEntity, components::Name("modelC_name"));
  ecm.CreateComponent(modelCEntity, components::ParentEntity(worldEntity));

  // Link C
  auto linkCEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkCEntity, components::Link());
  ecm.CreateComponent(linkCEntity, components::Name("linkC_name"));
  ecm.CreateComponent(linkCEntity, components::ParentEntity(modelCEntity));

  // Collision C
  auto collisionCEntity = ecm.CreateEntity();
  ecm.CreateComponent(collisionCEntity, components::Collision());
  ecm.CreateComponent(collisionCEntity, components::Name("collisionC_name"));
  ecm.CreateComponent(collisionCEntity, components::ParentEntity(linkCEntity));

  // Visual C
  auto visualCEntity = ecm.CreateEntity();
  ecm.CreateComponent(visualCEntity, components::Visual());
  ecm.CreateComponent(visualCEntity, components::Name("visualC_name"));
  ecm.CreateComponent(visualCEntity, components::ParentEntity(linkCEntity));

  // Link C
  auto jointCEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointCEntity, components::Joint());
  ecm.CreateComponent(jointCEntity, components::Name("jointC_name"));
  ecm.CreateComponent(jointCEntity, components::ParentEntity(modelCEntity));

  // Model CC
  auto modelCCEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelCCEntity, components::Model());
  ecm.CreateComponent(modelCCEntity, components::Name("modelCC_name"));
  ecm.CreateComponent(modelCCEntity, components::ParentEntity(modelCEntity));

  // Link CC
  auto linkCCEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkCCEntity, components::Link());
  ecm.CreateComponent(linkCCEntity, components::Name("linkCC_name"));
  ecm.CreateComponent(linkCCEntity, components::ParentEntity(modelCCEntity));

  // Actor D
  auto actorDEntity = ecm.CreateEntity();
  ecm.CreateComponent(actorDEntity, components::Actor(sdf::Actor()));
  ecm.CreateComponent(actorDEntity, components::Name("actorD_name"));
  ecm.CreateComponent(actorDEntity, components::ParentEntity(worldEntity));

  // Check names
  EXPECT_EQ(scopedName(worldEntity, ecm), "world/world_name");
  EXPECT_EQ(scopedName(lightAEntity, ecm, "::"),
    "world::world_name::light::lightA_name");
  EXPECT_EQ(scopedName(modelBEntity, ecm, "/"),
    "world/world_name/model/modelB_name");
  EXPECT_EQ(scopedName(linkBEntity, ecm, " "),
    "world world_name model modelB_name link linkB_name");
  EXPECT_EQ(scopedName(lightBEntity, ecm),
    "world/world_name/model/modelB_name/link/linkB_name/light/lightB_name");
  EXPECT_EQ(scopedName(sensorBEntity, ecm),
    "world/world_name/model/modelB_name/link/linkB_name/sensor/sensorB_name");
  EXPECT_EQ(scopedName(modelCEntity, ecm),
    "world/world_name/model/modelC_name");
  EXPECT_EQ(scopedName(linkCEntity, ecm),
    "world/world_name/model/modelC_name/link/linkC_name");
  EXPECT_EQ(scopedName(collisionCEntity, ecm),
    "world/world_name/model/modelC_name/link/linkC_name/collision/" +
    std::string("collisionC_name"));
  EXPECT_EQ(scopedName(visualCEntity, ecm),
    "world/world_name/model/modelC_name/link/linkC_name/visual/" +
    std::string("visualC_name"));
  EXPECT_EQ(scopedName(jointCEntity, ecm),
    "world/world_name/model/modelC_name/joint/jointC_name");
  EXPECT_EQ(scopedName(modelCCEntity, ecm),
    "world/world_name/model/modelC_name/model/modelCC_name");
  EXPECT_EQ(scopedName(linkCCEntity, ecm),
    "world/world_name/model/modelC_name/model/modelCC_name/link/linkCC_name");
  EXPECT_EQ(scopedName(actorDEntity, ecm, "::"),
    "world::world_name::actor::actorD_name");

  // check name without prefix
  EXPECT_EQ(scopedName(worldEntity, ecm, "/", false), "world_name");
  EXPECT_EQ(scopedName(lightAEntity, ecm, "::", false),
    "world_name::lightA_name");
  EXPECT_EQ(scopedName(modelBEntity, ecm, "/", false),
    "world_name/modelB_name");
  EXPECT_EQ(scopedName(linkBEntity, ecm, " ", false),
    "world_name modelB_name linkB_name");
  EXPECT_EQ(scopedName(lightBEntity, ecm, "/", false),
    "world_name/modelB_name/linkB_name/lightB_name");
  EXPECT_EQ(scopedName(sensorBEntity, ecm, "/", false),
    "world_name/modelB_name/linkB_name/sensorB_name");
  EXPECT_EQ(scopedName(modelCEntity, ecm, "/", false),
    "world_name/modelC_name");
  EXPECT_EQ(scopedName(linkCEntity, ecm, "/", false),
    "world_name/modelC_name/linkC_name");
  EXPECT_EQ(scopedName(collisionCEntity, ecm, "/", false),
    "world_name/modelC_name/linkC_name/" + std::string("collisionC_name"));
  EXPECT_EQ(scopedName(visualCEntity, ecm, "/", false),
    "world_name/modelC_name/linkC_name/" + std::string("visualC_name"));
  EXPECT_EQ(scopedName(jointCEntity, ecm, "/", false),
    "world_name/modelC_name/jointC_name");
  EXPECT_EQ(scopedName(modelCCEntity, ecm, "/", false),
    "world_name/modelC_name/modelCC_name");
  EXPECT_EQ(scopedName(linkCCEntity, ecm, "/", false),
    "world_name/modelC_name/modelCC_name/linkCC_name");
  EXPECT_EQ(scopedName(actorDEntity, ecm, "::", false),
    "world_name::actorD_name");

  // World entity
  EXPECT_EQ(worldEntity, gazebo::worldEntity(worldEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(lightAEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(modelBEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(linkBEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(lightBEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(sensorBEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(modelCEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(linkCEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(collisionCEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(visualCEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(jointCEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(modelCCEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(linkCCEntity, ecm));
  EXPECT_EQ(worldEntity, gazebo::worldEntity(actorDEntity, ecm));
  EXPECT_EQ(kNullEntity, gazebo::worldEntity(kNullEntity, ecm));
}

/////////////////////////////////////////////////
TEST(UtilTest, EntityTypeId)
{
  EntityComponentManager ecm;

  auto entity = ecm.CreateEntity();
  EXPECT_EQ(kComponentTypeIdInvalid, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::World());
  EXPECT_EQ(components::World::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Model());
  EXPECT_EQ(components::Model::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Light());
  EXPECT_EQ(components::Light::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Link());
  EXPECT_EQ(components::Link::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Visual());
  EXPECT_EQ(components::Visual::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Collision());
  EXPECT_EQ(components::Collision::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Joint());
  EXPECT_EQ(components::Joint::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Sensor());
  EXPECT_EQ(components::Sensor::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Actor());
  EXPECT_EQ(components::Actor::typeId, entityTypeId(entity, ecm));
}

/////////////////////////////////////////////////
TEST(UtilTest, EntityTypeStr)
{
  EntityComponentManager ecm;

  auto entity = ecm.CreateEntity();
  EXPECT_TRUE(entityTypeStr(entity, ecm).empty());

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::World());
  EXPECT_EQ("world", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Model());
  EXPECT_EQ("model", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Light());
  EXPECT_EQ("light", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Link());
  EXPECT_EQ("link", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Visual());
  EXPECT_EQ("visual", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Collision());
  EXPECT_EQ("collision", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Joint());
  EXPECT_EQ("joint", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Sensor());
  EXPECT_EQ("sensor", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Actor());
  EXPECT_EQ("actor", entityTypeStr(entity, ecm));
}

/////////////////////////////////////////////////
TEST(UtilTest, RemoveParentScopedName)
{
  EXPECT_EQ(removeParentScope("world/world_name", "/"), "world_name");
  EXPECT_EQ(removeParentScope("world::world_name::light::lightA_name", "::"),
            "world_name::light::lightA_name");
  EXPECT_EQ(removeParentScope("world_name/model/modelB_name", "/"),
            "model/modelB_name");
  EXPECT_EQ(removeParentScope(
                "world world_name model modelB_name link linkB_name", " "),
            "world_name model modelB_name link linkB_name");
  EXPECT_EQ(removeParentScope("world::world_name::light::lightA_name", ""),
            "world::world_name::light::lightA_name");
}

/////////////////////////////////////////////////
TEST(UtilTest, AsFullPath)
{
  const std::string relativeUriUnix{"meshes/collision.dae"};
  const std::string relativeUriWindows{"meshes\\collision.dae"};
  const std::string absoluteUriUnix{"/path/to/collision.dae"};
  const std::string absoluteUriWindows{R"(C:\path\to\collision.dae)"};
  const std::string schemeUri{"https://website.com/collision.dae"};

  // Empty path
  {
    const std::string path{""};

    EXPECT_EQ(relativeUriUnix, asFullPath(relativeUriUnix, path));
    EXPECT_EQ(relativeUriWindows, asFullPath(relativeUriWindows, path));
    EXPECT_EQ(absoluteUriUnix, asFullPath(absoluteUriUnix, path));
    EXPECT_EQ(absoluteUriWindows, asFullPath(absoluteUriWindows, path));
    EXPECT_EQ(schemeUri, asFullPath(schemeUri, path));
  }

  // Data string
  {
    const std::string path{"data-string"};

    EXPECT_EQ(relativeUriUnix, asFullPath(relativeUriUnix, path));
    EXPECT_EQ(relativeUriWindows, asFullPath(relativeUriWindows, path));
    EXPECT_EQ(absoluteUriUnix, asFullPath(absoluteUriUnix, path));
    EXPECT_EQ(absoluteUriWindows, asFullPath(absoluteUriWindows, path));
    EXPECT_EQ(schemeUri, asFullPath(schemeUri, path));
  }

#ifdef _WIN32
  {
    // Absolute Windows path
    const std::string path{R"(C:\abs\path\file)"};

    // Directory
    EXPECT_EQ("C:\\abs\\path\\meshes\\collision.dae",
        asFullPath(relativeUriUnix, path));
    EXPECT_EQ("C:\\abs\\path\\meshes\\collision.dae",
        asFullPath(relativeUriWindows, path));
    // TODO(anyone) Support absolute Unix-style URIs on Windows
    EXPECT_EQ(absoluteUriWindows, asFullPath(absoluteUriWindows, path));
    EXPECT_EQ(schemeUri, asFullPath(schemeUri, path));

    // File
    auto filePath = common::joinPaths(path, "file.sdf");

    EXPECT_EQ("C:\\abs\\path\\file\\meshes\\collision.dae",
        asFullPath(relativeUriUnix, filePath));
    EXPECT_EQ("C:\\abs\\path\\file\\meshes\\collision.dae",
        asFullPath(relativeUriWindows, filePath));
    // TODO(anyone) Support absolute Unix-style URIs on Windows
    EXPECT_EQ(absoluteUriWindows, asFullPath(absoluteUriWindows, filePath));
    EXPECT_EQ(schemeUri, asFullPath(schemeUri, filePath));
  }
#else
  {
    // Absolute Unix path
    const std::string path{"/abs/path/file"};

    // Directory
    EXPECT_EQ("/abs/path/meshes/collision.dae",
        asFullPath(relativeUriUnix, path));
    EXPECT_EQ("/abs/path/meshes/collision.dae",
        asFullPath(relativeUriWindows, path));
    EXPECT_EQ(absoluteUriUnix, asFullPath(absoluteUriUnix, path));
    // TODO(anyone) Support absolute Windows paths on Unix
    EXPECT_EQ(schemeUri, asFullPath(schemeUri, path));

    // File
    auto filePath = common::joinPaths(path, "file.sdf");

    EXPECT_EQ("/abs/path/file/meshes/collision.dae",
        asFullPath(relativeUriUnix, filePath));
    EXPECT_EQ("/abs/path/file/meshes/collision.dae",
        asFullPath(relativeUriWindows, filePath));
    EXPECT_EQ(absoluteUriUnix, asFullPath(absoluteUriUnix, filePath));
    // TODO(anyone) Support absolute Windows paths on Unix
    EXPECT_EQ(schemeUri, asFullPath(schemeUri, filePath));
  }
#endif
}
