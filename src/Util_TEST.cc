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

#include <string>
#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>
#include <gz/common/Console.hh>
#include <sdf/Actor.hh>
#include <sdf/Light.hh>
#include <sdf/Types.hh>

#include <gz/fuel_tools/ClientConfig.hh>

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParticleEmitter.hh"
#include "gz/sim/components/Projector.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

#include "helpers/EnvTestFixture.hh"
#include "test_config.hh"

using namespace gz;
using namespace sim;

/// \brief Tests for Util.hh
class UtilTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(UtilTest, ScopedName)
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
  EXPECT_EQ(kNullEntity, sim::worldEntity(ecm));
  EXPECT_EQ(kNullEntity, sim::worldEntity(worldEntity, ecm));
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
  EXPECT_EQ(worldEntity, sim::worldEntity(ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(worldEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(lightAEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(modelBEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(linkBEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(lightBEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(sensorBEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(modelCEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(linkCEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(collisionCEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(visualCEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(jointCEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(modelCCEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(linkCCEntity, ecm));
  EXPECT_EQ(worldEntity, sim::worldEntity(actorDEntity, ecm));
  EXPECT_EQ(kNullEntity, sim::worldEntity(kNullEntity, ecm));
}

/////////////////////////////////////////////////
TEST_F(UtilTest, EntitiesFromScopedName)
{
  EntityComponentManager ecm;

  // banana 1
  //  - orange 2
  //    - plum 3
  //      - grape 4
  //        - pear 5
  //          - plum 6
  //  - grape 7
  //    - pear 8
  //      - plum 9
  //        - pear 10
  //  - grape 11
  //    - pear 12
  //      - orange 13
  //        - orange 14
  //    - pear 15

  auto createEntity = [&ecm](const std::string &_name, Entity _parent) -> Entity
  {
    auto res = ecm.CreateEntity();
    ecm.CreateComponent(res, components::Name(_name));
    ecm.CreateComponent(res, components::ParentEntity(_parent));
    return res;
  };

  auto banana1 = ecm.CreateEntity();
  ecm.CreateComponent(banana1, components::Name("banana"));

  auto orange2 = createEntity("orange", banana1);
  auto plum3 = createEntity("plum", orange2);
  auto grape4 = createEntity("grape", plum3);
  auto pear5 = createEntity("pear", grape4);
  auto plum6 = createEntity("plum", pear5);
  auto grape7 = createEntity("grape", banana1);
  auto pear8 = createEntity("pear", grape7);
  auto plum9 = createEntity("plum", pear8);
  auto pear10 = createEntity("pear", plum9);
  auto grape11 = createEntity("grape", banana1);
  auto pear12 = createEntity("pear", grape11);
  auto orange13 = createEntity("orange", pear12);
  auto orange14 = createEntity("orange", orange13);
  auto pear15 = createEntity("pear", grape11);

  auto checkEntities = [&ecm](const std::string &_scopedName,
      Entity _relativeTo, const std::unordered_set<Entity> &_result,
      const std::string &_delim)
  {
    auto res = entitiesFromScopedName(_scopedName, ecm, _relativeTo, _delim);
    EXPECT_EQ(_result.size(), res.size()) << _scopedName;

    for (auto it : _result)
    {
      EXPECT_NE(res.find(it), res.end()) << it << "  " << _scopedName;
    }
  };

  checkEntities("watermelon", kNullEntity, {}, "::");
  checkEntities("banana", kNullEntity, {banana1}, "::");
  checkEntities("orange", kNullEntity, {orange2, orange13, orange14}, ":");
  checkEntities("banana::orange", kNullEntity, {orange2}, "::");
  checkEntities("banana::grape", kNullEntity, {grape7, grape11}, "::");
  checkEntities("grape/pear", kNullEntity, {pear5, pear8, pear12, pear15}, "/");
  checkEntities("grape...pear...plum", kNullEntity, {plum6, plum9}, "...");
  checkEntities(
      "banana::orange::plum::grape::pear::plum", kNullEntity, {plum6}, "::");
  checkEntities(
      "banana::orange::kiwi::grape::pear::plum", kNullEntity, {}, "::");
  checkEntities("orange+orange", kNullEntity, {orange14}, "+");
  checkEntities("orange", banana1, {orange2}, "::");
  checkEntities("grape", banana1, {grape7, grape11}, "::");
  checkEntities("orange", orange2, {}, "::");
  checkEntities("orange", orange13, {orange14}, "::");
  checkEntities("grape::pear::plum", plum3, {plum6}, "::");
  checkEntities("pear", grape11, {pear12, pear15}, "==");
  checkEntities("plum=pear", pear8, {pear10}, "=");
}

/////////////////////////////////////////////////
TEST_F(UtilTest, EntityTypeId)
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

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::ParticleEmitter());
  EXPECT_EQ(components::ParticleEmitter::typeId, entityTypeId(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Projector());
  EXPECT_EQ(components::Projector::typeId, entityTypeId(entity, ecm));
}

/////////////////////////////////////////////////
TEST_F(UtilTest, EntityTypeStr)
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

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::ParticleEmitter());
  EXPECT_EQ("particle_emitter", entityTypeStr(entity, ecm));

  entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, components::Projector());
  EXPECT_EQ("projector", entityTypeStr(entity, ecm));
}

/////////////////////////////////////////////////
TEST_F(UtilTest, RemoveParentScopedName)
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
TEST_F(UtilTest, AsFullPath)
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
    const std::string path{sdf::kSdfStringSource};

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

/////////////////////////////////////////////////
TEST_F(UtilTest, TopLevelModel)
{
  EntityComponentManager ecm;

  // world
  //  - modelA
  //    - linkA
  //    - modelB
  //      - linkB
  //        - visualB
  //  - modelC

  // World
  auto worldEntity = ecm.CreateEntity();
  ecm.CreateComponent(worldEntity, components::World());
  ecm.CreateComponent(worldEntity, components::Name("world_name"));

  // Model A
  auto modelAEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelAEntity, components::Model());
  ecm.CreateComponent(modelAEntity, components::Name("modelA_name"));
  ecm.CreateComponent(modelAEntity, components::ParentEntity(worldEntity));

  // Link A - Child of Model A
  auto linkAEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkAEntity, components::Link());
  ecm.CreateComponent(linkAEntity, components::Name("linkA_name"));
  ecm.CreateComponent(linkAEntity, components::ParentEntity(modelAEntity));

  // Model B - nested inside Model A
  auto modelBEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelBEntity, components::Model());
  ecm.CreateComponent(modelBEntity, components::Name("modelB_name"));
  ecm.CreateComponent(modelBEntity, components::ParentEntity(modelAEntity));

  // Link B - child of Model B
  auto linkBEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkBEntity, components::Link());
  ecm.CreateComponent(linkBEntity, components::Name("linkB_name"));
  ecm.CreateComponent(linkBEntity, components::ParentEntity(modelBEntity));

  // Visual B - child of Link B
  auto visualBEntity = ecm.CreateEntity();
  ecm.CreateComponent(visualBEntity, components::Visual());
  ecm.CreateComponent(visualBEntity, components::Name("visualB_name"));
  ecm.CreateComponent(visualBEntity, components::ParentEntity(linkBEntity));

  // Model C
  auto modelCEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelCEntity, components::Model());
  ecm.CreateComponent(modelCEntity, components::Name("modelC_name"));
  ecm.CreateComponent(modelCEntity, components::ParentEntity(worldEntity));

  // model A, link A, model B, link B and visual B should have
  // model A as the top level model
  EXPECT_EQ(modelAEntity, topLevelModel(modelAEntity, ecm));
  EXPECT_EQ(modelAEntity, topLevelModel(linkAEntity, ecm));
  EXPECT_EQ(modelAEntity, topLevelModel(modelBEntity, ecm));
  EXPECT_EQ(modelAEntity, topLevelModel(linkBEntity, ecm));
  EXPECT_EQ(modelAEntity, topLevelModel(visualBEntity, ecm));

  // model C should have itself as the top level model
  EXPECT_EQ(modelCEntity, topLevelModel(modelCEntity, ecm));

  // the world should have no top level model
  EXPECT_EQ(kNullEntity, topLevelModel(worldEntity, ecm));
}

/////////////////////////////////////////////////
TEST_F(UtilTest, ValidTopic)
{
  std::string good{"good"};
  std::string fixable{"not bad~"};
  std::string invalid{"@~@~@~"};

  EXPECT_EQ("good", validTopic({good}));
  EXPECT_EQ("not_bad", validTopic({fixable}));
  EXPECT_EQ("", validTopic({invalid}));

  EXPECT_EQ("good", validTopic({good, fixable}));
  EXPECT_EQ("not_bad", validTopic({fixable, good}));

  EXPECT_EQ("good", validTopic({good, invalid}));
  EXPECT_EQ("good", validTopic({invalid, good}));

  EXPECT_EQ("not_bad", validTopic({fixable, invalid}));
  EXPECT_EQ("not_bad", validTopic({invalid, fixable}));

  EXPECT_EQ("not_bad", validTopic({fixable, invalid, good}));
  EXPECT_EQ("good", validTopic({invalid, good, fixable}));
}

/////////////////////////////////////////////////
TEST_F(UtilTest, TopicFromScopedName)
{
  EntityComponentManager ecm;

  // world
  //  - modelA
  //    - linkA
  //    - modelB
  //      - linkB
  //        - emitterB
  //  - modelC

  // World
  auto worldEntity = ecm.CreateEntity();
  ecm.CreateComponent(worldEntity, components::World());
  ecm.CreateComponent(worldEntity, components::Name("world_name"));

  // Model A
  auto modelAEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelAEntity, components::Model());
  ecm.CreateComponent(modelAEntity, components::Name("modelA_name"));
  ecm.CreateComponent(modelAEntity, components::ParentEntity(worldEntity));

  // Link A - Child of Model A
  auto linkAEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkAEntity, components::Link());
  ecm.CreateComponent(linkAEntity, components::Name("linkA_name"));
  ecm.CreateComponent(linkAEntity, components::ParentEntity(modelAEntity));

  // Model B - nested inside Model A
  auto modelBEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelBEntity, components::Model());
  ecm.CreateComponent(modelBEntity, components::Name("modelB_name"));
  ecm.CreateComponent(modelBEntity, components::ParentEntity(modelAEntity));

  // Link B - child of Model B
  auto linkBEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkBEntity, components::Link());
  ecm.CreateComponent(linkBEntity, components::Name("linkB_name"));
  ecm.CreateComponent(linkBEntity, components::ParentEntity(modelBEntity));

  // Emitter B - child of Link B
  auto emitterBEntity = ecm.CreateEntity();
  ecm.CreateComponent(emitterBEntity, components::ParticleEmitter());
  ecm.CreateComponent(emitterBEntity, components::Name("emitterB_name"));
  ecm.CreateComponent(emitterBEntity, components::ParentEntity(linkBEntity));

  // Model C
  auto modelCEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelCEntity, components::Model());
  ecm.CreateComponent(modelCEntity, components::Name("modelC_name"));
  ecm.CreateComponent(modelCEntity, components::ParentEntity(worldEntity));

  std::string testName = "/model/modelA_name";
  std::string worldName = "/world/world_name";
  // model A, link A, model B, link B and visual B should have
  // model A as the top level model
  EXPECT_EQ(testName, topicFromScopedName(modelAEntity, ecm));
  EXPECT_EQ(worldName + testName,
      topicFromScopedName(modelAEntity, ecm, false));

  testName += "/link/linkA_name";
  EXPECT_EQ(testName, topicFromScopedName(linkAEntity, ecm));
  EXPECT_EQ(worldName + testName, topicFromScopedName(linkAEntity, ecm, false));

  testName = "/model/modelA_name/model/modelB_name";
  EXPECT_EQ(testName, topicFromScopedName(modelBEntity, ecm));
  EXPECT_EQ(worldName + testName,
      topicFromScopedName(modelBEntity, ecm, false));

  testName +="/link/linkB_name";
  EXPECT_EQ(testName, topicFromScopedName(linkBEntity, ecm));
  EXPECT_EQ(worldName + testName, topicFromScopedName(linkBEntity, ecm, false));

  testName += "/particle_emitter/emitterB_name";
  EXPECT_EQ(testName,
      topicFromScopedName(emitterBEntity, ecm));
  EXPECT_EQ(worldName + testName,
      topicFromScopedName(emitterBEntity, ecm, false));

  testName = "/model/modelC_name";
  EXPECT_EQ(testName, topicFromScopedName(modelCEntity, ecm));
  EXPECT_EQ(worldName + testName,
      topicFromScopedName(modelCEntity, ecm, false));

  EXPECT_TRUE(topicFromScopedName(worldEntity, ecm).empty());
  EXPECT_EQ(worldName, topicFromScopedName(worldEntity, ecm, false));
}

/////////////////////////////////////////////////
TEST_F(UtilTest, EnableComponent)
{
  EntityComponentManager ecm;

  auto entity1 = ecm.CreateEntity();
  EXPECT_EQ(nullptr, ecm.Component<components::Name>(entity1));

  // Enable
  EXPECT_TRUE(enableComponent<components::Name>(ecm, entity1));
  EXPECT_NE(nullptr, ecm.Component<components::Name>(entity1));

  // Enabling again makes no changes
  EXPECT_FALSE(enableComponent<components::Name>(ecm, entity1, true));
  EXPECT_NE(nullptr, ecm.Component<components::Name>(entity1));

  // Disable
  EXPECT_TRUE(enableComponent<components::Name>(ecm, entity1, false));
  EXPECT_EQ(nullptr, ecm.Component<components::Name>(entity1));

  // Disabling again makes no changes
  EXPECT_FALSE(enableComponent<components::Name>(ecm, entity1, false));
  EXPECT_EQ(nullptr, ecm.Component<components::Name>(entity1));

  // Creates a component with a specified value
  EXPECT_TRUE(enableComponent<components::Name>(ecm, entity1, true,
      components::Name("name")));
  EXPECT_EQ("name", ecm.Component<components::Name>(entity1)->Data());

  // Disabling with a dedicated value should have the same effect
  EXPECT_TRUE(enableComponent<components::Name>(ecm, entity1, false,
      components::Name("name")));
  EXPECT_EQ(nullptr, ecm.Component<components::Name>(entity1));
}

/////////////////////////////////////////////////
TEST_F(UtilTest, EntityFromMsg)
{
  EntityComponentManager ecm;

  // world
  //  - lightA (light)
  //  - modelB
  //    - linkB (link)
  //      - lightB (light)
  //      - sensorB (banana)
  //  - modelC
  //    - linkC (link)
  //      - collisionC
  //      - visualC (banana)
  //    - jointC
  //    - modelCC
  //      - linkCC (link)
  //  - actorD

  // World
  auto worldEntity = ecm.CreateEntity();
  EXPECT_EQ(kNullEntity, sim::worldEntity(ecm));
  EXPECT_EQ(kNullEntity, sim::worldEntity(worldEntity, ecm));
  ecm.CreateComponent(worldEntity, components::World());
  ecm.CreateComponent(worldEntity, components::Name("world"));

  // Light A
  auto lightAEntity = ecm.CreateEntity();
  ecm.CreateComponent(lightAEntity, components::Light(sdf::Light()));
  ecm.CreateComponent(lightAEntity, components::Name("light"));
  ecm.CreateComponent(lightAEntity, components::ParentEntity(worldEntity));

  // Model B
  auto modelBEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelBEntity, components::Model());
  ecm.CreateComponent(modelBEntity, components::Name("modelB"));
  ecm.CreateComponent(modelBEntity, components::ParentEntity(worldEntity));

  // Link B
  auto linkBEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkBEntity, components::Link());
  ecm.CreateComponent(linkBEntity, components::Name("link"));
  ecm.CreateComponent(linkBEntity, components::ParentEntity(modelBEntity));

  // Light B
  auto lightBEntity = ecm.CreateEntity();
  ecm.CreateComponent(lightBEntity, components::Light(sdf::Light()));
  ecm.CreateComponent(lightBEntity, components::Name("light"));
  ecm.CreateComponent(lightBEntity, components::ParentEntity(linkBEntity));

  // Sensor B
  auto sensorBEntity = ecm.CreateEntity();
  ecm.CreateComponent(sensorBEntity, components::Sensor());
  ecm.CreateComponent(sensorBEntity, components::Name("banana"));
  ecm.CreateComponent(sensorBEntity, components::ParentEntity(linkBEntity));

  // Model C
  auto modelCEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelCEntity, components::Model());
  ecm.CreateComponent(modelCEntity, components::Name("modelC"));
  ecm.CreateComponent(modelCEntity, components::ParentEntity(worldEntity));

  // Link C
  auto linkCEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkCEntity, components::Link());
  ecm.CreateComponent(linkCEntity, components::Name("link"));
  ecm.CreateComponent(linkCEntity, components::ParentEntity(modelCEntity));

  // Collision C
  auto collisionCEntity = ecm.CreateEntity();
  ecm.CreateComponent(collisionCEntity, components::Collision());
  ecm.CreateComponent(collisionCEntity, components::Name("collisionC"));
  ecm.CreateComponent(collisionCEntity, components::ParentEntity(linkCEntity));

  // Visual C
  auto visualCEntity = ecm.CreateEntity();
  ecm.CreateComponent(visualCEntity, components::Visual());
  ecm.CreateComponent(visualCEntity, components::Name("banana"));
  ecm.CreateComponent(visualCEntity, components::ParentEntity(linkCEntity));

  // Link C
  auto jointCEntity = ecm.CreateEntity();
  ecm.CreateComponent(jointCEntity, components::Joint());
  ecm.CreateComponent(jointCEntity, components::Name("jointC"));
  ecm.CreateComponent(jointCEntity, components::ParentEntity(modelCEntity));

  // Model CC
  auto modelCCEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelCCEntity, components::Model());
  ecm.CreateComponent(modelCCEntity, components::Name("modelCC"));
  ecm.CreateComponent(modelCCEntity, components::ParentEntity(modelCEntity));

  // Link CC
  auto linkCCEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkCCEntity, components::Link());
  ecm.CreateComponent(linkCCEntity, components::Name("link"));
  ecm.CreateComponent(linkCCEntity, components::ParentEntity(modelCCEntity));

  // Actor D
  auto actorDEntity = ecm.CreateEntity();
  ecm.CreateComponent(actorDEntity, components::Actor(sdf::Actor()));
  ecm.CreateComponent(actorDEntity, components::Name("actorD"));
  ecm.CreateComponent(actorDEntity, components::ParentEntity(worldEntity));

  // Check entities
  auto createMsg = [](Entity _id, const std::string &_name = "",
      msgs::Entity::Type _type = msgs::Entity::NONE) -> msgs::Entity
  {
    msgs::Entity msg;

    if (_id != kNullEntity)
      msg.set_id(_id);

    if (!_name.empty())
      msg.set_name(_name);

    if (_type != msgs::Entity::NONE)
      msg.set_type(_type);

    return msg;
  };

  // Only ID
  EXPECT_EQ(worldEntity, entityFromMsg(ecm, createMsg(worldEntity)));
  EXPECT_EQ(lightAEntity, entityFromMsg(ecm, createMsg(lightAEntity)));
  EXPECT_EQ(modelBEntity, entityFromMsg(ecm, createMsg(modelBEntity)));
  EXPECT_EQ(linkBEntity, entityFromMsg(ecm, createMsg(linkBEntity)));
  EXPECT_EQ(lightBEntity, entityFromMsg(ecm, createMsg(lightBEntity)));
  EXPECT_EQ(sensorBEntity, entityFromMsg(ecm, createMsg(sensorBEntity)));
  EXPECT_EQ(modelCEntity, entityFromMsg(ecm, createMsg(modelCEntity)));
  EXPECT_EQ(linkCEntity, entityFromMsg(ecm, createMsg(linkCEntity)));
  EXPECT_EQ(collisionCEntity, entityFromMsg(ecm, createMsg(collisionCEntity)));
  EXPECT_EQ(visualCEntity, entityFromMsg(ecm, createMsg(visualCEntity)));
  EXPECT_EQ(jointCEntity, entityFromMsg(ecm, createMsg(jointCEntity)));
  EXPECT_EQ(modelCCEntity, entityFromMsg(ecm, createMsg(modelCCEntity)));
  EXPECT_EQ(linkCCEntity, entityFromMsg(ecm, createMsg(linkCCEntity)));
  EXPECT_EQ(actorDEntity, entityFromMsg(ecm, createMsg(actorDEntity)));

  // Name and type, with different scopes
  EXPECT_EQ(worldEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world", msgs::Entity::WORLD)));

  // There's more than one "light", so we need to scope it
  EXPECT_EQ(lightAEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::light", msgs::Entity::LIGHT)));

  EXPECT_EQ(modelBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelB", msgs::Entity::MODEL)));
  EXPECT_EQ(modelBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelB", msgs::Entity::MODEL)));

  // There's more than one "link", so we need to scope it
  EXPECT_EQ(linkBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelB::link", msgs::Entity::LINK)));
  EXPECT_EQ(linkBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelB::link", msgs::Entity::LINK)));

  // There's more than one "light", so we need to scope it
  EXPECT_EQ(lightBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "link::light", msgs::Entity::LIGHT)));
  EXPECT_EQ(lightBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelB::link::light", msgs::Entity::LIGHT)));
  EXPECT_EQ(lightBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelB::link::light", msgs::Entity::LIGHT)));

  // There's more than one "link::banana", so we need to scope it
  EXPECT_EQ(sensorBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelB::link::banana", msgs::Entity::SENSOR)));
  EXPECT_EQ(sensorBEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelB::link::banana", msgs::Entity::SENSOR)));

  EXPECT_EQ(modelCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelC", msgs::Entity::MODEL)));
  EXPECT_EQ(modelCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelC", msgs::Entity::MODEL)));

  // There's more than one "link", so we need to scope it
  EXPECT_EQ(linkCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelC::link", msgs::Entity::LINK)));
  EXPECT_EQ(linkCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelC::link", msgs::Entity::LINK)));

  EXPECT_EQ(collisionCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "collisionC", msgs::Entity::COLLISION)));
  EXPECT_EQ(collisionCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "link::collisionC", msgs::Entity::COLLISION)));
  EXPECT_EQ(collisionCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelC::link::collisionC", msgs::Entity::COLLISION)));
  EXPECT_EQ(collisionCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelC::link::collisionC", msgs::Entity::COLLISION)));

  // There's more than one "banana", so we need to scope it
  EXPECT_EQ(visualCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "link::banana", msgs::Entity::VISUAL)));
  EXPECT_EQ(visualCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelC::link::banana", msgs::Entity::VISUAL)));
  EXPECT_EQ(visualCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelC::link::banana", msgs::Entity::VISUAL)));

  EXPECT_EQ(jointCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "jointC", msgs::Entity::JOINT)));
  EXPECT_EQ(jointCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelC::jointC", msgs::Entity::JOINT)));
  EXPECT_EQ(jointCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelC::jointC", msgs::Entity::JOINT)));

  EXPECT_EQ(modelCCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelCC", msgs::Entity::MODEL)));
  EXPECT_EQ(modelCCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelC::modelCC", msgs::Entity::MODEL)));
  EXPECT_EQ(modelCCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelC::modelCC", msgs::Entity::MODEL)));

  // There's more than one "link", so we need to scope it
  EXPECT_EQ(linkCCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "modelC::modelCC::link", msgs::Entity::LINK)));
  EXPECT_EQ(linkCCEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::modelC::modelCC::link", msgs::Entity::LINK)));

  EXPECT_EQ(actorDEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "actorD", msgs::Entity::ACTOR)));
  EXPECT_EQ(actorDEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "world::actorD", msgs::Entity::ACTOR)));

  // Inexistent entity
  EXPECT_EQ(1000u, entityFromMsg(ecm, createMsg(1000)));

  // Not found
  EXPECT_EQ(kNullEntity, entityFromMsg(ecm, createMsg(kNullEntity)));
  EXPECT_EQ(kNullEntity, entityFromMsg(ecm, createMsg(kNullEntity,
      "blueberry")));
  EXPECT_EQ(kNullEntity, entityFromMsg(ecm, createMsg(kNullEntity, "banana",
      msgs::Entity::COLLISION)));
  EXPECT_EQ(kNullEntity, entityFromMsg(ecm, createMsg(kNullEntity, "peach",
      msgs::Entity::WORLD)));
}

/////////////////////////////////////////////////
TEST_F(UtilTest, ResolveSdfWorldFile)
{
  // Test resolving a Fuel URI
  fuel_tools::ClientConfig config;

  // URI to a Fuel world.
  std::string fuelUri =
    "https://fuel.gazebosim.org/1.0/openrobotics/worlds/test world";

  // The expect path for the local Fuel world.
  std::string expectedPath = common::joinPaths(
      config.CacheLocation(), "fuel.gazebosim.org",
      "openrobotics", "worlds", "test world");

  // Get the Fuel world.
  std::string resolvedPath = resolveSdfWorldFile(fuelUri,
      config.CacheLocation());

  // The Fuel model has not been downloaded, so it should not exist.
  EXPECT_FALSE(resolvedPath.empty());

  // The expected path should be the first part of the resolved path. The
  // resolved path will have extra world version information at the end.
  EXPECT_EQ(0u, resolvedPath.find(expectedPath));

  // Now try to resolve the downloaded world file using an aboslute path
  EXPECT_EQ(resolvedPath, resolveSdfWorldFile(resolvedPath));

  // The "shapes.sdf" world file should resolve.
  EXPECT_FALSE(resolveSdfWorldFile("shapes.sdf").empty());

  // A bad absolute path should return an empty string
  EXPECT_TRUE(resolveSdfWorldFile("/invalid/does_not_exist.sdf").empty());

  // A bad relative path should return an empty string
  EXPECT_TRUE(resolveSdfWorldFile("../invalid/does_not_exist.sdf").empty());
}

/////////////////////////////////////////////////
TEST_F(UtilTest, LoadMesh)
{
  sdf::Mesh meshSdf;
  EXPECT_EQ(nullptr, loadMesh(meshSdf));

  meshSdf.SetUri("invalid_uri");
  meshSdf.SetFilePath("invalid_filepath");
  EXPECT_EQ(nullptr, loadMesh(meshSdf));

  meshSdf.SetUri("name://unit_box");
  EXPECT_NE(nullptr, loadMesh(meshSdf));

  meshSdf.SetUri("duck.dae");
  std::string filePath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "media", "duck.dae");
  meshSdf.SetFilePath(filePath);
  EXPECT_NE(nullptr, loadMesh(meshSdf));

  EXPECT_TRUE(meshSdf.SetOptimization("convex_decomposition"));
  sdf::ConvexDecomposition convexDecomp;
  convexDecomp.SetMaxConvexHulls(16u);
  meshSdf.SetConvexDecomposition(convexDecomp);
  auto *optimizedMesh = loadMesh(meshSdf);
  EXPECT_NE(nullptr, optimizedMesh);
  EXPECT_EQ(16u, optimizedMesh->SubMeshCount());
}

/////////////////////////////////////////////////
TEST_F(UtilTest, MeshAxisAlignedBoundingBox)
{
  sdf::Mesh meshSdf;
  math::AxisAlignedBox emptyBox, aab;
  EXPECT_FALSE(meshAxisAlignedBox(meshSdf).has_value());

  meshSdf.SetUri("invalid_uri");
  meshSdf.SetFilePath("invalid_filepath");
  EXPECT_FALSE(meshAxisAlignedBox(meshSdf).has_value());

  meshSdf.SetUri("name://unit_box");
  aab = meshAxisAlignedBox(meshSdf).value();
  EXPECT_NE(emptyBox, aab);
  EXPECT_EQ(aab.Size(), math::Vector3d::One);
  EXPECT_EQ(aab.Min(), math::Vector3d(-0.5, -0.5, -0.5));
  EXPECT_EQ(aab.Max(), math::Vector3d(0.5, 0.5, 0.5));
  EXPECT_EQ(aab.Center(), math::Vector3d::Zero);

  // Validate scaling using the unit box mesh
  meshSdf.SetScale(math::Vector3d(2, 3, 4));
  aab = meshAxisAlignedBox(meshSdf).value();
  EXPECT_NE(emptyBox, aab);
  EXPECT_EQ(aab.Size(), math::Vector3d(2, 3, 4));
  EXPECT_EQ(aab.Min(), math::Vector3d(-1, -1.5, -2));
  EXPECT_EQ(aab.Max(), math::Vector3d(1, 1.5, 2));
  EXPECT_EQ(aab.Center(), math::Vector3d::Zero);
  meshSdf.SetScale(math::Vector3d(1, 1, 1));

  meshSdf.SetUri("duck.dae");
  std::string filePath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "media", "duck.dae");
  meshSdf.SetFilePath(filePath);
  aab = meshAxisAlignedBox(meshSdf).value();
  EXPECT_NE(emptyBox, aab);

  // Expected values obtained from the mesh file using Blender:
  auto minExp = math::Vector3d(-69.2985, 9.92937, -61.3282) * 1e-2;
  auto maxExp = math::Vector3d(96.1799, 163.9700, 53.9252) * 1e-2;

  EXPECT_EQ(minExp, aab.Min());
  EXPECT_EQ(maxExp, aab.Max());
  EXPECT_EQ((minExp + maxExp) / 2, aab.Center());
  EXPECT_EQ(maxExp - minExp, aab.Size());
}

/////////////////////////////////////////////////
TEST_F(UtilTest, TransformAxisAlignedBoxFrame)
{
  // Creates a pseudo-random vector with values between -max and max
  std::srand(std::time(nullptr));
  auto randomVector3d = [](int max = 1) -> math::Vector3d
  {
    return (
      math::Vector3d(
        static_cast<double>(std::rand()),
        static_cast<double>(std::rand()),
        static_cast<double>(std::rand())
      ) / RAND_MAX - 0.5 * math::Vector3d::One
    ) * 2 * max;
  };

  math::AxisAlignedBox aab(randomVector3d(3), randomVector3d(2));

  // Trivial case: identity transform should not change the box
  math::Pose3d transform = math::Pose3d::Zero;
  math::AxisAlignedBox aabExp = aab;
  EXPECT_EQ(aabExp, transformAxisAlignedBox(aab, transform));

  // Pure translation offset
  transform.Pos() = randomVector3d(10);
  aabExp = aab + transform.Pos();
  EXPECT_EQ(aabExp, transformAxisAlignedBox(aab, transform));

  // Pure rotation of 90 degrees around the z-axis
  // x -> y, y -> -x, z -> z
  transform.Reset();
  transform.Rot() = math::Quaterniond(0, 0, GZ_PI_2);
  aabExp = math::AxisAlignedBox(
    math::Vector3d(-aab.Max().Y(), aab.Min().X(), aab.Min().Z()),
    math::Vector3d(-aab.Min().Y(), aab.Max().X(), aab.Max().Z())
  );
  EXPECT_EQ(aabExp, transformAxisAlignedBox(aab, transform));

  // Full transform: translation and rotation
  transform.Pos() = randomVector3d(150);
  aabExp = aabExp + transform.Pos();
  EXPECT_EQ(aabExp, transformAxisAlignedBox(aab, transform));
}
