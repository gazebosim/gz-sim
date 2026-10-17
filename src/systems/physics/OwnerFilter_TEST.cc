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

#include "OwnerFilter.hh"

#include <gtest/gtest.h>

#include <unordered_set>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/components/World.hh"

using namespace gz;
using namespace sim;
using namespace systems::physics_system;

namespace
{
const char *kOther = "gz::sim::systems::Other";

/// \brief A world with a model and a joint in the model.
struct Scene
{
  EntityComponentManager ecm;
  Entity world{kNullEntity};
  Entity model{kNullEntity};
  Entity joint{kNullEntity};

  Scene()
  {
    this->world = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->world, components::World());
    this->model = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->model, components::Model());
    this->ecm.CreateComponent(this->model,
        components::ParentEntity(this->world));
    this->joint = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->joint, components::Joint());
    this->ecm.CreateComponent(this->joint,
        components::ParentEntity(this->model));
  }
};
}

/////////////////////////////////////////////////
TEST(OwnerFilter, AcceptsEverythingByDefault)
{
  Scene scene;
  scene.ecm.CreateComponent(scene.model, components::PhysicsOwner(kOther));

  // A default filter does not look at the ECM, even if another provider
  // owns the entity.
  const OwnerFilter all;
  EXPECT_TRUE(all.Owns(scene.model));
  EXPECT_TRUE(all.Owns(scene.joint));
  EXPECT_TRUE(all.Owns(kNullEntity));
}

/////////////////////////////////////////////////
TEST(OwnerFilter, Predicate)
{
  Scene scene;
  std::unordered_set<Entity> owned{scene.joint};

  const OwnerFilter filter(
      [&owned](const Entity &_entity) { return owned.count(_entity) > 0; });
  EXPECT_TRUE(filter.Owns(scene.joint));
  EXPECT_FALSE(filter.Owns(scene.model));

  // The predicate is evaluated on every call.
  owned.clear();
  EXPECT_FALSE(filter.Owns(scene.joint));
}

/////////////////////////////////////////////////
TEST(OwnerFilter, NoOwnerComponent)
{
  Scene scene;
  // The default provider owns everything, others own nothing.
  EXPECT_TRUE(OwnerFilter::ForProvider(scene.ecm, "Default", true)
      .Owns(scene.joint));
  EXPECT_FALSE(OwnerFilter::ForProvider(scene.ecm, kOther, false)
      .Owns(scene.joint));
}

/////////////////////////////////////////////////
TEST(OwnerFilter, ModelOwner)
{
  Scene scene;
  scene.ecm.CreateComponent(scene.model, components::PhysicsOwner(kOther));

  const auto other = OwnerFilter::ForProvider(scene.ecm, kOther, false);
  const auto defaultProvider =
      OwnerFilter::ForProvider(scene.ecm, "Default", true);
  EXPECT_TRUE(other.Owns(scene.model));
  EXPECT_TRUE(other.Owns(scene.joint));
  EXPECT_FALSE(defaultProvider.Owns(scene.model));
  EXPECT_FALSE(defaultProvider.Owns(scene.joint));

  // A model in the same world without the component is not affected.
  const auto model2 = scene.ecm.CreateEntity();
  scene.ecm.CreateComponent(model2, components::Model());
  scene.ecm.CreateComponent(model2, components::ParentEntity(scene.world));
  EXPECT_TRUE(defaultProvider.Owns(model2));
  EXPECT_FALSE(other.Owns(model2));
}

/////////////////////////////////////////////////
TEST(OwnerFilter, WorldOwnerAndModelOverride)
{
  Scene scene;
  scene.ecm.CreateComponent(scene.world, components::PhysicsOwner(kOther));

  {
    const auto other = OwnerFilter::ForProvider(scene.ecm, kOther, false);
    const auto defaultProvider =
        OwnerFilter::ForProvider(scene.ecm, "Default", true);
    EXPECT_TRUE(other.Owns(scene.joint));
    EXPECT_FALSE(defaultProvider.Owns(scene.joint));
  }

  // An empty component on the model hands it back to the default provider.
  scene.ecm.CreateComponent(scene.model, components::PhysicsOwner(""));
  const auto other = OwnerFilter::ForProvider(scene.ecm, kOther, false);
  const auto defaultProvider =
      OwnerFilter::ForProvider(scene.ecm, "Default", true);
  EXPECT_FALSE(other.Owns(scene.joint));
  EXPECT_TRUE(defaultProvider.Owns(scene.joint));
}
