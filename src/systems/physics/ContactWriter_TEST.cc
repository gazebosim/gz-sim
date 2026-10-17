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

#include "ContactWriter.hh"

#include <gtest/gtest.h>

#include <optional>
#include <string>

#include <gz/math/Vector3.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/components/World.hh"

using namespace gz;
using namespace sim;
using namespace systems::physics_system;

namespace
{
const char *kOther = "gz::sim::systems::Other";

/// \brief A contact point with a normal, a force and a depth.
ContactPointData PointWithExtra(double _x)
{
  ContactPointData data;
  data.point = math::Vector3d(_x, 2, 3);
  data.extra = ContactPointData::Extra{
      math::Vector3d(0, 0, 1), math::Vector3d(0, 0, 10), 0.01};
  return data;
}

/// \brief Three collisions: A and B belong to the same unowned model, C
/// belongs to a model owned by another provider. All have ContactSensorData.
struct Scene
{
  EntityComponentManager ecm;
  Entity world{kNullEntity};
  Entity collA{kNullEntity};
  Entity collB{kNullEntity};
  Entity collC{kNullEntity};

  Scene()
  {
    this->world = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->world, components::World());
    this->ecm.CreateComponent(this->world, components::Name("world"));
    const auto model = this->AddModel("model_a", "");
    const auto modelOther = this->AddModel("model_c", kOther);
    this->collA = this->AddCollision("coll_a", model);
    this->collB = this->AddCollision("coll_b", model);
    this->collC = this->AddCollision("coll_c", modelOther);
    this->ecm.SetAllComponentsUnchanged();
  }

  Entity AddModel(const std::string &_name, const std::string &_owner)
  {
    const auto model = this->ecm.CreateEntity();
    this->ecm.CreateComponent(model, components::Model());
    this->ecm.CreateComponent(model, components::Name(_name));
    this->ecm.CreateComponent(model, components::ParentEntity(this->world));
    if (!_owner.empty())
      this->ecm.CreateComponent(model, components::PhysicsOwner(_owner));
    return model;
  }

  Entity AddCollision(const std::string &_name, Entity _parent)
  {
    const auto coll = this->ecm.CreateEntity();
    this->ecm.CreateComponent(coll, components::Collision());
    this->ecm.CreateComponent(coll, components::Name(_name));
    this->ecm.CreateComponent(coll, components::ParentEntity(_parent));
    this->ecm.CreateComponent(coll, components::ContactSensorData());
    return coll;
  }

  const msgs::Contacts &Contacts(Entity _coll) const
  {
    return this->ecm.Component<components::ContactSensorData>(_coll)->Data();
  }

  ComponentState State(Entity _coll) const
  {
    return this->ecm.ComponentState(_coll,
        components::ContactSensorData::typeId);
  }
};
}

/////////////////////////////////////////////////
TEST(ContactWriter, ContactsEqual)
{
  EntityContactMap map;
  map[2].push_back(PointWithExtra(1));
  map[2].push_back(ContactPointData{math::Vector3d(4, 5, 6), std::nullopt});

  msgs::Contacts msg;
  EXPECT_FALSE(ContactsEqual(msg, map));
  EXPECT_TRUE(ContactsEqual(msg, EntityContactMap()));

  Scene scene;
  const auto physics = OwnerFilter::ForProvider(
      scene.ecm, "Default", true);
  ContactMap contacts;
  contacts[scene.collA] = map;
  WriteContacts(scene.ecm, physics, contacts, false);
  EXPECT_TRUE(ContactsEqual(scene.Contacts(scene.collA), map));

  // A moved point is a change, a point inside the tolerance is not.
  map[2][0].point = math::Vector3d(1 + 1e-8, 2, 3);
  EXPECT_TRUE(ContactsEqual(scene.Contacts(scene.collA), map));
  map[2][0].point = math::Vector3d(1.1, 2, 3);
  EXPECT_FALSE(ContactsEqual(scene.Contacts(scene.collA), map));
  map[2][0].point = math::Vector3d(1, 2, 3);
  map[2][0].extra->force = math::Vector3d(0, 0, 11);
  EXPECT_FALSE(ContactsEqual(scene.Contacts(scene.collA), map));
  map[2][0].extra->force = math::Vector3d(0, 0, 10);
  map[2][0].extra->normal = math::Vector3d(0, 1, 0);
  EXPECT_FALSE(ContactsEqual(scene.Contacts(scene.collA), map));
}

/////////////////////////////////////////////////
TEST(ContactWriter, WritesContactMessages)
{
  Scene scene;
  const auto physics = OwnerFilter::ForProvider(
      scene.ecm, "Default", true);

  ContactMap contacts;
  contacts[scene.collA][scene.collB].push_back(PointWithExtra(1));
  contacts[scene.collA][scene.collB].push_back(
      ContactPointData{math::Vector3d(4, 5, 6), std::nullopt});
  contacts[scene.collB][scene.collA].push_back(PointWithExtra(1));

  WriteContacts(scene.ecm, physics, contacts, false);

  const auto &msgA = scene.Contacts(scene.collA);
  ASSERT_EQ(1, msgA.contact_size());
  EXPECT_EQ(scene.collA, msgA.contact(0).collision1().id());
  EXPECT_EQ(scene.collB, msgA.contact(0).collision2().id());
  EXPECT_TRUE(msgA.contact(0).collision1().name().empty());
  ASSERT_EQ(2, msgA.contact(0).position_size());
  EXPECT_DOUBLE_EQ(1.0, msgA.contact(0).position(0).x());
  EXPECT_DOUBLE_EQ(6.0, msgA.contact(0).position(1).z());

  // Only the point with extra data has a normal, a wrench and a depth.
  ASSERT_EQ(1, msgA.contact(0).normal_size());
  EXPECT_DOUBLE_EQ(1.0, msgA.contact(0).normal(0).z());
  ASSERT_EQ(1, msgA.contact(0).wrench_size());
  EXPECT_DOUBLE_EQ(10.0,
      msgA.contact(0).wrench(0).body_1_wrench().force().z());
  EXPECT_DOUBLE_EQ(-10.0,
      msgA.contact(0).wrench(0).body_2_wrench().force().z());
  ASSERT_EQ(1, msgA.contact(0).depth_size());
  EXPECT_DOUBLE_EQ(0.01, msgA.contact(0).depth(0));

  EXPECT_EQ(ComponentState::PeriodicChange, scene.State(scene.collA));
  EXPECT_EQ(ComponentState::PeriodicChange, scene.State(scene.collB));

  // The same contacts a step later are not a change.
  scene.ecm.SetAllComponentsUnchanged();
  WriteContacts(scene.ecm, physics, contacts, false);
  EXPECT_EQ(ComponentState::NoChange, scene.State(scene.collA));
  EXPECT_EQ(ComponentState::NoChange, scene.State(scene.collB));
  EXPECT_EQ(1, scene.Contacts(scene.collA).contact_size());
}

/////////////////////////////////////////////////
TEST(ContactWriter, EntityNames)
{
  Scene scene;
  const auto physics = OwnerFilter::ForProvider(
      scene.ecm, "Default", true);

  ContactMap contacts;
  contacts[scene.collA][scene.collB].push_back(PointWithExtra(1));
  WriteContacts(scene.ecm, physics, contacts, true);

  const auto &msgA = scene.Contacts(scene.collA);
  ASSERT_EQ(1, msgA.contact_size());
  EXPECT_EQ("model_a::coll_a", msgA.contact(0).collision1().name());
  EXPECT_EQ("model_a::coll_b", msgA.contact(0).collision2().name());
}

/////////////////////////////////////////////////
TEST(ContactWriter, ClearsStaleContacts)
{
  Scene scene;
  const auto physics = OwnerFilter::ForProvider(
      scene.ecm, "Default", true);

  ContactMap contacts;
  contacts[scene.collA][scene.collB].push_back(PointWithExtra(1));
  contacts[scene.collB][scene.collA].push_back(PointWithExtra(1));
  WriteContacts(scene.ecm, physics, contacts, false);
  ASSERT_EQ(1, scene.Contacts(scene.collA).contact_size());

  // A collision that no longer touches anything is emptied and marked.
  scene.ecm.SetAllComponentsUnchanged();
  contacts.erase(scene.collA);
  WriteContacts(scene.ecm, physics, contacts, false);
  EXPECT_EQ(0, scene.Contacts(scene.collA).contact_size());
  EXPECT_EQ(ComponentState::PeriodicChange, scene.State(scene.collA));
  EXPECT_EQ(ComponentState::NoChange, scene.State(scene.collB));

  // A collision that was already empty is marked, but not as changed.
  scene.ecm.SetAllComponentsUnchanged();
  WriteContacts(scene.ecm, physics, contacts, false);
  EXPECT_EQ(ComponentState::NoChange, scene.State(scene.collA));
}

/////////////////////////////////////////////////
TEST(ContactWriter, OwnerFilter)
{
  Scene scene;

  // Another provider wrote contacts for collision C.
  const auto other = OwnerFilter::ForProvider(
      scene.ecm, kOther, false);
  ContactMap otherContacts;
  otherContacts[scene.collC][scene.collA].push_back(PointWithExtra(7));
  WriteContacts(scene.ecm, other, otherContacts, false);
  ASSERT_EQ(1, scene.Contacts(scene.collC).contact_size());
  EXPECT_EQ(0, scene.Contacts(scene.collA).contact_size());

  // The default provider reports no contacts for its own collisions. It must
  // not clear collision C.
  scene.ecm.SetAllComponentsUnchanged();
  const auto physics = OwnerFilter::ForProvider(
      scene.ecm, "Default", true);
  WriteContacts(scene.ecm, physics, ContactMap(), false);
  EXPECT_EQ(1, scene.Contacts(scene.collC).contact_size());
  EXPECT_EQ(ComponentState::NoChange, scene.State(scene.collC));

  // Contacts of an entity that the filter rejects are not written.
  ContactMap contacts;
  contacts[scene.collC][scene.collA].push_back(PointWithExtra(8));
  contacts[scene.collA][scene.collC].push_back(PointWithExtra(8));
  WriteContacts(scene.ecm, physics, contacts, false);
  EXPECT_DOUBLE_EQ(7.0,
      scene.Contacts(scene.collC).contact(0).position(0).x());
  EXPECT_EQ(1, scene.Contacts(scene.collA).contact_size());
}

/////////////////////////////////////////////////
TEST(ContactWriter, AcceptAllFilterWritesEveryCollision)
{
  Scene scene;

  // Collision C belongs to a model owned by another provider, and the
  // accept-all filter still writes it and clears the others.
  ContactMap contacts;
  contacts[scene.collC][scene.collA].push_back(PointWithExtra(9));
  WriteContacts(scene.ecm, OwnerFilter(), contacts, false);
  EXPECT_EQ(1, scene.Contacts(scene.collC).contact_size());
  EXPECT_EQ(0, scene.Contacts(scene.collA).contact_size());
}
