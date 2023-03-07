/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"

#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/TestFixture.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

#define tol 10e-4

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test fixture for ApplyLinkWrench system
class ApplyLinkWrenchTestFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ApplyLinkWrenchTestFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(FromSdf))
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "apply_link_wrench.sdf"));

  std::size_t iterations{0};
  Link link1, link2;
  fixture.OnConfigure([&](
      const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &_ecm,
      EventManager &)
      {
        Model model1(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model1")));
        EXPECT_TRUE(model1.Valid(_ecm));

        link1 = Link(model1.CanonicalLink(_ecm));
        EXPECT_TRUE(link1.Valid(_ecm));
        link1.EnableAccelerationChecks(_ecm);

        Model model2(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model2")));
        EXPECT_TRUE(model2.Valid(_ecm));

        link2 = Link(model2.CanonicalLink(_ecm));
        EXPECT_TRUE(link2.Valid(_ecm));
        link2.EnableAccelerationChecks(_ecm);
      })
  .OnPostUpdate([&](
      const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        auto wrenchComp1 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link1.Entity());
        auto wrenchComp2 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link2.Entity());

        EXPECT_NE(nullptr, wrenchComp1);
        EXPECT_NE(nullptr, wrenchComp2);

        auto linAccel1 = link1.WorldLinearAcceleration(_ecm);
        ASSERT_TRUE(linAccel1.has_value());
        EXPECT_NEAR(50.0, linAccel1.value().X(), tol);

        auto linAccel2 = link2.WorldLinearAcceleration(_ecm);
        ASSERT_TRUE(linAccel2.has_value());
        EXPECT_NEAR(-100.0, linAccel2.value().X(), tol);

        ++iterations;
      }).Finalize();

  std::size_t targetIterations{100};
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations, iterations);
}

/////////////////////////////////////////////////
TEST_F(ApplyLinkWrenchTestFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(PersistentFromTopic))
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "apply_link_wrench.sdf"));

  std::size_t iterations{0};
  std::size_t movingIterations{0};
  std::size_t clearedIterations{0};
  bool wrenchesCleared{false};
  Link link3, link4;
  fixture.OnConfigure([&](
      const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &_ecm,
      EventManager &)
      {
        Model model3(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model3")));
        EXPECT_TRUE(model3.Valid(_ecm));

        link3 = Link(model3.CanonicalLink(_ecm));
        EXPECT_TRUE(link3.Valid(_ecm));
        link3.EnableAccelerationChecks(_ecm);

        Model model4(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model4")));
        EXPECT_TRUE(model4.Valid(_ecm));

        link4 = Link(model4.CanonicalLink(_ecm));
        EXPECT_TRUE(link4.Valid(_ecm));
        link4.EnableAccelerationChecks(_ecm);
        link4 = Link(model4.CanonicalLink(_ecm));
        EXPECT_TRUE(link4.Valid(_ecm));
        link4.EnableAccelerationChecks(_ecm);
      })
  .OnPostUpdate([&](
      const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        auto wrenchComp3 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link3.Entity());
        EXPECT_NE(nullptr, wrenchComp3);

        auto wrenchComp4 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link4.Entity());
        EXPECT_NE(nullptr, wrenchComp4);

        auto linAccel3 = link3.WorldLinearAcceleration(_ecm);
        ASSERT_TRUE(linAccel3.has_value());

        auto linAccel4 = link4.WorldLinearAcceleration(_ecm);
        ASSERT_TRUE(linAccel4.has_value());

        if (!wrenchesCleared)
        {
          EXPECT_NEAR(50.0, linAccel3.value().X(), tol);
          EXPECT_NEAR(-100.0, linAccel4.value().X(), tol);

          ++movingIterations;
        }
        else
        {
          EXPECT_NEAR(0.0, linAccel3.value().X(), tol);
          EXPECT_NEAR(0.0, linAccel4.value().X(), tol);

          ++clearedIterations;
        }

        ++iterations;
      }).Finalize();

  // Publish messages
  transport::Node node;
  auto pubPersistent = node.Advertise<msgs::EntityWrench>(
      "/world/apply_link_wrench/wrench/persistent");

  int sleep{0};
  int maxSleep{30};
  for (; !pubPersistent.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NE(maxSleep, sleep);
  EXPECT_TRUE(pubPersistent.HasConnections());

  {
    msgs::EntityWrench msg;
    msg.mutable_entity()->set_name("model3");
    msg.mutable_entity()->set_type(msgs::Entity::MODEL);
    msg.mutable_wrench()->mutable_force()->set_x(50);
    msg.mutable_wrench()->mutable_torque()->set_z(0.5);
    pubPersistent.Publish(msg);
  }

  {
    msgs::EntityWrench msg;
    msg.mutable_entity()->set_name("model4::link");
    msg.mutable_entity()->set_type(msgs::Entity::LINK);
    msg.mutable_wrench()->mutable_force()->set_x(-100);
    msg.mutable_wrench()->mutable_torque()->set_z(-1.0);
    pubPersistent.Publish(msg);
  }

  std::size_t targetIterations{100};
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations, iterations);
  EXPECT_EQ(movingIterations, iterations);

  // Clear wrenches
  auto pubClear = node.Advertise<msgs::Entity>(
      "/world/apply_link_wrench/wrench/clear");
  EXPECT_TRUE(pubClear.HasConnections());

  {
    msgs::Entity msg;
    msg.set_name("model3");
    msg.set_type(msgs::Entity::MODEL);
    pubClear.Publish(msg);
  }

  {
    msgs::Entity msg;
    msg.set_name("model4::link");
    msg.set_type(msgs::Entity::LINK);
    pubClear.Publish(msg);
  }

  // \todo(chapulina) Arbitrarily sleeping here isn't very robust
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  wrenchesCleared = true;
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations * 2, iterations);
  EXPECT_EQ(movingIterations, targetIterations);
  EXPECT_EQ(clearedIterations, targetIterations);
}

/////////////////////////////////////////////////
TEST_F(ApplyLinkWrenchTestFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(InstantaneousFromTopic))
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "apply_link_wrench.sdf"));

  std::size_t iterations{0};
  std::size_t impulseIterations{0};
  Link link3, link4;
  fixture.OnConfigure([&](
      const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &_ecm,
      EventManager &)
      {
        Model model3(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model3")));
        EXPECT_TRUE(model3.Valid(_ecm));

        link3 = Link(model3.CanonicalLink(_ecm));
        EXPECT_TRUE(link3.Valid(_ecm));
        link3.EnableAccelerationChecks(_ecm);

        Model model4(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model4")));
        EXPECT_TRUE(model4.Valid(_ecm));

        link4 = Link(model4.CanonicalLink(_ecm));
        EXPECT_TRUE(link4.Valid(_ecm));
        link4.EnableAccelerationChecks(_ecm);
      })
  .OnPostUpdate([&](
      const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
      {
        auto wrenchComp3 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link3.Entity());
        EXPECT_NE(nullptr, wrenchComp3);

        auto wrenchComp4 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link4.Entity());
        EXPECT_NE(nullptr, wrenchComp4);

        auto linAccel3 = link3.WorldLinearAcceleration(_ecm);
        ASSERT_TRUE(linAccel3.has_value());

        auto linAccel4 = link4.WorldLinearAcceleration(_ecm);
        ASSERT_TRUE(linAccel4.has_value());

        if (_info.iterations == 1)
        {
          EXPECT_NEAR(50.0, linAccel3.value().X(), tol);
          EXPECT_NEAR(-100.0, linAccel4.value().X(), tol);

          ++impulseIterations;
        }
        else
        {
          EXPECT_NEAR(0.0, linAccel3.value().X(), tol);
          EXPECT_NEAR(0.0, linAccel4.value().X(), tol);
        }

        ++iterations;
      }).Finalize();

  // Publish messages
  transport::Node node;
  auto pubWrench = node.Advertise<msgs::EntityWrench>(
      "/world/apply_link_wrench/wrench");

  int sleep{0};
  int maxSleep{30};
  for (; !pubWrench.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NE(maxSleep, sleep);
  EXPECT_TRUE(pubWrench.HasConnections());

  {
    msgs::EntityWrench msg;
    msg.mutable_entity()->set_name("model3");
    msg.mutable_entity()->set_type(msgs::Entity::MODEL);
    msg.mutable_wrench()->mutable_force()->set_x(50);
    msg.mutable_wrench()->mutable_torque()->set_z(0.5);
    pubWrench.Publish(msg);
  }

  {
    msgs::EntityWrench msg;
    msg.mutable_entity()->set_name("model4::link");
    msg.mutable_entity()->set_type(msgs::Entity::LINK);
    msg.mutable_wrench()->mutable_force()->set_x(-100);
    msg.mutable_wrench()->mutable_torque()->set_z(-1.0);
    pubWrench.Publish(msg);
  }

  std::size_t targetIterations{20};
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations, iterations);
  EXPECT_EQ(1u, impulseIterations);
}
