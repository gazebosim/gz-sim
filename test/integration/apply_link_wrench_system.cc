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

#include <cmath>

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

  // Commands are delivered over transport, which is asynchronous: neither a
  // successful Publish() nor HasConnections() means the ApplyLinkWrench system
  // has processed the message. While `settled` is false the server is being
  // stepped waiting for the latest command to take effect, and no assertions
  // are made. See https://github.com/gazebosim/gz-sim/issues/2727
  bool settled{false};
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
        auto wrenchComp4 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link4.Entity());

        auto linAccel3 = link3.WorldLinearAcceleration(_ecm);
        auto linAccel4 = link4.WorldLinearAcceleration(_ecm);

        const double expected3 = wrenchesCleared ? 0.0 : 50.0;
        const double expected4 = wrenchesCleared ? 0.0 : -100.0;

        // Still waiting for the last published command to reach the system.
        if (!settled)
        {
          settled = nullptr != wrenchComp3 && nullptr != wrenchComp4 &&
              linAccel3.has_value() && linAccel4.has_value() &&
              std::fabs(expected3 - linAccel3.value().X()) < tol &&
              std::fabs(expected4 - linAccel4.value().X()) < tol;
          return;
        }

        EXPECT_NE(nullptr, wrenchComp3);
        EXPECT_NE(nullptr, wrenchComp4);

        ASSERT_TRUE(linAccel3.has_value());
        ASSERT_TRUE(linAccel4.has_value());

        EXPECT_NEAR(expected3, linAccel3.value().X(), tol);
        EXPECT_NEAR(expected4, linAccel4.value().X(), tol);

        if (!wrenchesCleared)
          ++movingIterations;
        else
          ++clearedIterations;

        ++iterations;
      }).Finalize();

  // Step the server until the last published command has taken effect, so that
  // the measured iterations below are not racing transport delivery.
  auto stepUntilSettled = [&]()
  {
    const std::size_t maxWaitIterations{1000};
    for (std::size_t i = 0; i < maxWaitIterations && !settled; ++i)
      fixture.Server()->Run(true, 1, false);
  };

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

  stepUntilSettled();
  ASSERT_TRUE(settled) << "Persistent wrenches were never applied";

  std::size_t targetIterations{100};
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations, iterations);
  EXPECT_EQ(targetIterations, movingIterations);

  // Clear wrenches
  auto pubClear = node.Advertise<msgs::Entity>(
      "/world/apply_link_wrench/wrench/clear");

  for (sleep = 0; !pubClear.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NE(maxSleep, sleep);
  EXPECT_TRUE(pubClear.HasConnections());

  // Expect zero acceleration from now on, and wait for the clear commands to
  // be processed before measuring again.
  wrenchesCleared = true;
  settled = false;

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

  stepUntilSettled();
  ASSERT_TRUE(settled) << "Persistent wrenches were never cleared";

  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations * 2, iterations);
  EXPECT_EQ(targetIterations, movingIterations);
  EXPECT_EQ(targetIterations, clearedIterations);
}

/////////////////////////////////////////////////
TEST_F(ApplyLinkWrenchTestFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(InstantaneousFromTopic))
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "apply_link_wrench.sdf"));

  std::size_t iterations{0};

  // Number of time steps in which each link was accelerated. Delivery over
  // transport is asynchronous, so the impulses are not guaranteed to land on
  // any particular iteration, and the two commands may even land on different
  // ones. What must hold is that each wrench is applied for exactly one time
  // step, with the commanded value.
  // See https://github.com/gazebosim/gz-sim/issues/2727
  std::size_t impulseIterations3{0};
  std::size_t impulseIterations4{0};
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
      const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        auto wrenchComp3 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link3.Entity());
        auto wrenchComp4 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            link4.Entity());

        auto linAccel3 = link3.WorldLinearAcceleration(_ecm);
        ASSERT_TRUE(linAccel3.has_value());

        auto linAccel4 = link4.WorldLinearAcceleration(_ecm);
        ASSERT_TRUE(linAccel4.has_value());

        if (std::fabs(linAccel3.value().X()) > tol)
        {
          EXPECT_NEAR(50.0, linAccel3.value().X(), tol);
          ++impulseIterations3;
        }

        if (std::fabs(linAccel4.value().X()) > tol)
        {
          EXPECT_NEAR(-100.0, linAccel4.value().X(), tol);
          ++impulseIterations4;
        }

        // The component is created when the wrench is first applied to the
        // link, and persists from then on.
        if (impulseIterations3 > 0u)
        {
          EXPECT_NE(nullptr, wrenchComp3);
        }
        if (impulseIterations4 > 0u)
        {
          EXPECT_NE(nullptr, wrenchComp4);
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

  // Step until both impulses have been applied, one iteration at a time so
  // that the single time step each wrench acts on is never stepped over.
  const std::size_t maxWaitIterations{1000};
  std::size_t waited{0};
  for (; waited < maxWaitIterations &&
         (0u == impulseIterations3 || 0u == impulseIterations4); ++waited)
  {
    fixture.Server()->Run(true, 1, false);
  }
  ASSERT_EQ(1u, impulseIterations3) << "model3 wrench was never applied";
  ASSERT_EQ(1u, impulseIterations4) << "model4 wrench was never applied";

  // Keep stepping to confirm neither wrench is applied a second time.
  std::size_t targetIterations{20};
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations + waited, iterations);
  EXPECT_EQ(1u, impulseIterations3);
  EXPECT_EQ(1u, impulseIterations4);
}
