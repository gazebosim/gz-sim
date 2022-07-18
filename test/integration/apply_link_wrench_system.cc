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

#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/entity_wrench.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"

#include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/TestFixture.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/EnvTestFixture.hh"

#define tol 10e-4

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Test fixture for ApplyLinkWrench system
class ApplyLinkWrenchTestFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Persistent wrench defined on SDF file
TEST_F(ApplyLinkWrenchTestFixture, FromSdf)
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "apply_link_wrench.sdf"));

  std::size_t iterations{0};
  // TODO CHECK ACCELERATIONS INSTEAD OF POSES
  math::Pose3d prevPose1;
  math::Pose3d prevPose2;
  fixture.OnPostUpdate([&](
      const gazebo::UpdateInfo &_info,
      const gazebo::EntityComponentManager &_ecm)
      {
        Model model1(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model1")));
        EXPECT_TRUE(model1.Valid(_ecm));

        Model model2(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model2")));
        EXPECT_TRUE(model2.Valid(_ecm));

        auto pose1 = worldPose(model1.Entity(), _ecm);
        auto pose2 = worldPose(model2.Entity(), _ecm);

        auto wrenchComp1 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            model1.CanonicalLink(_ecm));
        auto wrenchComp2 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            model2.CanonicalLink(_ecm));

        EXPECT_NE(nullptr, wrenchComp1);
        EXPECT_NE(nullptr, wrenchComp2);

        if (_info.iterations == 1)
        {
          EXPECT_NEAR(pose1.Pos().X(), 0.0, tol);
          EXPECT_NEAR(pose1.Rot().Yaw(), 0.0, tol);

          EXPECT_NEAR(pose2.Pos().X(), 0.0, tol);
          EXPECT_NEAR(pose2.Rot().Yaw(), 0.0, tol);
        }
        else
        {
          EXPECT_GT(pose1.Pos().X(), prevPose1.Pos().X());
          EXPECT_GT(pose1.Rot().Yaw(), prevPose1.Rot().Yaw());

          EXPECT_LT(pose2.Pos().X(), prevPose2.Pos().X());
          EXPECT_LT(pose2.Rot().Yaw(), prevPose2.Rot().Yaw());

          // model 2 moves faster than 1
          EXPECT_LT(std::abs(pose1.Pos().X()), std::abs(pose2.Pos().X()));
          EXPECT_LT(std::abs(pose1.Rot().Yaw()), std::abs(pose2.Rot().Yaw()));
        }

        EXPECT_NEAR(pose1.Pos().Y(), 0.0, tol);
        EXPECT_NEAR(pose1.Pos().Z(), 0.5, tol);
        EXPECT_NEAR(pose1.Rot().Roll(), 0.0, tol);
        EXPECT_NEAR(pose1.Rot().Pitch(), 0.0, tol);

        EXPECT_NEAR(pose2.Pos().Y(), 2.0, tol);
        EXPECT_NEAR(pose2.Pos().Z(), 0.5, tol);
        EXPECT_NEAR(pose2.Rot().Roll(), 0.0, tol);
        EXPECT_NEAR(pose2.Rot().Pitch(), 0.0, tol);

        prevPose1 = pose1;
        ++iterations;
      }).Finalize();

  std::size_t targetIterations{100};
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations, iterations);
}

/////////////////////////////////////////////////
// Wrench set from topic
TEST_F(ApplyLinkWrenchTestFixture, FromTopic)
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "apply_link_wrench.sdf"));

  std::size_t iterations{0};
  std::size_t movingIterations3{0};
  std::size_t movingIterations4{0};
  std::size_t clearedIterations3{0};
  std::size_t clearedIterations4{0};
  math::Pose3d prevPose3;
  math::Pose3d prevPose4;
  bool wrenchesCleared{false};
  fixture.OnPostUpdate([&](
      const gazebo::UpdateInfo &,
      const gazebo::EntityComponentManager &_ecm)
      {
        Model model3(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model3")));
        EXPECT_TRUE(model3.Valid(_ecm));

        Model model4(_ecm.EntityByComponents(components::Model(),
                                             components::Name("model4")));
        EXPECT_TRUE(model4.Valid(_ecm));

        auto pose3 = worldPose(model3.Entity(), _ecm);
        auto pose4 = worldPose(model4.Entity(), _ecm);

        auto wrenchComp3 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            model3.CanonicalLink(_ecm));
        auto wrenchComp4 = _ecm.Component<components::ExternalWorldWrenchCmd>(
            model4.CanonicalLink(_ecm));

        if (wrenchComp3 == nullptr)
        {
          EXPECT_NEAR(pose3.Pos().X(), 0.0, tol) << iterations;
          EXPECT_NEAR(pose3.Rot().Yaw(), 0.0, tol) << iterations;
        }
        else if (!wrenchesCleared)
        {
          EXPECT_GT(pose3.Pos().X(), prevPose3.Pos().X()) << iterations;
          EXPECT_GT(pose3.Rot().Yaw(), prevPose3.Rot().Yaw()) << iterations;

          ++movingIterations3;
        }
        else
        {
          EXPECT_NEAR(pose3.Pos().X(), prevPose3.Pos().X(), tol) << iterations;
          EXPECT_NEAR(pose3.Rot().Yaw(), prevPose3.Rot().Yaw(), tol) << iterations;

          ++clearedIterations3;
        }

        if (wrenchComp4 == nullptr)
        {
          EXPECT_NEAR(pose4.Pos().X(), 0.0, tol) << iterations;
          EXPECT_NEAR(pose4.Rot().Yaw(), 0.0, tol) << iterations;
        }
        else if (!wrenchesCleared)
        {
          EXPECT_LT(pose4.Pos().X(), prevPose4.Pos().X()) << iterations;
          EXPECT_LT(pose4.Rot().Yaw(), prevPose4.Rot().Yaw()) << iterations;

          ++movingIterations4;
        }
        else
        {
          EXPECT_NEAR(pose4.Pos().X(), prevPose4.Pos().X(), tol) << iterations;
          EXPECT_NEAR(pose4.Rot().Yaw(), prevPose4.Rot().Yaw(), tol) << iterations;

          ++clearedIterations4;
        }

        if (wrenchComp3 != nullptr && wrenchComp4 != nullptr)
        {
          if (!wrenchesCleared)
          {
            EXPECT_GT(pose3.Pos().X(), prevPose3.Pos().X());
            EXPECT_GT(pose3.Rot().Yaw(), prevPose3.Rot().Yaw());

            EXPECT_LT(pose4.Pos().X(), prevPose4.Pos().X());
            EXPECT_LT(pose4.Rot().Yaw(), prevPose4.Rot().Yaw());

            // model 4 moves faster than 3
            EXPECT_LT(std::abs(pose3.Pos().X()), std::abs(pose4.Pos().X()))
                << iterations;
            EXPECT_LT(std::abs(pose3.Rot().Yaw()), std::abs(pose4.Rot().Yaw()))
                << iterations;
          }
          else
          {
            // EXPECT_NEAR(pose3.Pos().X(), prevPose3.Pos().X(), tol);
            // EXPECT_NEAR(pose3.Rot().Yaw(), prevPose3.Rot().Yaw(), tol);

            // EXPECT_NEAR(pose4.Pos().X(), prevPose4.Pos().X(), tol);
            // EXPECT_NEAR(pose4.Rot().Yaw(), prevPose4.Rot().Yaw(), tol);
          }
        }

        EXPECT_NEAR(pose3.Pos().Y(), 4.0, tol);
        EXPECT_NEAR(pose3.Pos().Z(), 0.5, tol);
        EXPECT_NEAR(pose3.Rot().Roll(), 0.0, tol);
        EXPECT_NEAR(pose3.Rot().Pitch(), 0.0, tol);

        EXPECT_NEAR(pose4.Pos().Y(), 6.0, tol);
        EXPECT_NEAR(pose4.Pos().Z(), 0.5, tol);
        EXPECT_NEAR(pose4.Rot().Roll(), 0.0, tol);
        EXPECT_NEAR(pose4.Rot().Pitch(), 0.0, tol);

        if (!wrenchesCleared)
        {
          prevPose3 = pose3;
          prevPose4 = pose4;
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

  auto pubWrench = node.Advertise<msgs::EntityWrench>(
      "/world/apply_link_wrench/wrench");

  std::size_t targetIterations{100};
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations, iterations);
  EXPECT_GT(movingIterations3, 0u);
  EXPECT_GT(movingIterations4, 0u);
  EXPECT_LE(movingIterations3, iterations);
  EXPECT_LE(movingIterations4, iterations);
  EXPECT_GE(movingIterations3, movingIterations4);

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

  wrenchesCleared = true;
  fixture.Server()->Run(true, targetIterations, false);
  EXPECT_EQ(targetIterations * 2, iterations);
}

