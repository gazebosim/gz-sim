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
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/test_config.hh"

#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/WindMode.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test DetachableJoint system
class DetachableJointTest : public InternalFixture<::testing::Test>
{
  public: void StartServer(const std::string &_sdfFile)
  {
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) + _sdfFile);
    this->server = std::make_unique<Server>(serverConfig);

    EXPECT_FALSE(this->server->Running());
    EXPECT_FALSE(*this->server->Running(0));
  }

  public: std::unique_ptr<Server> server;
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(DetachableJointTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(StartConnected))
{
  using namespace std::chrono_literals;

  this->StartServer("/test/worlds/detachable_joint.sdf");

  // A lambda that takes a model name and a mutable reference to a vector of
  // poses and returns another lambda that can be passed to
  // `Relay::OnPostUpdate`.
  auto poseRecorder =
      [](const std::string &_modelName, std::vector<math::Pose3d> &_poses)
  {
    return [&, _modelName](const sim::UpdateInfo &,
                           const sim::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
          [&](const Entity &_entity, const components::Model *,
              const components::Name *_name,
              const components::Pose *_pose) -> bool
          {
            if (_name->Data() == _modelName)
            {
              EXPECT_NE(kNullEntity, _entity);
              _poses.push_back(_pose->Data());
            }
            return true;
          });
    };
  };

  std::vector<math::Pose3d> m1Poses, m2Poses;
  test::Relay testSystem1;
  testSystem1.OnPostUpdate(poseRecorder("M1", m1Poses));
  test::Relay testSystem2;
  testSystem2.OnPostUpdate(poseRecorder("M2", m2Poses));

  this->server->AddSystem(testSystem1.systemPtr);
  this->server->AddSystem(testSystem2.systemPtr);

  const std::size_t nIters{20};
  this->server->Run(true, nIters, false);

  ASSERT_EQ(nIters, m1Poses.size());
  ASSERT_EQ(nIters, m2Poses.size());

  // Model1 is on the ground. It shouldn't move
  EXPECT_EQ(m1Poses.front(), m1Poses.back());

  // Model2 is rigidly connected to Model1 which isn't moving so it should
  // remain at rest.
  EXPECT_EQ(m2Poses.front(), m2Poses.back());

  m1Poses.clear();
  m2Poses.clear();

  transport::Node node;
  auto pub = node.Advertise<msgs::Empty>("/model/M1/detachable_joint/detach");
  pub.Publish(msgs::Empty());
  std::this_thread::sleep_for(250ms);

  const std::size_t nItersAfterDetach{100};
  this->server->Run(true, nItersAfterDetach, false);

  ASSERT_EQ(nItersAfterDetach, m1Poses.size());
  ASSERT_EQ(nItersAfterDetach, m2Poses.size());

  // Model1 is still on the ground. It shouldn't move
  EXPECT_EQ(m1Poses.front(), m1Poses.back());

  // Model2 is now detached. It should be falling
  const double expDist =
      0.5 * 9.8 * pow(static_cast<double>(nItersAfterDetach-1) / 1000, 2);
  // Due integration error, we check that the travelled distance is greater than
  // the expected distance.
  EXPECT_GT(m2Poses.front().Pos().Z() - m2Poses.back().Pos().Z(), expDist);
}

/////////////////////////////////////////////////
TEST_F(DetachableJointTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(LinksInSameModel))
{
  using namespace std::chrono_literals;

  this->StartServer("/test/worlds/detachable_joint.sdf");

  // A lambda that takes a model name and a mutable reference to a vector of
  // poses and returns another lambda that can be passed to
  // `Relay::OnPostUpdate`.
  auto poseRecorder =
      [](const std::string &_linkName, std::vector<math::Pose3d> &_poses)
  {
    return [&, _linkName](const sim::UpdateInfo &,
                          const sim::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Link, components::Name, components::Pose>(
          [&](const Entity &_entity, const components::Link *,
              const components::Name *_name,
              const components::Pose *_pose) -> bool
          {
            if (_name->Data() == _linkName)
            {
              EXPECT_NE(kNullEntity, _entity);
              _poses.push_back(_pose->Data());
            }
            return true;
          });
    };
  };

  std::vector<math::Pose3d> b1Poses, b2Poses;
  test::Relay testSystem1;
  testSystem1.OnPostUpdate(poseRecorder("body1", b1Poses));
  test::Relay testSystem2;
  testSystem2.OnPostUpdate(poseRecorder("body2", b2Poses));

  this->server->AddSystem(testSystem1.systemPtr);
  this->server->AddSystem(testSystem2.systemPtr);

  const std::size_t nIters{20};
  this->server->Run(true, nIters, false);

  ASSERT_EQ(nIters, b1Poses.size());
  ASSERT_EQ(nIters, b2Poses.size());

  // body1 is on the ground. It shouldn't move
  EXPECT_EQ(b1Poses.front(), b1Poses.back());

  // body2 is connected to body1 with a fixed joint so it should remain at rest
  // as well.
  EXPECT_EQ(b2Poses.front(), b2Poses.back());

  b1Poses.clear();
  b2Poses.clear();

  transport::Node node;
  auto pub = node.Advertise<msgs::Empty>("/model/M3/detachable_joint/detach");
  pub.Publish(msgs::Empty());
  std::this_thread::sleep_for(250ms);

  const std::size_t nItersAfterDetach{100};
  this->server->Run(true, nItersAfterDetach, false);

  ASSERT_EQ(nItersAfterDetach, b1Poses.size());
  ASSERT_EQ(nItersAfterDetach, b2Poses.size());

  // body1 is still on the ground. It shouldn't move
  EXPECT_EQ(b1Poses.front(), b1Poses.back());

  // body2 is now detached. It should be falling
  const double expDist =
      0.5 * 9.81 * pow(static_cast<double>(nItersAfterDetach - 1) / 1000, 2);
  // Due integration error, we check that the travelled distance is greater than
  // the expected distance.
  EXPECT_GT(b2Poses.front().Pos().Z() - b2Poses.back().Pos().Z(), expDist);
}
