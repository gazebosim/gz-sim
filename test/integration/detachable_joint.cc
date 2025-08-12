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

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

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
TEST_F(DetachableJointTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(StartConnected))
{
  using namespace std::chrono_literals;

  this->StartServer(common::joinPaths("/test", "worlds",
       "detachable_joint.sdf"));

  // A lambda that takes a model name and a mutable reference to a vector of
  // poses and returns another lambda that can be passed to
  // `Relay::OnPostUpdate`.
  auto poseRecorder =
      [](const std::string &_modelName, std::vector<math::Pose3d> &_poses)
  {
    return [&, _modelName](const UpdateInfo &,
                           const EntityComponentManager &_ecm)
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
TEST_F(DetachableJointTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LinksInSameModel))
{
  using namespace std::chrono_literals;

  this->StartServer(common::joinPaths("/test", "worlds",
       "detachable_joint.sdf"));

  // A lambda that takes a model name and a mutable reference to a vector of
  // poses and returns another lambda that can be passed to
  // `Relay::OnPostUpdate`.
  auto poseRecorder =
      [](const std::string &_linkName, std::vector<math::Pose3d> &_poses)
  {
    return [&, _linkName](const UpdateInfo &,
                          const EntityComponentManager &_ecm)
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

/////////////////////////////////////////////////
TEST_F(DetachableJointTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(NestedModelsWithSameName))
{
  using namespace std::chrono_literals;

  this->StartServer(common::joinPaths("/test", "worlds",
       "detachable_joint_child.sdf"));


  std::vector<math::Pose3d> childM4Poses, childM5Poses;
  test::Relay testSystem1;
  testSystem1.OnPostUpdate([&childM4Poses, &childM5Poses](
    const sim::UpdateInfo &,
    const sim::EntityComponentManager &_ecm)
  {
    auto childModels = _ecm.EntitiesByComponents(
      components::Model(), components::Name("child_model"));

    auto entityM5 = _ecm.EntityByComponents(
      components::Model(), components::Name("M5"));

    Model modelM5(entityM5);
    auto childModelsM5 = modelM5.Models(_ecm);

    Entity childEntityM5{kNullEntity}, childEntityM4{kNullEntity};
    for(auto entity : childModelsM5)
    {
      if (entity == childModels[0])
      {
        childEntityM5 = childModels[0];
        childEntityM4 = childModels[1];
      }
      if (entity == childModels[1])
      {
        childEntityM5 = childModels[1];
        childEntityM4 = childModels[0];
      }
    }

    Model childModelM4(childEntityM4);
    Model childModelM5(childEntityM5);

    auto poseM4 = _ecm.Component<components::Pose>(childEntityM4);
    auto poseM5 = _ecm.Component<components::Pose>(childEntityM5);

    childM4Poses.push_back(poseM4->Data());
    childM5Poses.push_back(poseM5->Data());
  }
  );

  this->server->AddSystem(testSystem1.systemPtr);

  const std::size_t nIters{20};
  this->server->Run(true, nIters, false);

  // Children of model4 and model5 should not move as they are held
  // in place
  EXPECT_EQ(childM4Poses.front(), childM4Poses.back());
  EXPECT_EQ(childM5Poses.front(), childM5Poses.back());

  // Release M5's child only
  transport::Node node;
  auto pub = node.Advertise<msgs::Empty>("/model/M5/detachable_joint/detach");
  pub.Publish(msgs::Empty());
  std::this_thread::sleep_for(250ms);

  this->server->Run(true, nIters, false);
  // M5 and M4 start at the same height
  // Only M5 should fall.
  EXPECT_LT(childM5Poses.back().Z(), childM4Poses.back().Z());
  EXPECT_LT(childM5Poses.back().Z(), childM5Poses.front().Z());
  EXPECT_EQ(childM4Poses.front(), childM4Poses.back());
}
 /////////////////////////////////////////////////
 // Test for re-attaching a detached joint. This uses the vehicle_blue and B1
 // box models. The B1 model is first detached from the vehicle. Although
 // detached, the distance (x-direction) between B1 and vehicle is 1.5, which
 // is the default offset. Then, linear velocity of 1.0 is published on the
 // `/cmd_vel` topic. After 200 iterations, the B1 model will remain in the same
 // position whereas the vehicle will move in the x-direction. Now the
 // distance between B1 and the vehicle will be the default offset (1.5)
 // in addition to the distance traveled by the vehicle. Next, B1 is re-attached
 // to the vehicle. After 200 iterations, we can confirm that B1 has moved with
 // the vehicle and the distance traveled by B1 is close to that of the vehicle.
 // Therefore, it confirms that B1 is re-attached to the vehicle.
 TEST_F(DetachableJointTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(ReAttach))
 {
   using namespace std::chrono_literals;

   this->StartServer(common::joinPaths("/test", "worlds",
       "detachable_joint.sdf"));

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

   std::vector<math::Pose3d> b1Poses, vehiclePoses;
   test::Relay testSystem1;
   testSystem1.OnPostUpdate(poseRecorder("B1", b1Poses));
   test::Relay testSystem2;
   testSystem2.OnPostUpdate(poseRecorder("vehicle_blue", vehiclePoses));

   this->server->AddSystem(testSystem1.systemPtr);
   this->server->AddSystem(testSystem2.systemPtr);

   transport::Node node;
   // time required for the child and parent links to be attached
   gzdbg << "Initially attaching the links" << std::endl;
   const std::size_t nItersInitialize{100};
   this->server->Run(true, nItersInitialize, false);

   // detach the B1 model from the vehicle model
   auto pub = node.Advertise<msgs::Empty>("/B1/detach");
   pub.Publish(msgs::Empty());
   std::this_thread::sleep_for(250ms);
   const std::size_t nItersAfterDetach{100};
   this->server->Run(true, nItersAfterDetach, false);

   ASSERT_EQ(nItersAfterDetach + nItersInitialize, b1Poses.size());
   ASSERT_EQ(nItersAfterDetach + nItersInitialize, vehiclePoses.size());

   // Deafult distance between B1 and the vehicle is 1.5.
   auto defaultDist = 1.5;
   // Although detached, distance (x-axis) between B1 and vehicle should be 1.5.
   EXPECT_NEAR(vehiclePoses.back().Pos().X(),
       abs(b1Poses.back().Pos().X()) - defaultDist, 0.0001);

   // clear the vectors
   b1Poses.clear();
   vehiclePoses.clear();

   // Move the vehicle along the x-axis with linear speed of x = 1.0. Since B1
   // has been detached, it's just the vehicle moving forward.
   auto cmdVelPub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
   msgs::Twist msg;
   msgs::Set(msg.mutable_linear(), math::Vector3d(1.0, 0, 0));
   cmdVelPub.Publish(msg);
   std::this_thread::sleep_for(250ms);
   const std::size_t nItersAfterMoving{200};
   this->server->Run(true, nItersAfterMoving, false);

   ASSERT_EQ(nItersAfterMoving, b1Poses.size());
   ASSERT_EQ(nItersAfterMoving, vehiclePoses.size());

   // Model B1 X pos is stationary. Therefore the diff will be close to 0.
   EXPECT_TRUE(abs(b1Poses.front().Pos().X() -
       b1Poses.back().Pos().X()) < 0.001);

   // Model vehicle_blue X pos will be different since it moved.
   auto distTraveled = 0.1;
   EXPECT_TRUE(abs(vehiclePoses.front().Pos().X() -
       vehiclePoses.back().Pos().X()) > distTraveled);

   // Distance between the B1 and vehicle model confirms that it is detached
   // and the vehicle traveled away from B1.
   auto totalDist = defaultDist + distTraveled;
   EXPECT_TRUE(abs(vehiclePoses.back().Pos().X() -
       b1Poses.back().Pos().X()) > totalDist);

   // clear the vectors
   b1Poses.clear();
   vehiclePoses.clear();

   // Now re-attach the B1 model back to the vehicle. B1 will move with the
   // vehicle.
   auto attachPub = node.Advertise<msgs::Empty>("/B1/attach");
   attachPub.Publish(msgs::Empty());
   std::this_thread::sleep_for(250ms);
   const std::size_t nItersAfterMovingTogether{200};
   this->server->Run(true, nItersAfterMovingTogether, false);

   ASSERT_EQ(nItersAfterMovingTogether, b1Poses.size());
   ASSERT_EQ(nItersAfterMovingTogether, vehiclePoses.size());

   // Model B1 should move along with the vehicle. Therefore the position should
   // change.
   EXPECT_TRUE(abs(b1Poses.front().Pos().X() -
       b1Poses.back().Pos().X()) > distTraveled);

   // distance traveled along the x-axis by the B1 model
   auto distTraveledB1 = abs(b1Poses.back().Pos().X() -
       b1Poses.front().Pos().X());

   // distance traveled along the x-axis by the vehicle model
   auto distTraveledVehicle = abs(vehiclePoses.back().Pos().X() -
       vehiclePoses.front().Pos().X());
   gzdbg << "dist by B1: " << distTraveledB1 << " ,dist by vehicle: "
          << distTraveledVehicle << ", diff: "
          << abs(distTraveledB1 - distTraveledVehicle) << std::endl;

   // since the two models are attached, the distances traveled by both objects
   // should be close.
   EXPECT_TRUE(abs(distTraveledB1 - distTraveledVehicle) < 0.01);
 }
