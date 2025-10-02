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
 * Author: Adarsh Karan K P, Neobotix GmbH
 */

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/dynamic_detachable_joint.pb.h>
#include <gz/msgs/pose.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test DynamicDetachableJoint system
class DynamicDetachableJointTest : public InternalFixture<::testing::Test>
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

// See https://github.com/gazebosim/gz-sim/issues/1175 for GZ_UTILS_TEST_DISABLED_ON_WIN32(AttachDetach)

/////////////////////////////////////////////////
// TEST 1
// "AttachDetach"
// This test checks the following:
// 1. attach when within threshold → child (red_cube) follows parent, meaning when the rod moves up (+Z axis) the child moves along with it
// 2. detach → child free-falls due to gravity
TEST_F(DynamicDetachableJointTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(AttachDetach))
{
  using namespace std::chrono_literals;

  // load the test world
  this->StartServer("/test/worlds/dynamic_detachable_joint.sdf");

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

  // the arm (aka "rod") is the parent, the cube is the child
  std::vector<math::Pose3d> rodPoses, cubePoses;
  test::Relay rodRelay, cubeRelay;
  rodRelay.OnPostUpdate(poseRecorder("rod", rodPoses));
  cubeRelay.OnPostUpdate(poseRecorder("red_cube", cubePoses));

  this->server->AddSystem(rodRelay.systemPtr);
  this->server->AddSystem(cubeRelay.systemPtr);

  // Short wait time to ensure everything is initialized
  const std::size_t waitIters{20};
  this->server->Run(true, waitIters, false);
  ASSERT_EQ(waitIters, cubePoses.size());
  ASSERT_EQ(waitIters, rodPoses.size());
  gzdbg << "[startup_wait] rod z=" << rodPoses.back().Pos().Z()
        << ", cube z=" << cubePoses.back().Pos().Z() << std::endl;

  transport::Node node;

  // "attach" red_cube(which is on the ground) to the "rod" using the DynamicDetachableJoint service
  {
    msgs::AttachDetachRequest req;
    req.set_child_model_name("red_cube");
    req.set_child_link_name("link");
    req.set_command("attach");
    gzdbg << "[attach] request: model=red_cube link=link" << std::endl;

    msgs::AttachDetachResponse rep;
    bool result_1 = false;
    bool outcome_1 = node.Request("/payload/attach_detach", 
      req, 1000u, rep, result_1);

    // verify service call succeeded
    ASSERT_TRUE(outcome_1);
    ASSERT_TRUE(result_1);
    ASSERT_TRUE(rep.success()) << rep.message();
    gzdbg << "[attach] ok" << std::endl;
  }

  // Wait
  { 
    // Reset poses
    cubePoses.clear();
    rodPoses.clear();

    const std::size_t iters{60};
    this->server->Run(true, iters, false);

    // Verify if we recorded expected number of poses
    ASSERT_EQ(iters, cubePoses.size());
    ASSERT_EQ(iters, rodPoses.size());

    // Additional check to ensure the red_cube is rigidly attached to the rod by comparing relative pose
    auto rel0 = cubePoses.front().Pos() - rodPoses.front().Pos();
    auto rel1 = cubePoses.back().Pos()  - rodPoses.back().Pos();
    EXPECT_LT((rel1 - rel0).Length(), 5e-4);
    gzdbg << "[attached-idle] rel Δ=" << (rel1 - rel0).Length()
          << " (expect ~0)" << std::endl;
  }

  // "lift" the rod up in Z so the attached red_cube is above ground
  {
    gzdbg << "[lift] moving rod from z 0.14m to  0.35m" << std::endl;
    msgs::Pose poseReq;
    poseReq.set_name("rod");
    // keep x,y (same as sdf) and update only z
    auto *pos = poseReq.mutable_position();
    pos->set_x(0.30); pos->set_y(0.0); pos->set_z(0.35);
    auto *ori = poseReq.mutable_orientation();
    ori->set_x(0); ori->set_y(0); ori->set_z(0); ori->set_w(1);

    msgs::Boolean rep;
    bool result_2 = false;
    bool outcome_2 = node.Request("/world/dynamic_detachable_joint_test/set_pose", 
      poseReq, 1000u, rep, result_2);

    // verify service call succeeded
    ASSERT_TRUE(outcome_2);
    ASSERT_TRUE(result_2);
    ASSERT_TRUE(rep.data());
    gzdbg << "[lift] pose service ok" << std::endl;
  }

  // After lifting, child follows upward along with the parent rod
  // this causes it's Z position to change
  double pos_before_lift = cubePoses.back().Pos().Z();
  {
    cubePoses.clear();
    rodPoses.clear();
    const std::size_t iters{50};
    this->server->Run(true, iters, false);
    ASSERT_EQ(iters, cubePoses.size());
    ASSERT_EQ(iters, rodPoses.size());

    // Track how much the red_cube moved up
    auto movedZ = cubePoses.back().Pos().Z() - pos_before_lift;
    EXPECT_GT(movedZ, 0.15);  // followed rod upwards
    gzdbg << "[after-lift] red_cube Δz=" << movedZ << " (expect ~0.21)" << std::endl;

    // Additional check to ensure the red_cube is rigidly attached to the rod by comparing relative pose
    auto rel0 = cubePoses.front().Pos() - rodPoses.front().Pos();
    auto rel1 = cubePoses.back().Pos()  - rodPoses.back().Pos();
    EXPECT_LT((rel1 - rel0).Length(), 5e-4);
  }

  // "detach" and verify free fall of the red_cube
  {
    gzdbg << "[detach] request: model=red_cube link=link" << std::endl;
    msgs::AttachDetachRequest req;
    req.set_child_model_name("red_cube");
    req.set_child_link_name("link");
    req.set_command("detach");

    msgs::AttachDetachResponse rep;
    bool result_3 = false;
    bool outcome_3 = node.Request("/payload/attach_detach", 
      req, 1000u, rep, result_3);

    // verify service call succeeded
    ASSERT_TRUE(outcome_3);
    ASSERT_TRUE(result_3);
    ASSERT_TRUE(rep.success()) << rep.message();
    gzdbg << "[detach] ok" << std::endl;
  }

  // The ideal distance an object falls under gravity is given as d = 0.5 * g * t^2
  // Let it fall for about 150 ms, then check if dz (z distance travelled) > d
  {
    cubePoses.clear();
    const std::size_t iters{150};
    this->server->Run(true, iters, false);
    ASSERT_EQ(iters, cubePoses.size());

    const double dt = 0.001; // 1 ms from the test world
    const double t = static_cast<double>(iters - 1) * dt; // N poses give N-1 integration intervals
    const double d = 0.5 * 9.8 * t * t;
    const double dz = cubePoses.front().Pos().Z() - cubePoses.back().Pos().Z();
    
    // Check if the red_cube fell "at least" the ideal distance
    EXPECT_GT(dz, d);
    gzdbg << "[free-fall] dz=" << dz << " > " << d
          << "  (t=" << t << "s, N=" << iters << ")" << std::endl;
  }
}

/////////////////////////////////////////////////
// TEST 2
// "OutOfRange"
// This test checks the following:
// 1. move the parent (rod) far enough so that the center-to-center distance
//    to the red_cube exceeds the configured attach_distance (0.1 m)
// 2. call "attach" → service accepts the request (rep.success() == true);
// 3. verify the target child "red_cube" stayed on the ground (~z = 0.05)
TEST_F(DynamicDetachableJointTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(OutOfRange))
{
  using namespace std::chrono_literals;

  this->StartServer("/test/worlds/dynamic_detachable_joint.sdf");

  // A lambda that records poses for a named model
  auto poseRecorder =
      [](const std::string &_name, std::vector<math::Pose3d> &_poses)
  {
    return [&, _name](const UpdateInfo &, const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
          [&](const Entity &, const components::Model *,
              const components::Name *n, const components::Pose *p) -> bool
          {
            if (n->Data() == _name) _poses.push_back(p->Data());
            return true;
          });
    };
  };

  std::vector<math::Pose3d> rodPoses, cubePoses;
  test::Relay rodRelay, cubeRelay;
  rodRelay.OnPostUpdate(poseRecorder("rod", rodPoses));
  cubeRelay.OnPostUpdate(poseRecorder("red_cube", cubePoses));

  this->server->AddSystem(rodRelay.systemPtr);
  this->server->AddSystem(cubeRelay.systemPtr);

  // Short wait time to ensure everything is initialized
  const std::size_t waitIters{20};
  this->server->Run(true, waitIters, false);
  ASSERT_EQ(waitIters, rodPoses.size());
  ASSERT_EQ(waitIters, cubePoses.size());
  gzdbg << "[startup_wait] rod z=" << rodPoses.back().Pos().Z()
        << ", red_cube z=" << cubePoses.back().Pos().Z() << std::endl;

  // Move rod high enough so center distance > 0.1 m (attach_distance).
  // red_cube remains at center z = 0.05m and set rod z to 0.50m → gap = 0.45 m.
  transport::Node node;
  {
    msgs::Pose poseReq;
    poseReq.set_name("rod");
    auto *pos = poseReq.mutable_position();
    pos->set_x(0.30); pos->set_y(0.0); pos->set_z(0.50);
    auto *ori = poseReq.mutable_orientation();
    ori->set_x(0); ori->set_y(0); ori->set_z(0); ori->set_w(1);

    msgs::Boolean rep;
    bool result_1 = false;
    bool outcome_1 = node.Request("/world/dynamic_detachable_joint_test/set_pose",
      poseReq, 1000u, rep, result_1);

    // verify service call succeeded
    ASSERT_TRUE(outcome_1);
    ASSERT_TRUE(result_1);
    ASSERT_TRUE(rep.data());
    gzdbg << "[lift-far] pose service ok (rod z=0.50)" << std::endl;
  }
  // Let the pose update apply before attempting attach
  this->server->Run(true, 5, false);

  // Additional check to ensure if gap is actually larger than attach_distance (0.1 m)
  {
    ASSERT_FALSE(rodPoses.empty());
    ASSERT_FALSE(cubePoses.empty());
    const double gap = std::abs(rodPoses.back().Pos().Z() - cubePoses.back().Pos().Z());
    ASSERT_GT(gap, 0.1 + 1e-3);
  }

  // Attempt to "attach" → request is processed, but attach is rejected in PreUpdate of the plugin
  // Error is logged if out of range, and we verify by poses that nothing attached
  {
    msgs::AttachDetachRequest req;
    req.set_child_model_name("red_cube");
    req.set_child_link_name("link");
    req.set_command("attach");

    msgs::AttachDetachResponse rep;
    bool result_2 = false;
    bool outcome_2 = node.Request("/payload/attach_detach",
      req, 1000u, rep, result_2);

    // verify service call succeeded
    ASSERT_TRUE(outcome_2);
    ASSERT_TRUE(result_2);
    gzdbg << "[attach-out-of-range] request processed and an error is logged if out of range"
          << std::endl;
  }

  // Additional check to ensure red_cube is still on the ground
  // red_cube should still be on the ground (~0.05)
  {
    cubePoses.clear();
    const std::size_t iters{40};
    this->server->Run(true, iters, false);
    ASSERT_EQ(iters, cubePoses.size());
    EXPECT_NEAR(cubePoses.back().Pos().Z(), 0.05, 1e-2);
    gzdbg << "[verify-ground] red_cube z=" << cubePoses.back().Pos().Z()
          << " (expect ~0.05)" << std::endl;
  }
}

/////////////////////////////////////////////////
// TEST 3
// "MultipleChildrenSequential"
// This test checks the following
// 1. attach lift detach each target child in turn (red_cube, green_cube, blue_cube)
// 2. only the attached child follows the parent rod when it moves up (+Z axis)
// 3. after detaching the target child free-falls due to gravity
// 4. non-target children stay on the ground (~z = 0.05)
TEST_F(DynamicDetachableJointTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(MultipleChildrenSequential))
{
  using namespace std::chrono_literals;

  this->StartServer("/test/worlds/dynamic_detachable_joint.sdf");

  // Record poses for rod and all 3 cubes
  auto poseRecorder =
      [](const std::string &_name, std::vector<math::Pose3d> &_poses)
  {
    return [&, _name](const UpdateInfo &, const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name, components::Pose>(
          [&](const Entity &, const components::Model *,
              const components::Name *n, const components::Pose *p) -> bool
          {
            if (n->Data() == _name) _poses.push_back(p->Data());
            return true;
          });
    };
  };

  std::vector<math::Pose3d> rodPoses, redPoses, greenPoses, bluePoses;
  test::Relay rodRelay, redRelay, greenRelay, blueRelay;
  rodRelay.OnPostUpdate(poseRecorder("rod", rodPoses));
  redRelay.OnPostUpdate(poseRecorder("red_cube", redPoses));
  greenRelay.OnPostUpdate(poseRecorder("green_cube", greenPoses));
  blueRelay.OnPostUpdate(poseRecorder("blue_cube", bluePoses));

  this->server->AddSystem(rodRelay.systemPtr);
  this->server->AddSystem(redRelay.systemPtr);
  this->server->AddSystem(greenRelay.systemPtr);
  this->server->AddSystem(blueRelay.systemPtr);

  // wait to ensure everything is initialized
  const std::size_t waitIters{20};
  this->server->Run(true, waitIters, false);
  ASSERT_EQ(waitIters, rodPoses.size());
  ASSERT_EQ(waitIters, redPoses.size());
  ASSERT_EQ(waitIters, greenPoses.size());
  ASSERT_EQ(waitIters, bluePoses.size());

  transport::Node node;

  struct Target { const char *name; double x; std::vector<math::Pose3d>* poses; };
  std::array<Target,3> targets{{
    {"red_cube",   0.30, &redPoses},
    {"green_cube", 0.10, &greenPoses},
    {"blue_cube",  0.50, &bluePoses},
  }};

  auto setRodPose = [&](double x, double z)
  {
    msgs::Pose poseReq;
    poseReq.set_name("rod");
    auto *pos = poseReq.mutable_position();
    pos->set_x(x); pos->set_y(0.0); pos->set_z(z);
    auto *ori = poseReq.mutable_orientation();
    ori->set_x(0); ori->set_y(0); ori->set_z(0); ori->set_w(1);

    msgs::Boolean rep;
    bool result_pose = false;
    bool outcome_pose = node.Request("/world/dynamic_detachable_joint_test/set_pose",
      poseReq, 1000u, rep, result_pose);
    
    // verify service call succeeded
    ASSERT_TRUE(outcome_pose);
    ASSERT_TRUE(result_pose);
    ASSERT_TRUE(rep.data());
  };

  auto nearGround = [](double z){ return std::abs(z - 0.05) < 1e-2; };

  for (const auto &target : targets)
  {
    // Move rod above target within attach_distance (0.1 m)
    setRodPose(target.x, 0.14);
    this->server->Run(true, 5, false);

    // "attach" target child to the rod using the service
    {
      gzdbg << "[attach] request: model=" << target.name << " link=link" << std::endl;

      msgs::AttachDetachRequest req;
      req.set_child_model_name(target.name);
      req.set_child_link_name("link");
      req.set_command("attach");

      msgs::AttachDetachResponse rep;
      bool result_attach = false;
      bool outcome_attach = node.Request("/payload/attach_detach",
        req, 1000u, rep, result_attach);

      ASSERT_TRUE(outcome_attach);
      ASSERT_TRUE(result_attach);
      ASSERT_TRUE(rep.success()) << rep.message();
      gzdbg << "[attach] ok target=" << target.name << std::endl;
    }

    // Step and verify 
    // 1. the child is rigidly attached to parent
    // 2. other target cubes stay on ground
    {
      rodPoses.clear(); redPoses.clear(); greenPoses.clear(); bluePoses.clear();
      const std::size_t iters{30};
      this->server->Run(true, iters, false);

      ASSERT_EQ(iters, rodPoses.size());
      ASSERT_EQ(iters, redPoses.size());
      ASSERT_EQ(iters, greenPoses.size());
      ASSERT_EQ(iters, bluePoses.size());

      const auto &v = *target.poses;
      auto rel0 = v.front().Pos() - rodPoses.front().Pos();
      auto rel1 = v.back().Pos()  - rodPoses.back().Pos();
      EXPECT_LT((rel1 - rel0).Length(), 5e-4);

      if (target.name != std::string("red_cube"))
      {
        EXPECT_TRUE(nearGround(redPoses.back().Pos().Z()));
      }
      if (target.name != std::string("green_cube"))
      {
        EXPECT_TRUE(nearGround(greenPoses.back().Pos().Z()));
      }
      if (target.name != std::string("blue_cube"))
      {
        EXPECT_TRUE(nearGround(bluePoses.back().Pos().Z()));
      }
    }

    // "lift" the rod to z 0.35 and verify only target follows upward
    double pos_before_lift = target.poses->back().Pos().Z();
    setRodPose(target.x, 0.35);
    {
      rodPoses.clear(); redPoses.clear(); greenPoses.clear(); bluePoses.clear();
      const std::size_t iters{50};
      this->server->Run(true, iters, false);

      ASSERT_EQ(iters, rodPoses.size());
      ASSERT_EQ(iters, redPoses.size());
      ASSERT_EQ(iters, greenPoses.size());
      ASSERT_EQ(iters, bluePoses.size());

      double movedZ = target.poses->back().Pos().Z() - pos_before_lift;
      EXPECT_GT(movedZ, 0.15);

      if (target.name != std::string("red_cube"))
      {
        EXPECT_TRUE(nearGround(redPoses.back().Pos().Z()));
      }
      if (target.name != std::string("green_cube"))
      {
        EXPECT_TRUE(nearGround(greenPoses.back().Pos().Z()));
      }
      if (target.name != std::string("blue_cube"))
      {
        EXPECT_TRUE(nearGround(bluePoses.back().Pos().Z()));
      }
    }

    // "detach" the target and verify free-fall
    {
      gzdbg << "[detach] request: model=" << target.name << " link=link" << std::endl;

      msgs::AttachDetachRequest req;
      req.set_child_model_name(target.name);
      req.set_child_link_name("link");
      req.set_command("detach");

      msgs::AttachDetachResponse rep;
      bool result_detach = false;
      bool outcome_detach = node.Request("/payload/attach_detach",
        req, 1000u, rep, result_detach);

      ASSERT_TRUE(outcome_detach);
      ASSERT_TRUE(result_detach);
      ASSERT_TRUE(rep.success()) << rep.message();
      gzdbg << "[detach] ok target=" << target.name << std::endl;
    }
    {
      rodPoses.clear(); redPoses.clear(); greenPoses.clear(); bluePoses.clear();
      const std::size_t iters{150};
      this->server->Run(true, iters, false);

      ASSERT_EQ(iters, redPoses.size());
      ASSERT_EQ(iters, greenPoses.size());
      ASSERT_EQ(iters, bluePoses.size());

      const double dt = 0.001; // 1 ms from the test world
      const double t = static_cast<double>(iters - 1) * dt; // N poses give N-1 integration intervals
      const double d = 0.5 * 9.8 * t * t;
      const auto &v = *target.poses;
      const double dz = v.front().Pos().Z() - v.back().Pos().Z();
      EXPECT_GT(dz, d);

      if (target.name != std::string("red_cube"))
      {
        EXPECT_TRUE(nearGround(redPoses.back().Pos().Z()));
      }
      if (target.name != std::string("green_cube"))
      {
        EXPECT_TRUE(nearGround(greenPoses.back().Pos().Z()));
      }
      if (target.name != std::string("blue_cube"))
      {
        EXPECT_TRUE(nearGround(bluePoses.back().Pos().Z()));
      }
    }

    // Let the detached target land then reset rod for next target
    {
      const std::size_t landIters{350};
      this->server->Run(true, landIters, false);
      EXPECT_NEAR(target.poses->back().Pos().Z(), 0.05, 1e-2);
    }

    setRodPose(target.x, 0.14);
    this->server->Run(true, 5, false);
  }
}
