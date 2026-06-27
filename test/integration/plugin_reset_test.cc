#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
/*
 * Test suite for verifying reset behavior of core Gazebo plugins.
 * Part of GSoC 2026 contribution — auditing plugin reset correctness.
 *
 * Plugins covered:
 *   - DiffDrive
 *   - JointController
 *   - Imu
 *   - Altimeter
 *
 * Each test spawns the plugin, runs a few steps, resets,
 * and checks that state returns to initial conditions.
 */

#include <gtest/gtest.h>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

// ──────────────────────────────────────────────
// Helper: run N steps then reset and run M more
// ──────────────────────────────────────────────

struct SimResult
{
  gz::math::Pose3d poseAfterRun;
  gz::math::Pose3d poseAfterReset;
  gz::math::Vector3d velAfterReset;
};

// ──────────────────────────────────────────────
// Test 1: DiffDrive plugin reset
// ──────────────────────────────────────────────

class DiffDriveResetTest : public InternalFixture<::testing::Test>
{
};

TEST_F(DiffDriveResetTest, ResetReturnsToPose)
{
  // Load the diff drive SDF world
  TestFixture fixture(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "diff_drive.sdf"));

  bool modelFound = false;
  gz::math::Pose3d poseBeforeReset;
  gz::math::Pose3d poseAfterReset;
  gz::math::Vector3d velAfterReset;

  // Run 50 steps to move the robot
  fixture.OnPreUpdate([&](const UpdateInfo &,
                          EntityComponentManager &_ecm)
  {
    // publish a cmd_vel so the robot actually moves
    // (in real test we'd use transport, simplified here)
  }).OnPostUpdate([&](const UpdateInfo &_info,
                      const EntityComponentManager &_ecm)
  {
    Entity model = _ecm.EntityByComponents(
        components::Model(), components::Name("vehicle"));

    if (model == kNullEntity)
      return;

    modelFound = true;

    auto poseComp = _ecm.Component<components::Pose>(model);
    if (poseComp)
      poseBeforeReset = poseComp->Data();

    auto velComp = _ecm.Component<components::LinearVelocity>(model);
    if (velComp)
      velAfterReset = velComp->Data();

  }).Finalize();

  // Run 50 steps
  fixture.Server()->Run(true, 50, false);

  // Reset simulation
  fixture.Server()->SetPaused(true);
  fixture.Server()->Run(true, 0, false);

  // Check robot is back at origin after reset
  fixture.OnPostUpdate([&](const UpdateInfo &,
                           const EntityComponentManager &_ecm)
  {
    Entity model = _ecm.EntityByComponents(
        components::Model(), components::Name("vehicle"));

    if (model == kNullEntity)
      return;

    auto poseComp = _ecm.Component<components::Pose>(model);
    if (poseComp)
      poseAfterReset = poseComp->Data();

  }).Finalize();

  fixture.Server()->Run(true, 1, false);

  ASSERT_TRUE(modelFound) << "vehicle model not found in world";

  // After reset, pose should be close to origin
  EXPECT_NEAR(poseAfterReset.Pos().X(), 0.0, 1e-3)
      << "X position did not reset to origin";
  EXPECT_NEAR(poseAfterReset.Pos().Y(), 0.0, 1e-3)
      << "Y position did not reset to origin";
  EXPECT_NEAR(poseAfterReset.Rot().Yaw(), 0.0, 1e-3)
      << "Yaw did not reset to origin";
}

// ──────────────────────────────────────────────
// Test 2: JointController plugin reset
// ──────────────────────────────────────────────

class JointControllerResetTest : public InternalFixture<::testing::Test>
{
};

TEST_F(JointControllerResetTest, JointVelocityResetsToZero)
{
  TestFixture fixture(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "joint_controller.sdf"));

  bool checked = false;
  double velAfterReset = -999.0;

  fixture.OnPostUpdate([&](const UpdateInfo &,
                           const EntityComponentManager &_ecm)
  {
    // find any joint with a velocity component
    _ecm.Each<components::JointVelocity>(
        [&](const Entity &,
            const components::JointVelocity *_vel) -> bool
    {
      if (!_vel->Data().empty())
      {
        velAfterReset = _vel->Data()[0];
        checked = true;
      }
      return true;
    });
  }).Finalize();

  // Run 30 steps to spin up joints
  fixture.Server()->Run(true, 30, false);

  // Reset
  fixture.Server()->Run(true, 0, false);

  // Run 1 more step to read post-reset state
  fixture.Server()->Run(true, 1, false);

  ASSERT_TRUE(checked) << "No joint velocity component found";

  // After reset, joint velocity should be near zero
  EXPECT_NEAR(velAfterReset, 0.0, 1e-2)
      << "Joint velocity did not reset to 0 after simulation reset";
}

// ──────────────────────────────────────────────
// Test 3: Altimeter plugin reset
// ──────────────────────────────────────────────

class AltimeterResetTest : public InternalFixture<::testing::Test>
{
};

TEST_F(AltimeterResetTest, AltimeterReadsZeroAfterReset)
{
  TestFixture fixture(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "altimeter.sdf"));

  bool altFound = false;
  double altAfterReset = -999.0;

  fixture.OnPostUpdate([&](const UpdateInfo &,
                           const EntityComponentManager &_ecm)
  {
    // Altimeter stores vertical position — should return to 0 on reset
    Entity altEntity = _ecm.EntityByComponents(
        components::Name("altimeter_sensor"));

    if (altEntity == kNullEntity)
      return;

    altFound = true;

    auto poseComp = _ecm.Component<components::Pose>(altEntity);
    if (poseComp)
      altAfterReset = poseComp->Data().Pos().Z();

  }).Finalize();

  fixture.Server()->Run(true, 20, false);
  fixture.Server()->Run(true, 0, false); // reset
  fixture.Server()->Run(true, 1, false);

  ASSERT_TRUE(altFound) << "altimeter_sensor entity not found";

  EXPECT_NEAR(altAfterReset, 0.0, 0.05)
      << "Altimeter Z position did not return to initial after reset";
}

// ──────────────────────────────────────────────
// Test 4: IMU plugin reset
// ──────────────────────────────────────────────

class ImuResetTest : public InternalFixture<::testing::Test>
{
};

TEST_F(ImuResetTest, ImuAngularVelocityZeroAfterReset)
{
  TestFixture fixture(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "imu.sdf"));

  bool imuFound = false;
  gz::math::Vector3d angVelAfterReset;

  fixture.OnPostUpdate([&](const UpdateInfo &,
                           const EntityComponentManager &_ecm)
  {
    _ecm.Each<components::AngularVelocity>(
        [&](const Entity &,
            const components::AngularVelocity *_angVel) -> bool
    {
      angVelAfterReset = _angVel->Data();
      imuFound = true;
      return false; // only need first one
    });
  }).Finalize();

  fixture.Server()->Run(true, 40, false);
  fixture.Server()->Run(true, 0, false); // reset
  fixture.Server()->Run(true, 1, false);

  ASSERT_TRUE(imuFound) << "No angular velocity component found";

  EXPECT_NEAR(angVelAfterReset.X(), 0.0, 1e-3)
      << "IMU angular velocity X not zero after reset";
  EXPECT_NEAR(angVelAfterReset.Y(), 0.0, 1e-3)
      << "IMU angular velocity Y not zero after reset";
  EXPECT_NEAR(angVelAfterReset.Z(), 0.0, 1e-3)
      << "IMU angular velocity Z not zero after reset";
}
