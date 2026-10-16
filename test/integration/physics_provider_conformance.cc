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

/// \file
/// Conformance test for physics providers.
///
/// A physics provider is a system that simulates the dynamics of models and
/// writes the results to the ECM, for example the Physics system with one of
/// the gz-physics engines. Other systems read what a provider writes. For
/// example, DiffDrive reads wheel joint positions, and sensors read link
/// velocities and accelerations. The checks below describe the components
/// that every provider has to read and write, so that those systems work
/// unchanged with any provider.
///
/// To add a provider, add an entry to Providers(). It needs a name, the SDF of
/// its world plugins, and the checks that the provider does not support yet,
/// each with the reason. An unsupported check is reported as skipped, so the
/// gap stays visible.
///
/// To add a check, add a value to Check, add a test below that starts
/// with SKIP_IF_UNSUPPORTED, and keep the check independent of the provider:
/// use only components and public system plugins, not engine specific APIs.

#include <gtest/gtest.h>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/physics/InstallationDirectories.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityReset.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Imu.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityReset.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"

#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

namespace
{
/// \brief The behaviors that a provider has to implement. Each value is one
/// test below.
enum class Check
{
  /// \brief DiffDrive moves a vehicle, wheel joint positions advance and the
  /// odometry that DiffDrive computes from them matches the ground truth.
  DiffDriveOdometry,

  /// \brief Model velocity commands are in the model frame, and link
  /// velocities are written in both the world and the body frame.
  VelocityControlFrames,

  /// \brief WorldPoseCmd teleports a model and is removed one step later.
  WorldPoseCmd,

  /// \brief JointPositionReset and JointVelocityReset apply before the next
  /// step and are removed.
  JointResets,

  /// \brief WorldLinearVelocityReset and WorldAngularVelocityReset apply
  /// before the next step and are removed.
  LinkVelocityResets,

  /// \brief ContactSensorData is filled on contact and cleared afterwards.
  Contacts,

  /// \brief An IMU at rest reads gravity, and no rotation.
  ImuAtRest
};

/// \brief A physics provider to test.
struct PhysicsProvider
{
  /// \brief Name of the provider. It names the test instance, so use only
  /// letters, digits and underscores.
  std::string name;

  /// \brief SDF of the world plugins that make up the provider. The worlds
  /// disable the server's default plugins, which include the Physics system,
  /// so that only these plugins simulate the world.
  std::string plugins;

  /// \brief True if the provider can run in this environment, for example
  /// because its physics engine is installed. The test is skipped otherwise.
  std::function<bool()> available;

  /// \brief Checks that the provider does not support yet, and why.
  std::map<Check, std::string> unsupported;

  /// \brief Maximum position error of the odometry, in meters, and heading
  /// error, in radians, after DiffDriveOdometry. The error comes from wheel
  /// slip, so it depends on the provider's contact model.
  double odometryPositionTolerance{0.03};

  /// \brief See odometryPositionTolerance.
  double odometryHeadingTolerance{0.03};
};

/// \brief A provider that is the Physics system with a gz-physics engine.
/// \param[in] _name Name of the test instance.
/// \param[in] _engine Name of the engine plugin library.
PhysicsProvider PhysicsSystemProvider(const std::string &_name,
    const std::string &_engine)
{
  PhysicsProvider provider;
  provider.name = _name;
  provider.plugins = R"(
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
      <engine><filename>)" + _engine + R"(</filename></engine>
    </plugin>)";
  provider.available = [_engine]()
  {
    common::SystemPaths systemPaths;
    systemPaths.SetPluginPathEnv("GZ_SIM_PHYSICS_ENGINE_PATH");
    systemPaths.AddPluginPaths(gz::physics::getEngineInstallDir());
    return !systemPaths.FindSharedLibrary(_engine).empty();
  };
  return provider;
}

/// \brief All the providers to test.
std::vector<PhysicsProvider> Providers()
{
  std::vector<PhysicsProvider> providers;
  providers.push_back(
      PhysicsSystemProvider("dartsim", "gz-physics-dartsim-plugin"));
  providers.push_back(PhysicsSystemProvider("bullet_featherstone",
      "gz-physics-bullet-featherstone-plugin"));
  return providers;
}

/// \brief Step size of the test worlds, in seconds.
constexpr double kStepSize{0.001};

/// \brief SDF of a world.
/// \param[in] _provider Provider that simulates the world.
/// \param[in] _gravity Gravity vector.
/// \param[in] _plugins Plugins of the world, other than the provider.
/// \param[in] _models Models of the world.
std::string WorldSdf(const PhysicsProvider &_provider,
    const std::string &_gravity, const std::string &_plugins,
    const std::string &_models)
{
  return std::string(R"(<?xml version="1.0"?>
<sdf version="1.9">
  <world name="conformance">
    <physics name="1ms" type="ode">
      <max_step_size>)") + std::to_string(kStepSize) + R"(</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <gz:policies>
      <include_server_config_plugins>false</include_server_config_plugins>
    </gz:policies>
    <gravity>)" + _gravity + "</gravity>" + _provider.plugins + _plugins +
    _models +
    "</world></sdf>";
}

/// \brief A ground plane.
const char *kGround = R"(
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <surface>
            <friction><torsional><coefficient>0</coefficient></torsional>
            </friction>
          </surface>
        </collision>
      </link>
    </model>)";

/// \brief A box link with a unit mass, which is a cube of 1 m.
/// \param[in] _name Name of the link.
/// \param[in] _pose Pose of the link.
std::string BoxLink(const std::string &_name, const std::string &_pose)
{
  return "<link name='" + _name + "'><pose>" + _pose + R"(</pose>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.1667</ixx><iyy>0.1667</iyy><izz>0.1667</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
      </link>)";
}

/// \brief Prefix of the transport topics that a test of a provider uses. Test
/// processes share their transport partition, so topics that carry a default
/// name, such as /model/vehicle/cmd_vel, collide with other tests.
/// \param[in] _provider Provider under test.
std::string TopicPrefix(const PhysicsProvider &_provider)
{
  return "/conformance/" + _provider.name;
}

/// \brief A two wheel vehicle with a caster, driven by DiffDrive. Its
/// geometry and parameters are those of test/worlds/diff_drive.sdf. The
/// wheels, the caster and the ground have no torsional friction. Without
/// that, bullet-featherstone's default torsional friction keeps the vehicle
/// from turning (gazebosim/gz-physics#697).
/// \param[in] _provider Provider under test.
std::string DiffDriveVehicle(const PhysicsProvider &_provider)
{
  const std::string prefix = TopicPrefix(_provider);
  return R"(
    <model name="vehicle">
      <pose>0 0 0.325 0 0 0</pose>
      <link name="chassis">
        <pose>-0.151427 0 0.175 0 0 0</pose>
        <inertial>
          <mass>1.14395</mass>
          <inertia>
            <ixx>0.126164</ixx><iyy>0.416519</iyy><izz>0.481014</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>2.01142 1 0.568726</size></box></geometry>
        </collision>
      </link>
      <link name="left_wheel">
        <pose>0.554283 0.625029 -0.025 -1.5707 0 0</pose>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.145833</ixx><iyy>0.145833</iyy><izz>0.125</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>0.3</radius></sphere></geometry>
          <surface>
            <friction>
              <ode><mu>1</mu><mu2>1</mu2></ode>
              <torsional><coefficient>0</coefficient></torsional>
            </friction>
          </surface>
        </collision>
      </link>
      <link name="right_wheel">
        <pose>0.554282 -0.625029 -0.025 -1.5707 0 0</pose>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.145833</ixx><iyy>0.145833</iyy><izz>0.125</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>0.3</radius></sphere></geometry>
          <surface>
            <friction>
              <ode><mu>1</mu><mu2>1</mu2></ode>
              <torsional><coefficient>0</coefficient></torsional>
            </friction>
          </surface>
        </collision>
      </link>
      <link name="caster">
        <pose>-0.957138 0 -0.125 0 0 0</pose>
        <inertial>
          <mass>1</mass>
          <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>0.2</radius></sphere></geometry>
          <surface>
            <friction>
              <ode><mu>0</mu><mu2>0</mu2></ode>
              <torsional><coefficient>0</coefficient></torsional>
            </friction>
          </surface>
        </collision>
      </link>
      <joint name="left_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>left_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
          </limit>
        </axis>
      </joint>
      <joint name="right_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
          </limit>
        </axis>
      </joint>
      <joint name="caster_wheel" type="ball">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>
      <plugin filename="gz-sim-diff-drive-system"
              name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>1.25</wheel_separation>
        <wheel_radius>0.3</wheel_radius>
        <topic>)" + prefix + R"(/cmd_vel</topic>
        <odom_topic>)" + prefix + R"(/odometry</odom_topic>
        <tf_topic>)" + prefix + R"(/tf</tf_topic>
        <enable_topic>)" + prefix + R"(/enable</enable_topic>
        <max_linear_acceleration>1</max_linear_acceleration>
        <min_linear_acceleration>-1</min_linear_acceleration>
        <max_angular_acceleration>2</max_angular_acceleration>
        <min_angular_acceleration>-2</min_angular_acceleration>
        <max_linear_velocity>0.5</max_linear_velocity>
        <min_linear_velocity>-0.5</min_linear_velocity>
        <max_angular_velocity>1</max_angular_velocity>
        <min_angular_velocity>-1</min_angular_velocity>
      </plugin>
    </model>)";
}

/// \brief Find a model by name.
/// \param[in] _ecm Entity component manager.
/// \param[in] _name Name of the model.
Model ModelByName(const EntityComponentManager &_ecm, const std::string &_name)
{
  return Model(_ecm.EntityByComponents(components::Model(),
      components::Name(_name)));
}

}  // namespace

/// \brief Skip the test if the provider can't run here, or if it doesn't
/// support the check yet. Use it first in each test.
#define SKIP_IF_UNSUPPORTED(_check) \
  if (auto reason = this->SkipReason(_check); !reason.empty()) \
  { \
    GTEST_SKIP() << reason; \
  }

/// \brief Fixture that runs each check on each provider.
class PhysicsProviderConformance
  : public InternalFixture<::testing::TestWithParam<PhysicsProvider>>
{
  /// \brief Explain why a check can't run, if it can't.
  /// \param[in] _check Check that the test implements.
  /// \return Reason to skip the test, or an empty string if it can run.
  protected: std::string SkipReason(Check _check) const
  {
    const auto &provider = this->GetParam();
    if (!provider.available())
    {
      return "Provider [" + provider.name +
          "] is not available in this environment";
    }

    auto it = provider.unsupported.find(_check);
    if (it != provider.unsupported.end())
    {
      return "Provider [" + provider.name +
          "] does not support this check yet: " + it->second;
    }
    return "";
  }

  /// \brief Create a test fixture that simulates a world with the provider.
  /// \param[in] _gravity Gravity vector.
  /// \param[in] _plugins Plugins of the world, other than the provider.
  /// \param[in] _models Models of the world.
  protected: std::unique_ptr<TestFixture> MakeFixture(
      const std::string &_gravity, const std::string &_plugins,
      const std::string &_models)
  {
    ServerConfig config;
    config.SetSdfString(WorldSdf(this->GetParam(), _gravity, _plugins,
        _models));
    return std::make_unique<TestFixture>(config);
  }
};

/////////////////////////////////////////////////
TEST_P(PhysicsProviderConformance,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(DiffDriveOdometry))
{
  SKIP_IF_UNSUPPORTED(Check::DiffDriveOdometry);

  const auto &provider = this->GetParam();
  auto fixture = this->MakeFixture("0 0 -9.8", "",
      std::string(kGround) + DiffDriveVehicle(provider));

  transport::Node node;
  auto cmdPub = node.Advertise<msgs::Twist>(
      TopicPrefix(provider) + "/cmd_vel");

  // Odometry messages: simulation time and pose.
  std::mutex odomMutex;
  std::vector<std::pair<double, math::Pose3d>> odom;
  std::function<void(const msgs::Odometry &)> odomCb =
      [&](const msgs::Odometry &_msg)
      {
        const double stamp = static_cast<double>(_msg.header().stamp().sec()) +
            static_cast<double>(_msg.header().stamp().nsec()) * 1e-9;
        std::lock_guard<std::mutex> lock(odomMutex);
        odom.emplace_back(stamp, msgs::Convert(_msg.pose()));
      };
  node.Subscribe(TopicPrefix(provider) + "/odometry", odomCb);

  // Ground truth: model pose at each step, by simulation time.
  std::map<int64_t, math::Pose3d> truth;
  math::Vector2d wheelPositions;
  bool wheelPositionsFound{false};

  msgs::Twist cmd;
  msgs::Set(cmd.mutable_linear(), math::Vector3d(0.5, 0, 0));
  msgs::Set(cmd.mutable_angular(), math::Vector3d(0, 0, 0.2));

  fixture->OnPreUpdate([&](const UpdateInfo &_info, EntityComponentManager &)
      {
        // Publish repeatedly, because the subscriber may not be connected yet.
        if (_info.iterations % 10 == 1)
          cmdPub.Publish(cmd);
      });
  fixture->OnPostUpdate([&](const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
      {
        auto vehicle = ModelByName(_ecm, "vehicle");
        ASSERT_TRUE(vehicle.Valid(_ecm));
        auto pose = _ecm.Component<components::Pose>(vehicle.Entity());
        ASSERT_NE(nullptr, pose);
        truth[static_cast<int64_t>(std::lround(
            std::chrono::duration<double>(_info.simTime).count() /
            kStepSize))] = pose->Data();

        // DiffDrive creates the joint position components. The provider has to
        // fill them in, because DiffDrive computes the odometry from them.
        auto left = _ecm.Component<components::JointPosition>(
            vehicle.JointByName(_ecm, "left_wheel_joint"));
        auto right = _ecm.Component<components::JointPosition>(
            vehicle.JointByName(_ecm, "right_wheel_joint"));
        if (left && right && !left->Data().empty() && !right->Data().empty())
        {
          wheelPositions.Set(left->Data()[0], right->Data()[0]);
          wheelPositionsFound = true;
        }
      });
  fixture->Finalize();

  fixture->Server()->Run(true, 3000, false);

  // DiffDrive publishes odometry at 50 Hz by default, which is about 150
  // messages in 3 s. Messages arrive asynchronously, and the test only needs
  // enough of them to follow the trajectory.
  constexpr size_t kMinOdometryMessages{100};
  for (int i = 0; i < 50; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(odomMutex);
      if (odom.size() >= kMinOdometryMessages)
        break;
    }
    std::this_thread::sleep_for(100ms);
  }

  std::lock_guard<std::mutex> lock(odomMutex);
  ASSERT_GE(odom.size(), kMinOdometryMessages);
  ASSERT_FALSE(truth.empty());

  // The vehicle drove forward and turned left.
  const auto &start = truth.begin()->second;
  const auto &end = truth.rbegin()->second;
  EXPECT_GT(start.Pos().Distance(end.Pos()), 0.8);
  EXPECT_GT(end.Rot().Yaw() - start.Rot().Yaw(), 0.3);

  // The wheels turned, forward.
  ASSERT_TRUE(wheelPositionsFound);
  EXPECT_GT(wheelPositions.X(), 1.0);
  EXPECT_GT(wheelPositions.Y(), 1.0);
  EXPECT_GT(wheelPositions.Y(), wheelPositions.X());

  // The odometry frame is at the point halfway between the wheels, at the
  // pose of the vehicle when the simulation started. Express the ground truth
  // in that frame.
  const math::Pose3d wheelsInModel(0.554283, 0, 0, 0, 0, 0);
  const math::Pose3d modelAtStart(0, 0, 0.325, 0, 0, 0);
  const auto odomFromModel =
      wheelsInModel.Inverse() * modelAtStart.Inverse();

  // Track the largest errors, and report each one once with its time stamp.
  size_t compared{0};
  double worstPosition{0};
  double worstPositionStamp{0};
  double worstHeading{0};
  double worstHeadingStamp{0};
  for (const auto &[stamp, odomPose] : odom)
  {
    auto it = truth.find(
        static_cast<int64_t>(std::lround(stamp / kStepSize)));
    if (it == truth.end())
      continue;

    const math::Pose3d expected = odomFromModel * it->second * wheelsInModel;
    const double position = std::max(
        std::abs(expected.Pos().X() - odomPose.Pos().X()),
        std::abs(expected.Pos().Y() - odomPose.Pos().Y()));
    const double heading =
        std::abs(math::Angle(expected.Rot().Yaw() - odomPose.Rot().Yaw())
            .Normalized().Radian());
    if (position > worstPosition)
    {
      worstPosition = position;
      worstPositionStamp = stamp;
    }
    if (heading > worstHeading)
    {
      worstHeading = heading;
      worstHeadingStamp = stamp;
    }
    ++compared;
  }
  EXPECT_LE(worstPosition, provider.odometryPositionTolerance)
      << "worst position error at t = " << worstPositionStamp;
  EXPECT_LE(worstHeading, provider.odometryHeadingTolerance)
      << "worst heading error at t = " << worstHeadingStamp;
  EXPECT_GE(compared, kMinOdometryMessages);

  // The last odometry pose is far from the origin, so the comparison above
  // can't pass with odometry that stays at zero.
  EXPECT_GT(odom.back().second.Pos().Length(), 0.8);
}

/////////////////////////////////////////////////
TEST_P(PhysicsProviderConformance,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(VelocityControlFrames))
{
  SKIP_IF_UNSUPPORTED(Check::VelocityControlFrames);

  // A model that is yawed by 90 degrees, in a world without gravity. The
  // VelocityControl command is in the model frame, so the model has to move
  // along its own x axis, which is the world y axis.
  const std::string model = R"(
    <model name="robot">
      <pose>0 0 1 0 0 1.5707963267948966</pose>)" +
      BoxLink("link", "0 0 0 0 0 0") + R"(
      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
        <initial_linear>1 0 0</initial_linear>
      </plugin>
    </model>)";

  auto fixture = this->MakeFixture("0 0 0", "", model);

  math::Pose3d startPose;
  math::Pose3d endPose;
  math::Vector3d worldVel;
  math::Vector3d bodyVel;
  math::Pose3d linkWorldPose;
  bool linkDataFound{false};

  fixture->OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        // Providers fill link frame data only when someone asks for it.
        auto link = ModelByName(_ecm, "robot").LinkByName(_ecm, "link");
        ASSERT_NE(kNullEntity, link);
        if (!_ecm.Component<components::WorldPose>(link))
          _ecm.CreateComponent(link, components::WorldPose());
        if (!_ecm.Component<components::WorldLinearVelocity>(link))
          _ecm.CreateComponent(link, components::WorldLinearVelocity());
        if (!_ecm.Component<components::LinearVelocity>(link))
          _ecm.CreateComponent(link, components::LinearVelocity());
      });
  fixture->OnPostUpdate([&](const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
      {
        auto robot = ModelByName(_ecm, "robot");
        auto pose = _ecm.Component<components::Pose>(robot.Entity());
        ASSERT_NE(nullptr, pose);
        if (_info.iterations == 1)
          startPose = pose->Data();
        endPose = pose->Data();

        auto link = robot.LinkByName(_ecm, "link");
        auto w = _ecm.Component<components::WorldLinearVelocity>(link);
        auto b = _ecm.Component<components::LinearVelocity>(link);
        auto p = _ecm.Component<components::WorldPose>(link);
        if (w && b && p)
        {
          worldVel = w->Data();
          bodyVel = b->Data();
          linkWorldPose = p->Data();
          linkDataFound = true;
        }
      });
  fixture->Finalize();

  fixture->Server()->Run(true, 500, false);

  ASSERT_TRUE(linkDataFound);

  // Moved 1 m/s for 0.5 s along the world y axis.
  const auto travelled = endPose.Pos() - startPose.Pos();
  EXPECT_NEAR(0.0, travelled.X(), 0.02);
  EXPECT_NEAR(0.5, travelled.Y(), 0.02);
  EXPECT_NEAR(0.0, travelled.Z(), 0.02);
  EXPECT_NEAR(1.5707963, endPose.Rot().Yaw(), 0.01);

  // The link velocity in the world frame is along y, and in the body frame
  // along x.
  EXPECT_NEAR(0.0, worldVel.X(), 0.01);
  EXPECT_NEAR(1.0, worldVel.Y(), 0.01);
  EXPECT_NEAR(0.0, worldVel.Z(), 0.01);
  EXPECT_NEAR(1.0, bodyVel.X(), 0.01);
  EXPECT_NEAR(0.0, bodyVel.Y(), 0.01);
  EXPECT_NEAR(0.0, bodyVel.Z(), 0.01);

  // The world pose of the link follows the model.
  EXPECT_NEAR(endPose.Pos().X(), linkWorldPose.Pos().X(), 1e-3);
  EXPECT_NEAR(endPose.Pos().Y(), linkWorldPose.Pos().Y(), 1e-3);
  EXPECT_NEAR(endPose.Rot().Yaw(), linkWorldPose.Rot().Yaw(), 1e-3);
}

/////////////////////////////////////////////////
TEST_P(PhysicsProviderConformance,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(WorldPoseCmd))
{
  SKIP_IF_UNSUPPORTED(Check::WorldPoseCmd);

  const std::string model = "<model name='box'><pose>0 0 1 0 0 0</pose>" +
      BoxLink("link", "0 0 0 0 0 0") + "</model>";
  auto fixture = this->MakeFixture("0 0 0", "", model);

  const math::Pose3d target(3, 2, 4, 0.1, 0.2, 0.5);

  // Iteration at which the command is written.
  constexpr uint64_t kCmdIteration{5};

  bool poseReached{false};
  bool cmdPresentInSameStep{true};
  bool cmdRemovedAfter{false};
  fixture->OnPreUpdate([&](const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
      {
        if (_info.iterations == kCmdIteration)
        {
          auto box = ModelByName(_ecm, "box");
          _ecm.CreateComponent(box.Entity(), components::WorldPoseCmd(target));
        }
      });
  fixture->OnPostUpdate([&](const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
      {
        auto box = ModelByName(_ecm, "box");
        auto pose = _ecm.Component<components::Pose>(box.Entity());
        ASSERT_NE(nullptr, pose);
        auto cmd = _ecm.Component<components::WorldPoseCmd>(box.Entity());

        if (_info.iterations < kCmdIteration)
        {
          EXPECT_NEAR(1.0, pose->Data().Z(), 1e-3);
          EXPECT_EQ(nullptr, cmd);
        }
        else if (_info.iterations == kCmdIteration)
        {
          // The pose changes in the same step, and other systems still see the
          // command.
          poseReached = pose->Data().Pos().Equal(target.Pos(), 1e-3) &&
              pose->Data().Rot().Equal(target.Rot(), 1e-3);
          cmdPresentInSameStep = cmd != nullptr;
        }
        else if (_info.iterations == kCmdIteration + 1)
        {
          cmdRemovedAfter = cmd == nullptr;
        }
      });
  fixture->Finalize();

  fixture->Server()->Run(true, 10, false);

  EXPECT_TRUE(poseReached);
  EXPECT_TRUE(cmdPresentInSameStep);
  EXPECT_TRUE(cmdRemovedAfter);
}

/////////////////////////////////////////////////
TEST_P(PhysicsProviderConformance,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(JointResets))
{
  SKIP_IF_UNSUPPORTED(Check::JointResets);

  // An arm on a revolute joint about the vertical axis, so that nothing
  // pushes on it. Gravity is off anyway.
  const std::string model = R"(
    <model name="arm">
      <link name="base">
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy>
          <izz>1</izz></inertia></inertial>
      </link>)" + BoxLink("link", "0 0 1 0 0 0") + R"(
      <joint name="fixed_to_world" type="fixed">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint name="joint" type="revolute">
        <parent>base</parent>
        <child>link</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
    </model>)";
  auto fixture = this->MakeFixture("0 0 0", "", model);

  constexpr double kPosition{0.42};
  constexpr double kVelocity{1.5};
  constexpr uint64_t kPositionIteration{3};
  constexpr uint64_t kVelocityIteration{20};

  std::map<uint64_t, double> positions;
  std::map<uint64_t, double> velocities;
  std::map<uint64_t, bool> resetsPresent;

  auto joint = [](const EntityComponentManager &_ecm)
  {
    return ModelByName(_ecm, "arm").JointByName(_ecm, "joint");
  };

  fixture->OnPreUpdate([&](const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
      {
        auto entity = joint(_ecm);
        ASSERT_NE(kNullEntity, entity);
        if (_info.iterations == 1)
        {
          // The provider only writes joint state that someone asked for.
          _ecm.CreateComponent(entity, components::JointPosition());
          _ecm.CreateComponent(entity, components::JointVelocity());
        }
        else if (_info.iterations == kPositionIteration)
        {
          _ecm.CreateComponent(entity,
              components::JointPositionReset({kPosition}));
        }
        else if (_info.iterations == kVelocityIteration)
        {
          _ecm.CreateComponent(entity,
              components::JointVelocityReset({kVelocity}));
        }
      });
  fixture->OnPostUpdate([&](const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
      {
        auto entity = joint(_ecm);
        auto pos = _ecm.Component<components::JointPosition>(entity);
        auto vel = _ecm.Component<components::JointVelocity>(entity);
        if (pos && vel && !pos->Data().empty() && !vel->Data().empty())
        {
          positions[_info.iterations] = pos->Data()[0];
          velocities[_info.iterations] = vel->Data()[0];
        }
        resetsPresent[_info.iterations] =
            _ecm.Component<components::JointPositionReset>(entity) ||
            _ecm.Component<components::JointVelocityReset>(entity);
      });
  fixture->Finalize();

  fixture->Server()->Run(true, 40, false);

  ASSERT_EQ(40u, positions.size());

  // Nothing moves before the resets.
  EXPECT_NEAR(0.0, positions[2], 1e-6);
  EXPECT_NEAR(0.0, velocities[2], 1e-6);

  // The position is reset in the step in which it is requested, and it keeps
  // that value, with no velocity.
  EXPECT_NEAR(kPosition, positions[kPositionIteration], 1e-6);
  EXPECT_NEAR(kPosition, positions[kPositionIteration + 10], 1e-3);
  EXPECT_NEAR(0.0, velocities[kPositionIteration + 10], 1e-3);

  // The velocity is reset in the step in which it is requested, and then the
  // joint keeps turning.
  EXPECT_NEAR(kVelocity, velocities[kVelocityIteration], 1e-3);
  EXPECT_NEAR(kVelocity, velocities[kVelocityIteration + 10], 1e-2);
  EXPECT_NEAR(kPosition + kVelocity * 11 * kStepSize,
      positions[kVelocityIteration + 10], 0.005);

  // The reset components are consumed.
  for (const auto &[iteration, present] : resetsPresent)
    EXPECT_FALSE(present) << "iteration " << iteration;
}

/////////////////////////////////////////////////
TEST_P(PhysicsProviderConformance,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(LinkVelocityResets))
{
  SKIP_IF_UNSUPPORTED(Check::LinkVelocityResets);

  const std::string model = "<model name='box'><pose>0 0 1 0 0 0</pose>" +
      BoxLink("link", "0 0 0 0 0 0") + "</model>";
  auto fixture = this->MakeFixture("0 0 0", "", model);

  constexpr uint64_t kResetIteration{10};
  const math::Vector3d linear(0, 0, 2);
  const math::Vector3d angular(0, 0, 3);

  std::map<uint64_t, math::Pose3d> poses;
  std::map<uint64_t, bool> resetsPresent;

  auto link = [](const EntityComponentManager &_ecm)
  {
    return ModelByName(_ecm, "box").LinkByName(_ecm, "link");
  };

  fixture->OnPreUpdate([&](const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
      {
        if (_info.iterations == kResetIteration)
        {
          _ecm.CreateComponent(link(_ecm),
              components::WorldLinearVelocityReset(linear));
          _ecm.CreateComponent(link(_ecm),
              components::WorldAngularVelocityReset(angular));
        }
      });
  fixture->OnPostUpdate([&](const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
      {
        auto entity = link(_ecm);
        auto pose = _ecm.Component<components::Pose>(
            ModelByName(_ecm, "box").Entity());
        poses[_info.iterations] = pose->Data();
        resetsPresent[_info.iterations] =
            _ecm.Component<components::WorldLinearVelocityReset>(entity) ||
            _ecm.Component<components::WorldAngularVelocityReset>(entity);
      });
  fixture->Finalize();

  fixture->Server()->Run(true, 110, false);

  // At rest before the reset.
  EXPECT_NEAR(1.0, poses[kResetIteration - 1].Z(), 1e-6);
  EXPECT_NEAR(0.0, poses[kResetIteration - 1].Rot().Yaw(), 1e-6);

  // The velocity applies to the step in which it is written, so the pose in
  // that step has already moved by one step of the new velocity. If a
  // provider applied the reset one step late, this pose would differ by
  // 2 mm in position and 3 mrad in heading. The tolerance is a quarter of
  // that.
  EXPECT_NEAR(1.0 + linear.Z() * kStepSize, poses[kResetIteration].Z(),
      5e-4);
  EXPECT_NEAR(angular.Z() * kStepSize, poses[kResetIteration].Rot().Yaw(),
      7.5e-4);

  // After 100 more steps the box moved 100 steps * 1 ms * 2 m/s up and turned
  // 100 * 1 ms * 3 rad/s. The 1 cm tolerance is wide enough for different
  // integrators. At this distance it only shows that the velocity was
  // applied and kept, not when it started. The check above covers the timing.
  const double elapsed = 100 * kStepSize;
  EXPECT_NEAR(1.0 + linear.Z() * elapsed, poses[kResetIteration + 100].Z(),
      0.01);
  EXPECT_NEAR(angular.Z() * elapsed,
      poses[kResetIteration + 100].Rot().Yaw(), 0.01);

  for (const auto &[iteration, present] : resetsPresent)
    EXPECT_FALSE(present) << "iteration " << iteration;
}

/////////////////////////////////////////////////
TEST_P(PhysicsProviderConformance,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(Contacts))
{
  SKIP_IF_UNSUPPORTED(Check::Contacts);

  // A box that falls from 10 cm above the ground, and is moved up later.
  const std::string model = "<model name='box'><pose>0 0 0.6 0 0 0</pose>" +
      BoxLink("link", "0 0 0 0 0 0") + "</model>";
  auto fixture = this->MakeFixture("0 0 -9.8", "",
      std::string(kGround) + model);

  constexpr uint64_t kLiftIteration{700};

  // Number of contacts by iteration, and the entities in the last contact.
  std::map<uint64_t, int> contacts;
  std::set<Entity> contactEntities;
  Entity boxCollision{kNullEntity};
  Entity groundCollision{kNullEntity};

  fixture->OnPreUpdate([&](const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
      {
        auto box = ModelByName(_ecm, "box");
        boxCollision = Link(box.LinkByName(_ecm, "link")).CollisionByName(
            _ecm, "collision");
        groundCollision = Link(ModelByName(_ecm, "ground").LinkByName(
            _ecm, "link")).CollisionByName(_ecm, "collision");
        ASSERT_NE(kNullEntity, boxCollision);
        if (_info.iterations == 1)
        {
          // The provider only reports contacts of collisions that have the
          // component.
          _ecm.CreateComponent(boxCollision, components::ContactSensorData());
        }
        else if (_info.iterations == kLiftIteration)
        {
          _ecm.CreateComponent(box.Entity(),
              components::WorldPoseCmd(math::Pose3d(0, 0, 5, 0, 0, 0)));
        }
      });
  fixture->OnPostUpdate([&](const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
      {
        auto data = _ecm.Component<components::ContactSensorData>(boxCollision);
        ASSERT_NE(nullptr, data);
        contacts[_info.iterations] = data->Data().contact_size();
        if (data->Data().contact_size() > 0)
        {
          contactEntities.clear();
          for (const auto &contact : data->Data().contact())
          {
            contactEntities.insert(contact.collision1().id());
            contactEntities.insert(contact.collision2().id());
          }
        }
      });
  fixture->Finalize();

  fixture->Server()->Run(true, 720, false);

  // Falling: no contact. At rest on the ground: contact.
  EXPECT_EQ(0, contacts[10]);
  EXPECT_GT(contacts[kLiftIteration - 1], 0);

  // The contact is between the box and the ground.
  EXPECT_EQ(1u, contactEntities.count(boxCollision));
  EXPECT_EQ(1u, contactEntities.count(groundCollision));

  // The box was lifted off the ground, so the contacts clear. They don't linger
  // from the previous step.
  EXPECT_EQ(0, contacts[kLiftIteration + 5]);
  EXPECT_EQ(0, contacts[kLiftIteration + 15]);
}

/////////////////////////////////////////////////
TEST_P(PhysicsProviderConformance,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(ImuAtRest))
{
  SKIP_IF_UNSUPPORTED(Check::ImuAtRest);

  const std::string plugins = R"(
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>)";

  // A box on the ground with an IMU. The box is yawed, and the sensor is
  // tilted in the link frame, so gravity shows on all three sensor axes.
  const std::string topic = TopicPrefix(this->GetParam()) + "/imu";
  const std::string model = R"(
    <model name="box">
      <pose>0 0 0.5 0 0 0.3</pose>
      <link name="link">
        <inertial>
          <mass>1</mass>
          <inertia><ixx>0.1667</ixx><iyy>0.1667</iyy><izz>0.1667</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <sensor name="imu" type="imu">
          <pose>0 0 0 0.3 0.2 0</pose>
          <always_on>true</always_on>
          <update_rate>100</update_rate>
          <topic>)" + topic + R"(</topic>
        </sensor>
      </link>
    </model>)";
  auto fixture = this->MakeFixture("0 0 -9.8", plugins,
      std::string(kGround) + model);

  std::mutex imuMutex;
  msgs::IMU lastImu;
  size_t imuCount{0};
  transport::Node node;
  std::function<void(const msgs::IMU &)> imuCb =
      [&](const msgs::IMU &_msg)
      {
        std::lock_guard<std::mutex> lock(imuMutex);
        lastImu = _msg;
        ++imuCount;
      };
  node.Subscribe(topic, imuCb);

  // Kinematic acceleration of the sensor in the ECM, which is what the
  // IMU system gives the sensor.
  math::Vector3d kinematicAcceleration{1, 1, 1};
  math::Vector3d angularVelocity{1, 1, 1};
  bool sensorFound{false};
  math::Quaterniond boxRotation;
  fixture->OnPostUpdate([&](const UpdateInfo &,
                            const EntityComponentManager &_ecm)
      {
        boxRotation = _ecm.Component<components::Pose>(
            ModelByName(_ecm, "box").Entity())->Data().Rot();
        _ecm.Each<components::Imu, components::LinearAcceleration,
                  components::AngularVelocity>(
            [&](const Entity &, const components::Imu *,
                const components::LinearAcceleration *_acc,
                const components::AngularVelocity *_vel) -> bool
            {
              kinematicAcceleration = _acc->Data();
              angularVelocity = _vel->Data();
              sensorFound = true;
              return true;
            });
      });
  fixture->Finalize();

  // Let the box settle, then read the sensor.
  fixture->Server()->Run(true, 2000, false);

  for (int i = 0; i < 50; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(imuMutex);
      if (imuCount >= 150u)
        break;
    }
    std::this_thread::sleep_for(100ms);
  }

  ASSERT_TRUE(sensorFound);

  // At rest the sensor has no kinematic acceleration and does not rotate.
  EXPECT_NEAR(0.0, kinematicAcceleration.X(), 0.05);
  EXPECT_NEAR(0.0, kinematicAcceleration.Y(), 0.05);
  EXPECT_NEAR(0.0, kinematicAcceleration.Z(), 0.05);
  EXPECT_NEAR(0.0, angularVelocity.X(), 0.01);
  EXPECT_NEAR(0.0, angularVelocity.Y(), 0.01);
  EXPECT_NEAR(0.0, angularVelocity.Z(), 0.01);

  // The sensor message reads the reaction to gravity: 1 g up, in the sensor
  // frame. The sensor is rotated in the world by the rotation of the box and
  // the tilt of the sensor in the link.
  std::lock_guard<std::mutex> lock(imuMutex);
  ASSERT_GT(imuCount, 0u);
  const math::Quaterniond sensorRotation =
      boxRotation * math::Quaterniond(0.3, 0.2, 0);
  const auto expected = sensorRotation.RotateVectorReverse(
      math::Vector3d(0, 0, 9.8));
  EXPECT_GT(std::abs(expected.X()), 1.0);
  EXPECT_GT(std::abs(expected.Y()), 1.0);
  EXPECT_NEAR(expected.X(), lastImu.linear_acceleration().x(), 0.1);
  EXPECT_NEAR(expected.Y(), lastImu.linear_acceleration().y(), 0.1);
  EXPECT_NEAR(expected.Z(), lastImu.linear_acceleration().z(), 0.1);
  EXPECT_NEAR(0.0, lastImu.angular_velocity().x(), 0.01);
  EXPECT_NEAR(0.0, lastImu.angular_velocity().y(), 0.01);
  EXPECT_NEAR(0.0, lastImu.angular_velocity().z(), 0.01);
}

INSTANTIATE_TEST_SUITE_P(Providers, PhysicsProviderConformance,
    ::testing::ValuesIn(Providers()),
    [](const ::testing::TestParamInfo<PhysicsProvider> &_info)
    {
      return _info.param.name;
    });
