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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/components/Pose.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief World step size.
constexpr double kDt{0.001};

/// \brief Wheel radius and half the track of the test robot.
constexpr double kRadius{0.1};
constexpr double kHalfTrack{0.2};

/// \brief Distance of the axle from the model origin along the model x axis.
constexpr double kAxleX{0.05};

/// \brief Name of the SimplifiedPhysicsModel plugin.
const char *kModelPlugin = "gz::sim::systems::SimplifiedPhysicsModel";

/// \brief Settings of one test robot.
struct RobotOptions
{
  /// \brief Model name. DiffDrive topics derive from it.
  std::string name;

  /// \brief Pose in the world, as SDF text.
  std::string pose{"0 0 0.1 0 0 0"};

  /// \brief True to run the robot on planar_kinematic. False leaves it on
  /// the default physics system.
  bool planar{false};

  /// \brief More parameters of SimplifiedPhysicsModel, as SDF text.
  std::string planarParams;

  /// \brief Rate of the odometry messages.
  double odomHz{1000};

  /// \brief Other plugins of the model, as SDF text.
  std::string plugins;

  /// \brief Include the DiffDrive plugin.
  bool diffDrive{true};

  /// \brief Put a massless footprint link first, so that it is the
  /// canonical link, and weld the chassis to it.
  bool footprint{false};

  /// \brief Wheels are spheres. Otherwise they are 4 cm wide cylinders. A
  /// dartsim cylinder touches the floor at its edges, which shortens the
  /// track by the width of the wheel, a sphere touches it in the middle.
  bool sphereWheels{false};
};

/// \brief SDF text of a differential drive robot. One 2 kg chassis, two
/// cylinder wheels (radius 0.1, track 0.4) and a ball caster. The wheel axle
/// is kAxleX ahead of the model origin, which is the center of the chassis.
std::string RobotXml(const RobotOptions &_o)
{
  const std::string wheelShape = _o.sphereWheels ?
      "<sphere><radius>0.1</radius></sphere>" :
      "<cylinder><radius>0.1</radius><length>0.04</length></cylinder>";
  std::ostringstream out;
  out << R"(
    <model name=")" << _o.name << R"(">
      <pose>)" << _o.pose << R"(</pose>)";
  if (_o.footprint)
  {
    out << R"(
      <link name="base_footprint">
        <inertial>
          <mass>0.001</mass>
          <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia>
        </inertial>
      </link>)";
  }
  out << R"(
      <link name="chassis">
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.0073</ixx><iyy>0.0272</iyy><izz>0.0333</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.4 0.2 0.06</size></box></geometry>
        </collision>
      </link>
      <link name="left_wheel">
        <pose relative_to="chassis">)" << kAxleX << " " << kHalfTrack <<
      R"( 0 -1.570796327 0 0</pose>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.00132</ixx><iyy>0.00132</iyy><izz>0.0025</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>)" << wheelShape << R"(</geometry>
        </collision>
      </link>
      <link name="right_wheel">
        <pose relative_to="chassis">)" << kAxleX << " " << -kHalfTrack <<
      R"( 0 -1.570796327 0 0</pose>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.00132</ixx><iyy>0.00132</iyy><izz>0.0025</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>)" << wheelShape << R"(</geometry>
        </collision>
      </link>
      <link name="caster">
        <pose relative_to="chassis">-0.2 0 -0.07 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>
        </collision>
      </link>
      )" << (_o.footprint ? R"(<joint name="base_joint" type="fixed">
        <parent>base_footprint</parent>
        <child>chassis</child>
      </joint>)" : "") << R"(
      <joint name="left_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>left_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="right_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>right_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="caster_joint" type="ball">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>)";

  if (_o.diffDrive)
  {
    out << R"(
      <plugin filename="gz-sim-diff-drive-system"
              name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>)" << 2 * kHalfTrack << R"(</wheel_separation>
        <wheel_radius>)" << kRadius << R"(</wheel_radius>
        <odom_publish_frequency>)" << _o.odomHz <<
        R"(</odom_publish_frequency>
        <max_linear_acceleration>2</max_linear_acceleration>
        <min_linear_acceleration>-2</min_linear_acceleration>
        <max_angular_acceleration>4</max_angular_acceleration>
        <min_angular_acceleration>-4</min_angular_acceleration>
      </plugin>)";
  }

  if (_o.planar)
  {
    out << R"(
      <plugin filename="gz-sim-simplified-physics-system"
              name=")" << kModelPlugin << R"(">
        <mode>planar_kinematic</mode>)" << _o.planarParams << R"(
      </plugin>)";
  }
  out << _o.plugins << "\n    </model>\n";
  return out.str();
}

/// \brief SDF text of a world with a ground and the models.
std::string WorldXml(const std::string &_models)
{
  return R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="wheels_test">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-simplified-physics-system"
            name="gz::sim::systems::SimplifiedPhysics"/>
    <model name="ground">
      <static>true</static>
      <pose>0 0 -0.1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>400 400 0.2</size></box></geometry>
        </collision>
      </link>
    </model>)" + _models + R"(
  </world>
</sdf>
)";
}

/// \brief Odometry messages of one robot, by simulation time.
class OdometryLog
{
  /// \brief Subscribe to the odometry topic of a robot.
  public: void Subscribe(const std::string &_model)
  {
    ASSERT_TRUE(this->node.Subscribe("/model/" + _model + "/odometry",
        &OdometryLog::OnMessage, this));
  }

  /// \brief What one message says.
  public: struct Sample
  {
    /// \brief Position in the odometry frame.
    math::Vector3d position;

    /// \brief Heading in the odometry frame.
    double yaw{0.0};
  };

  /// \brief Store a message.
  private: void OnMessage(const msgs::Odometry &_msg)
  {
    const auto stamp = static_cast<std::int64_t>(
        _msg.header().stamp().sec()) * 1000000000 +
        _msg.header().stamp().nsec();
    Sample sample;
    sample.position.Set(_msg.pose().position().x(),
        _msg.pose().position().y(), 0.0);
    sample.yaw = math::Quaterniond(_msg.pose().orientation().w(),
        _msg.pose().orientation().x(), _msg.pose().orientation().y(),
        _msg.pose().orientation().z()).Yaw();
    std::lock_guard<std::mutex> lock(this->mutex);
    this->samples[stamp] = sample;
  }

  /// \brief Copy of the log.
  public: std::map<std::int64_t, Sample> Samples()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->samples;
  }

  private: transport::Node node;
  private: std::mutex mutex;
  private: std::map<std::int64_t, Sample> samples;
};

/// \brief A robot's pose after every step, by simulation time, and its
/// first pose.
struct GroundTruth
{
  math::Pose3d initial;
  std::map<std::int64_t, math::Pose3d> poses;
  std::vector<double> leftPosition;
  std::vector<double> rightPosition;
  std::vector<double> leftVelocity;
};

/// \brief Pose of the wheel axle midpoint, given the model pose.
math::Pose3d Axle(const math::Pose3d &_model)
{
  return _model * math::Pose3d(kAxleX, 0, 0, 0, 0, 0);
}

/// \brief Odometry errors against the ground truth of the axle.
struct OdometryError
{
  double position{0.0};
  double yaw{0.0};
  std::size_t count{0};
};

/// \brief Largest difference between the odometry of the robot and the
/// motion of its axle midpoint, in the frame of the first axle pose.
OdometryError CompareOdometry(const GroundTruth &_truth,
    const std::map<std::int64_t, OdometryLog::Sample> &_odometry)
{
  OdometryError error;
  const auto start = Axle(_truth.initial);
  for (const auto &[stamp, sample] : _odometry)
  {
    const auto pose = _truth.poses.find(stamp);
    if (pose == _truth.poses.end())
      continue;
    const auto axle = start.Inverse() * Axle(pose->second);
    error.position = std::max(error.position,
        axle.Pos().Distance(sample.position));
    error.yaw = std::max(error.yaw, std::abs(
        math::Angle(axle.Rot().Yaw() - sample.yaw).Normalized().Radian()));
    ++error.count;
  }
  return error;
}

/// \brief Test fixture.
class SimplifiedPhysicsWheelsTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture<::testing::Test>::SetUp();
    common::setenv("GZ_SIM_RESOURCE_PATH", common::joinPaths(
        std::string(PROJECT_SOURCE_PATH), "test", "worlds", "models").c_str());
  }

  /// \brief Fixture for a world made of the given models.
  protected: static ServerConfig Config(const std::string &_models,
      unsigned int _seed = 0)
  {
    ServerConfig config;
    config.SetSdfString(WorldXml(_models));
    if (_seed != 0)
      config.SetSeed(_seed);
    return config;
  }

  /// \brief Record the model pose and the wheel joint state of the robots
  /// after every step.
  protected: static void Record(TestFixture &_fixture,
      const std::vector<std::string> &_names,
      std::map<std::string, GroundTruth> &_truth)
  {
    _fixture.OnConfigure([_names, &_truth](const Entity &,
        const std::shared_ptr<const sdf::Element> &,
        EntityComponentManager &_ecm, EventManager &)
        {
          for (const auto &name : _names)
          {
            const auto model = _ecm.EntityByComponents(
                components::Model(), components::Name(name));
            ASSERT_NE(kNullEntity, model) << name;
            _truth[name].initial = worldPose(model, _ecm);
          }
        })
    .OnPostUpdate([_names, &_truth](const UpdateInfo &_info,
        const EntityComponentManager &_ecm)
        {
          const auto stamp = static_cast<std::int64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  _info.simTime).count());
          for (const auto &name : _names)
          {
            const Model model(_ecm.EntityByComponents(
                components::Model(), components::Name(name)));
            auto &truth = _truth[name];
            truth.poses[stamp] = worldPose(model.Entity(), _ecm);

            const auto left = model.JointByName(_ecm, "left_wheel_joint");
            const auto right = model.JointByName(_ecm, "right_wheel_joint");
            const auto *leftPos =
                _ecm.Component<components::JointPosition>(left);
            const auto *rightPos =
                _ecm.Component<components::JointPosition>(right);
            truth.leftPosition.push_back(
                leftPos && !leftPos->Data().empty() ? leftPos->Data()[0] :
                std::nan(""));
            truth.rightPosition.push_back(
                rightPos && !rightPos->Data().empty() ? rightPos->Data()[0] :
                std::nan(""));
            const auto *leftVel =
                _ecm.Component<components::JointVelocity>(left);
            truth.leftVelocity.push_back(
                leftVel && !leftVel->Data().empty() ? leftVel->Data()[0] :
                std::nan(""));
          }
        });
  }

  /// \brief Send a velocity to the DiffDrive of a robot, and wait until the
  /// plugin has it, so that every robot gets the command in the same step.
  protected: void Command(const std::vector<std::string> &_names,
      double _linear, double _angular)
  {
    transport::Node node;
    msgs::Twist twist;
    twist.mutable_linear()->set_x(_linear);
    twist.mutable_angular()->set_z(_angular);
    for (const auto &name : _names)
    {
      auto pub = node.Advertise<msgs::Twist>("/model/" + name + "/cmd_vel");
      for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ASSERT_TRUE(pub.HasConnections()) << name;
      pub.Publish(twist);
    }
    // The paused server does not step, and the plugin stores the command as
    // the transport thread delivers it.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
};
}

/////////////////////////////////////////////////
/// A DiffDrive robot on planar_kinematic follows a commanded arc, and the
/// odometry it publishes equals the ground truth of the planar motion.
TEST_F(SimplifiedPhysicsWheelsTest, OdometryEqualsGroundTruth)
{
  RobotOptions robot;
  robot.name = "dd";
  robot.planar = true;
  TestFixture fixture(Config(RobotXml(robot)));

  std::map<std::string, GroundTruth> truth;
  Record(fixture, {"dd"}, truth);
  fixture.Finalize();

  OdometryLog odometry;
  odometry.Subscribe("dd");
  this->Command({"dd"}, 0.5, 0.4);

  fixture.Server()->Run(true, 6000, false);
  this->Command({"dd"}, 0.0, 0.0);
  fixture.Server()->Run(true, 2000, false);

  const auto &t = truth["dd"];
  const auto samples = odometry.Samples();
  ASSERT_GT(samples.size(), 7000u);

  const auto end = Axle(t.initial).Inverse() * Axle(t.poses.rbegin()->second);
  EXPECT_GT(end.Pos().Length(), 2.0);
  EXPECT_GT(std::abs(end.Rot().Yaw()), 1.0);

  const auto error = CompareOdometry(t, samples);
  EXPECT_GT(error.count, 7000u);
  EXPECT_LT(error.position, 1e-3);
  EXPECT_LT(error.yaw, 1e-3);
  gzmsg << "odometry error: position " << error.position << " m, yaw "
        << error.yaw << " rad over " << error.count << " samples"
        << std::endl;
}

/////////////////////////////////////////////////
/// The same robot and the same commands on planar_kinematic and on dartsim.
/// The planar model leaves out wheel slip, wheel inertia and contact
/// compliance, so the two differ by a small amount. The test states how
/// much, as a fraction of the distance travelled.
TEST_F(SimplifiedPhysicsWheelsTest, MatchesDartsim)
{
  RobotOptions planar;
  planar.name = "dd_planar";
  planar.pose = "0 0 0.1 0 0 0";
  planar.planar = true;
  planar.sphereWheels = true;
  RobotOptions full;
  full.name = "dd_full";
  full.pose = "0 8 0.1 0 0 0";
  full.sphereWheels = true;

  TestFixture fixture(Config(RobotXml(planar) + RobotXml(full)));
  std::map<std::string, GroundTruth> truth;
  Record(fixture, {"dd_planar", "dd_full"}, truth);
  fixture.Finalize();

  OdometryLog planarOdometry;
  OdometryLog fullOdometry;
  planarOdometry.Subscribe("dd_planar");
  fullOdometry.Subscribe("dd_full");

  // Let the dartsim robot settle on its wheels before it gets a command.
  fixture.Server()->Run(true, 500, false);
  this->Command({"dd_planar", "dd_full"}, 0.5, 0.4);
  fixture.Server()->Run(true, 6000, false);
  this->Command({"dd_planar", "dd_full"}, 0.0, 0.0);
  fixture.Server()->Run(true, 2000, false);

  const auto &a = truth["dd_planar"];
  const auto &b = truth["dd_full"];
  ASSERT_EQ(a.poses.size(), b.poses.size());

  // Compare the paths in the frame of the pose at the time of the first
  // command (after the settling steps).
  auto it = a.poses.begin();
  auto jt = b.poses.begin();
  std::advance(it, 499);
  std::advance(jt, 499);
  const auto startA = it->second;
  const auto startB = jt->second;

  double travelled{0.0};
  double worstPosition{0.0};
  double worstYaw{0.0};
  math::Pose3d previous;
  bool first{true};
  for (; it != a.poses.end(); ++it, ++jt)
  {
    const auto pa = startA.Inverse() * it->second;
    const auto pb = startB.Inverse() * jt->second;
    if (!first)
      travelled += pa.Pos().Distance(previous.Pos());
    previous = pa;
    first = false;
    worstPosition = std::max(worstPosition, pa.Pos().Distance(pb.Pos()));
    worstYaw = std::max(worstYaw, std::abs(math::Angle(
        pa.Rot().Yaw() - pb.Rot().Yaw()).Normalized().Radian()));
  }
  gzmsg << "planar vs dartsim: travelled " << travelled << " m, worst "
        << "position difference " << worstPosition << " m ("
        << 100.0 * worstPosition / travelled << " percent), worst yaw "
        << "difference " << worstYaw << " rad" << std::endl;

  // Measured on this machine: 0.46 mm (0.015 percent) and 0.3 microrad.
  // The limits are the "within 1 percent of the distance" of the accuracy
  // contract for the mode.
  EXPECT_GT(travelled, 2.0);
  EXPECT_LT(worstPosition, 0.01 * travelled);
  EXPECT_LT(worstYaw, 0.01);

  // The odometry of the planar robot equals its ground truth. The dartsim
  // robot's wheels slip a little, so its odometry differs by more.
  const auto planarError = CompareOdometry(a, planarOdometry.Samples());
  const auto fullError = CompareOdometry(b, fullOdometry.Samples());
  gzmsg << "odometry error against ground truth: planar "
        << planarError.position << " m, dartsim " << fullError.position
        << " m" << std::endl;
  EXPECT_LT(planarError.position, 1e-6);
  EXPECT_LT(fullError.position, 0.01 * travelled);
}

namespace
{
/// \brief What one run of the odometry noise world gives.
struct NoiseRun
{
  /// \brief Distance each robot travelled, by name.
  std::vector<double> distance;

  /// \brief Distance between the odometry's end point and the true one.
  std::vector<double> error;

  /// \brief Length of the true path driven.
  double travelled{0.0};
};
}

/////////////////////////////////////////////////
/// With <odometry_noise>, the odometry drifts from the ground truth by the
/// configured fraction of the distance, the motion of the robots stays exact,
/// and a run repeats exactly for a fixed world seed.
TEST_F(SimplifiedPhysicsWheelsTest, OdometryNoise)
{
  constexpr int kRobots{40};
  constexpr double kFraction{0.015};

  const auto run = [this](unsigned int _seed, double _fraction)
  {
    std::string models;
    std::vector<std::string> names;
    for (int i = 0; i < kRobots; ++i)
    {
      RobotOptions robot;
      robot.name = "noisy" + std::to_string(i);
      robot.pose = "0 " + std::to_string(i) + " 0.1 0 0 0";
      robot.planar = true;
      robot.planarParams = "<odometry_noise>" + std::to_string(_fraction) +
          "</odometry_noise>";
      robot.odomHz = 10;
      models += RobotXml(robot);
      names.push_back(robot.name);
    }

    TestFixture fixture(Config(models, _seed));
    std::map<std::string, GroundTruth> truth;
    Record(fixture, names, truth);
    fixture.Finalize();

    std::vector<OdometryLog> logs(names.size());
    for (std::size_t i = 0; i < names.size(); ++i)
      logs[i].Subscribe(names[i]);
    this->Command(names, 1.0, 0.0);
    fixture.Server()->Run(true, 10250, false);

    NoiseRun result;
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      const auto &t = truth[names[i]];
      const auto samples = logs[i].Samples();
      EXPECT_GT(samples.size(), 100u) << names[i];

      // The last odometry message and the ground truth at its time.
      const auto last = samples.rbegin();
      const auto axle = Axle(t.initial).Inverse() *
          Axle(t.poses.at(last->first));
      result.travelled = axle.Pos().Length();
      result.error.push_back(axle.Pos().Distance(last->second.position));
      result.distance.push_back(result.travelled);
    }
    return result;
  };

  const auto first = run(7, kFraction);
  const auto repeat = run(7, kFraction);
  const auto other = run(8, kFraction);
  const auto exact = run(7, 0.0);

  // The motion does not depend on the noise: every robot drives the same
  // distance, along x.
  for (const auto distance : first.distance)
    EXPECT_NEAR(first.distance[0], distance, 1e-6);
  EXPECT_GT(first.travelled, 9.0);

  // The drift is about the configured fraction. The contract is for a 10 m
  // run, and these runs are a little shorter.
  auto errors = first.error;
  std::sort(errors.begin(), errors.end());
  const double median = errors[errors.size() / 2];
  gzmsg << "odometry noise " << kFraction << ": median drift " <<
      100.0 * median / first.travelled << " percent of " << first.travelled
      << " m, min " << 100.0 * errors.front() / first.travelled << ", max "
      << 100.0 * errors.back() / first.travelled << std::endl;
  EXPECT_GT(median / first.travelled, 0.010);
  EXPECT_LT(median / first.travelled, 0.020);

  // A fixed seed gives the same noise, another seed another one.
  EXPECT_EQ(first.error, repeat.error);
  EXPECT_NE(first.error, other.error);

  // Without the noise the odometry is exact.
  for (const auto error : exact.error)
    EXPECT_LT(error, 1e-6);
}

/////////////////////////////////////////////////
/// The wheels follow the motion that the model achieves, also when a model
/// velocity command moves the model and no wheel controller runs. Wheel
/// travel equals d . (v + wz x p) / r for each contact point.
TEST_F(SimplifiedPhysicsWheelsTest, WheelsFollowModelVelocityCommands)
{
  RobotOptions robot;
  robot.name = "vc";
  robot.planar = true;
  robot.diffDrive = false;
  robot.plugins = R"(
      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
        <initial_linear>0.5 0 0</initial_linear>
        <initial_angular>0 0 0.4</initial_angular>
      </plugin>)";
  TestFixture fixture(Config(RobotXml(robot)));

  std::vector<double> left;
  std::vector<double> right;
  std::vector<double> leftVelocity;
  Entity leftJoint{kNullEntity};
  Entity rightJoint{kNullEntity};
  fixture.OnConfigure([&](const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &_ecm, EventManager &)
      {
        const Model model(_ecm.EntityByComponents(
            components::Model(), components::Name("vc")));
        leftJoint = model.JointByName(_ecm, "left_wheel_joint");
        rightJoint = model.JointByName(_ecm, "right_wheel_joint");
        for (const auto joint : {leftJoint, rightJoint})
        {
          _ecm.CreateComponent(joint, components::JointPosition());
          _ecm.CreateComponent(joint, components::JointVelocity());
        }
      })
  .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        left.push_back(
            _ecm.Component<components::JointPosition>(leftJoint)->Data()[0]);
        right.push_back(
            _ecm.Component<components::JointPosition>(rightJoint)->Data()[0]);
        leftVelocity.push_back(
            _ecm.Component<components::JointVelocity>(leftJoint)->Data()[0]);
      }).Finalize();
  fixture.Server()->Run(true, 1000, false);

  // The command is the velocity of the chassis at the model origin, 0.5 m/s
  // and 0.4 rad/s. A wheel contact 0.2 m to the side moves 0.5 -+ 0.08 m/s.
  ASSERT_EQ(1000u, left.size());
  EXPECT_NEAR(4.2, left.back(), 1e-9);
  EXPECT_NEAR(5.8, right.back(), 1e-9);
  EXPECT_NEAR(4.2, leftVelocity.back(), 1e-9);
  EXPECT_NEAR(4.2 * 0.5, left[499], 1e-9);
}

/////////////////////////////////////////////////
/// A model velocity command wins over wheel commands, with one warning, and
/// the wheels follow the motion that results.
TEST_F(SimplifiedPhysicsWheelsTest, ModelVelocityCommandWinsOverWheels)
{
  RobotOptions robot;
  robot.name = "both";
  robot.planar = true;
  robot.plugins = R"(
      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
        <topic>/both_unused</topic>
        <initial_linear>0.2 0 0</initial_linear>
      </plugin>)";

  ::testing::internal::CaptureStderr();
  TestFixture fixture(Config(RobotXml(robot)));
  std::map<std::string, GroundTruth> truth;
  Record(fixture, {"both"}, truth);
  fixture.Finalize();

  this->Command({"both"}, 0.5, 0.0);
  fixture.Server()->Run(true, 2000, false);
  const auto output = ::testing::internal::GetCapturedStderr();

  const auto &t = truth["both"];
  // VelocityControl moves the model at 0.2 m/s, whatever DiffDrive asks.
  EXPECT_NEAR(0.4, t.poses.rbegin()->second.Pos().X(), 1e-6);
  // The wheels turn at 2 rad/s with it, not at the DiffDrive command.
  EXPECT_NEAR(4.0, t.leftPosition.back(), 1e-9);

  const std::string text{"Model [both] gets a model velocity command and "
                         "wheel commands in the same step"};
  const auto pos = output.find(text);
  ASSERT_NE(std::string::npos, pos) << output;
  EXPECT_EQ(std::string::npos, output.find(text, pos + 1)) << output;
}

/////////////////////////////////////////////////
/// A <wheel> element replaces the inference, and its radius replaces the
/// radius of the collision. A name that is no joint gets a warning.
TEST_F(SimplifiedPhysicsWheelsTest, WheelElementOverridesInference)
{
  RobotOptions inferred;
  inferred.name = "inferred";
  inferred.pose = "0 0 0.1 0 0 0";
  inferred.planar = true;

  // DiffDrive believes the radius is 0.1 m, so a command of 0.25 m/s turns
  // the wheels at 2.5 rad/s. With a radius of 0.2 m that moves the robot at
  // 0.5 m/s.
  RobotOptions wide;
  wide.name = "wide";
  wide.pose = "0 2 0.1 0 0 0";
  wide.planar = true;
  wide.planarParams = R"(
        <wheel radius="0.2">left_wheel_joint</wheel>
        <wheel radius="0.2">right_wheel_joint</wheel>
        <wheel>not_a_joint</wheel>)";

  ::testing::internal::CaptureStderr();
  TestFixture fixture(Config(RobotXml(inferred) + RobotXml(wide)));
  std::map<std::string, GroundTruth> truth;
  Record(fixture, {"inferred", "wide"}, truth);
  fixture.Finalize();

  this->Command({"inferred", "wide"}, 0.25, 0.0);
  fixture.Server()->Run(true, 3000, false);
  const auto output = ::testing::internal::GetCapturedStderr();

  const double inferredX = truth["inferred"].poses.rbegin()->second.Pos().X();
  const double wideX = truth["wide"].poses.rbegin()->second.Pos().X();
  EXPECT_NEAR(0.25 * 3 - 0.5 * 0.25 * 0.125, inferredX, 1e-3);
  EXPECT_NEAR(2.0 * inferredX, wideX, 1e-6);

  EXPECT_NE(std::string::npos, output.find("Model [wide] names wheel joint "
      "[not_a_joint], which is not a revolute or continuous joint of the "
      "model")) << output;
}

/////////////////////////////////////////////////
/// Joints that are no wheels integrate their velocity commands into their
/// position and stop at their limits. A model with no wheels gets a warning
/// for odometry_noise. A command on a joint with two axes is ignored, with
/// one warning.
TEST_F(SimplifiedPhysicsWheelsTest, ArticulatedJoints)
{
  const std::string model = R"(
    <model name="lifter">
      <pose>0 0 0.1 0 0 0</pose>
      <link name="chassis">
        <collision name="collision">
          <geometry><box><size>0.4 0.4 0.1</size></box></geometry>
        </collision>
      </link>
      <link name="deck">
        <pose>0 0 0.1 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>0.3 0.3 0.02</size></box></geometry>
        </collision>
      </link>
      <link name="plate">
        <pose>0 0 0.2 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>0.2 0.2 0.02</size></box></geometry>
        </collision>
      </link>
      <link name="gimbal_link">
        <pose>0 0 0.3 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>0.1 0.1 0.02</size></box></geometry>
        </collision>
      </link>
      <joint name="lift" type="prismatic">
        <parent>chassis</parent>
        <child>deck</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit><lower>0</lower><upper>0.2</upper></limit>
        </axis>
      </joint>
      <joint name="turntable" type="continuous">
        <parent>deck</parent>
        <child>plate</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="gimbal" type="revolute2">
        <parent>plate</parent>
        <child>gimbal_link</child>
        <axis><xyz>1 0 0</xyz></axis>
        <axis2><xyz>0 1 0</xyz></axis2>
      </joint>
      <plugin filename="gz-sim-simplified-physics-system"
              name="gz::sim::systems::SimplifiedPhysicsModel">
        <mode>planar_kinematic</mode>
        <odometry_noise>0.01</odometry_noise>
      </plugin>
    </model>)";

  ::testing::internal::CaptureStderr();
  TestFixture fixture(Config(model));

  Entity lift{kNullEntity};
  Entity turntable{kNullEntity};
  Entity gimbal{kNullEntity};
  Entity modelEntity{kNullEntity};
  int step{0};
  std::vector<double> liftPosition;
  std::vector<double> liftVelocity;
  std::vector<double> turntablePosition;
  std::vector<double> gimbalPosition;  // Size of the position data.
  math::Pose3d first;
  math::Pose3d last;
  fixture.OnConfigure([&](const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &_ecm, EventManager &)
      {
        modelEntity = _ecm.EntityByComponents(
            components::Model(), components::Name("lifter"));
        const Model m(modelEntity);
        lift = m.JointByName(_ecm, "lift");
        turntable = m.JointByName(_ecm, "turntable");
        gimbal = m.JointByName(_ecm, "gimbal");
        for (const auto joint : {lift, turntable, gimbal})
        {
          _ecm.CreateComponent(joint, components::JointPosition());
          _ecm.CreateComponent(joint, components::JointVelocity());
        }
        first = worldPose(modelEntity, _ecm);
      })
  .OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        ++step;
        // Lift up for 3 s (it stops at 0.2 m after 2 s), turn the table at
        // 1 rad/s for 2 s, and command the two-axis joint in steps 10 to 12.
        if (step <= 3000)
          _ecm.SetComponentData<components::JointVelocityCmd>(lift, {0.1});
        if (step <= 2000)
          _ecm.SetComponentData<components::JointVelocityCmd>(turntable, {1.0});
        if (step >= 10 && step <= 12)
        {
          _ecm.SetComponentData<components::JointVelocityCmd>(gimbal,
              {1.0, 1.0});
        }
        // Reset the lift in step 3500.
        if (step == 3500)
        {
          _ecm.SetComponentData<components::JointPositionReset>(lift, {0.05});
        }
      })
  .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        liftPosition.push_back(
            _ecm.Component<components::JointPosition>(lift)->Data()[0]);
        liftVelocity.push_back(
            _ecm.Component<components::JointVelocity>(lift)->Data()[0]);
        turntablePosition.push_back(
            _ecm.Component<components::JointPosition>(turntable)->Data()[0]);
        gimbalPosition.push_back(static_cast<double>(
            _ecm.Component<components::JointPosition>(gimbal)->Data().size()));
        last = worldPose(modelEntity, _ecm);
      }).Finalize();
  fixture.Server()->Run(true, 4000, false);
  const auto output = ::testing::internal::GetCapturedStderr();

  ASSERT_EQ(4000u, liftPosition.size());
  // Rising at 0.1 m/s, then held by the limit.
  EXPECT_NEAR(0.1, liftPosition[999], 1e-9);
  EXPECT_NEAR(0.1, liftVelocity[999], 1e-9);
  EXPECT_NEAR(0.2, liftPosition[2999], 1e-9);
  EXPECT_NEAR(0.0, liftVelocity[2999], 1e-9);
  EXPECT_NEAR(0.2, liftPosition[3498], 1e-9);
  // The reset sets the position, and the command ended before.
  EXPECT_NEAR(0.05, liftPosition[3499], 1e-9);
  EXPECT_NEAR(0.05, liftPosition.back(), 1e-9);

  // A continuous joint turns at the commanded rate, and holds its angle when
  // the command stops.
  EXPECT_NEAR(1.0, turntablePosition[999], 1e-9);
  EXPECT_NEAR(2.0, turntablePosition[1999], 1e-9);
  EXPECT_NEAR(2.0, turntablePosition.back(), 1e-9);

  // The two-axis joint is not followed: its position is never filled.
  EXPECT_EQ(0.0, *std::max_element(gimbalPosition.begin(),
      gimbalPosition.end()));
  const std::string text{"has more than one degree of freedom"};
  const auto pos = output.find(text);
  ASSERT_NE(std::string::npos, pos) << output;
  EXPECT_EQ(std::string::npos, output.find(text, pos + 1)) << output;

  // The model itself does not move, and odometry_noise has no wheels to act
  // on.
  EXPECT_EQ(first, last);
  EXPECT_NE(std::string::npos, output.find("Model [lifter] asks for "
      "odometry_noise, but has no wheel joints")) << output;
}

/////////////////////////////////////////////////
/// A robot that has not moved still fills the wheel positions, so that
/// DiffDrive publishes odometry from the first step.
TEST_F(SimplifiedPhysicsWheelsTest, IdleRobotPublishesOdometry)
{
  RobotOptions robot;
  robot.name = "idle";
  robot.planar = true;
  robot.odomHz = 100;
  TestFixture fixture(Config(RobotXml(robot)));
  fixture.Finalize();

  OdometryLog odometry;
  odometry.Subscribe("idle");
  fixture.Server()->Run(true, 500, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const auto samples = odometry.Samples();
  EXPECT_GT(samples.size(), 3u);
  for (const auto &[stamp, sample] : samples)
  {
    EXPECT_EQ(0.0, sample.position.Length()) << stamp;
    EXPECT_EQ(0.0, sample.yaw) << stamp;
  }
}

/////////////////////////////////////////////////
/// A world reset puts the wheels back to zero, and the robot drives again.
/// (DiffDrive does not drive again after a reset, on any physics provider,
/// so a VelocityControl moves this robot.)
TEST_F(SimplifiedPhysicsWheelsTest, Reset)
{
  RobotOptions robot;
  robot.name = "reset";
  robot.planar = true;
  robot.diffDrive = false;
  robot.plugins = R"(
      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
        <initial_linear>0.5 0 0</initial_linear>
      </plugin>)";
  TestFixture fixture(Config(RobotXml(robot)));

  Entity leftJoint{kNullEntity};
  Entity model{kNullEntity};
  std::vector<double> left;
  std::vector<double> x;
  fixture.OnConfigure([&](const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &_ecm, EventManager &)
      {
        model = _ecm.EntityByComponents(
            components::Model(), components::Name("reset"));
        leftJoint = Model(model).JointByName(_ecm, "left_wheel_joint");
      })
  .OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        // A reset drops components that were added after the start, so the
        // test adds this one again, as a consumer such as DiffDrive does.
        if (!_ecm.Component<components::JointPosition>(leftJoint))
          _ecm.CreateComponent(leftJoint, components::JointPosition());
      })
  .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        const auto *position =
            _ecm.Component<components::JointPosition>(leftJoint);
        left.push_back(
            position && !position->Data().empty() ? position->Data()[0] : -1);
        x.push_back(worldPose(model, _ecm).Pos().X());
      }).Finalize();

  fixture.Server()->Run(true, 1000, false);
  EXPECT_NEAR(5.0, left.back(), 1e-9);

  msgs::WorldControl req;
  msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  transport::Node node;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/wheels_test/control", req, 1000, rep,
      result));
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());

  fixture.Server()->Run(true, 500, false);
  // The step that carries out the reset does not end in a PostUpdate.
  ASSERT_GE(left.size(), 1490u);

  // The wheels start from zero again: 0.5 m/s turns a wheel of radius 0.1 m
  // at 5 rad/s. The robot is half a meter from its start.
  EXPECT_NEAR(2.5, left.back(), 0.02);
  EXPECT_NEAR(0.25, x.back(), 0.01);
}

/////////////////////////////////////////////////
/// One model file, two physics modes, and the noise chosen at include time.
TEST_F(SimplifiedPhysicsWheelsTest, ChosenAtInclude)
{
  const std::string models = R"(
    <include>
      <uri>model://simplified_physics_diff_drive</uri>
      <name>inc_full</name>
      <pose>0 6 0.1 0 0 0</pose>
    </include>
    <include>
      <uri>model://simplified_physics_diff_drive</uri>
      <name>inc_planar</name>
      <pose>0 0 0.1 0 0 0</pose>
      <experimental:params>
        <plugin element_id="" action="add"
            name="gz::sim::systems::SimplifiedPhysicsModel"
            filename="gz-sim-simplified-physics-system">
          <mode>planar_kinematic</mode>
          <odometry_noise>0.05</odometry_noise>
        </plugin>
      </experimental:params>
    </include>)";

  TestFixture fixture(Config(models, 11));
  std::map<std::string, GroundTruth> truth;
  Record(fixture, {"inc_full", "inc_planar"}, truth);
  fixture.Finalize();

  OdometryLog planarOdometry;
  planarOdometry.Subscribe("inc_planar");
  fixture.Server()->Run(true, 300, false);
  this->Command({"inc_full", "inc_planar"}, 0.5, 0.0);
  fixture.Server()->Run(true, 4000, false);

  const auto &planar = truth["inc_planar"];
  const auto &full = truth["inc_full"];
  const double planarX = planar.poses.rbegin()->second.Pos().X();
  const double fullX = full.poses.rbegin()->second.Pos().X();
  EXPECT_GT(planarX, 1.5);
  EXPECT_NEAR(fullX, planarX, 0.01 * planarX);

  // The noise shows in the odometry of the planar robot and not in its
  // motion.
  const auto error = CompareOdometry(planar, planarOdometry.Samples());
  gzmsg << "noisy odometry: largest error " << error.position << " m over "
        << planarX << " m" << std::endl;
  EXPECT_GT(error.position, 1e-3);
  EXPECT_NEAR(0.0, planar.poses.rbegin()->second.Pos().Y(), 1e-9);
}

/////////////////////////////////////////////////
/// The system computes link frame data only when some component asks for it,
/// and a component that appears later gets its value in the next step, also
/// for a robot that does not move.
TEST_F(SimplifiedPhysicsWheelsTest, FrameDataFollowsReaders)
{
  RobotOptions robot;
  robot.name = "frames";
  robot.pose = "1 2 0.1 0 0 0.5";
  robot.planar = true;
  robot.diffDrive = false;
  TestFixture fixture(Config(RobotXml(robot)));

  Entity chassis{kNullEntity};
  Entity wheel{kNullEntity};
  int step{0};
  std::vector<math::Pose3d> poses;
  std::vector<bool> present;
  fixture.OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        ++step;
        if (step == 1)
        {
          const Model model(_ecm.EntityByComponents(
              components::Model(), components::Name("frames")));
          chassis = model.LinkByName(_ecm, "chassis");
          wheel = model.LinkByName(_ecm, "left_wheel");
        }
        // A reader appears in step 10, on a wheel link that does not move.
        if (step == 10)
        {
          _ecm.CreateComponent(wheel,
              components::WorldPose(math::Pose3d::Zero));
        }
      })
  .OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        const auto *pose = _ecm.Component<components::WorldPose>(wheel);
        present.push_back(pose != nullptr);
        poses.push_back(pose ? pose->Data() : math::Pose3d::Zero);
      }).Finalize();
  fixture.Server()->Run(true, 20, false);

  ASSERT_EQ(20u, poses.size());
  EXPECT_FALSE(present[8]);
  ASSERT_TRUE(present[9]);

  // The wheel link sits 0.05 m ahead and 0.2 m to the left of the model
  // origin, which is at (1, 2) and faces 0.5 rad.
  const math::Pose3d want = math::Pose3d(1, 2, 0.1, 0, 0, 0.5) *
      math::Pose3d(0.05, 0.2, 0, -1.570796327, 0, 0);
  for (std::size_t i : {10u, 15u, 19u})
  {
    EXPECT_NEAR(want.Pos().X(), poses[i].Pos().X(), 1e-9) << i;
    EXPECT_NEAR(want.Pos().Y(), poses[i].Pos().Y(), 1e-9) << i;
    EXPECT_NEAR(want.Pos().Z(), poses[i].Pos().Z(), 1e-9) << i;
  }
}

/////////////////////////////////////////////////
/// The wheels hang on a link that is welded to the canonical link. Many
/// models put a massless footprint link first.
TEST_F(SimplifiedPhysicsWheelsTest, WheelsOnALinkWeldedToTheCanonicalLink)
{
  RobotOptions robot;
  robot.name = "welded";
  robot.planar = true;
  robot.footprint = true;
  TestFixture fixture(Config(RobotXml(robot)));
  std::map<std::string, GroundTruth> truth;
  Record(fixture, {"welded"}, truth);
  fixture.Finalize();

  OdometryLog odometry;
  odometry.Subscribe("welded");
  this->Command({"welded"}, 0.5, 0.4);
  fixture.Server()->Run(true, 3000, false);

  const auto &t = truth["welded"];
  EXPECT_GT(Axle(t.initial).Pos().Distance(
      Axle(t.poses.rbegin()->second).Pos()), 0.5);
  const auto error = CompareOdometry(t, odometry.Samples());
  EXPECT_GT(error.count, 2000u);
  EXPECT_LT(error.position, 1e-6);
  EXPECT_LT(error.yaw, 1e-6);
}
