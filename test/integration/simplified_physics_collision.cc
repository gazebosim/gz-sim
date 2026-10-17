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
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief Name of the SimplifiedPhysicsModel plugin.
const char *kModelPlugin = "gz::sim::systems::SimplifiedPhysicsModel";

/// \brief Distance of the axle from the model origin, along the model x axis.
constexpr double kAxleX{0.05};

/// \brief Distance from the model origin to the front of the chassis, and
/// the wall's clearance that the collision code keeps (its `contactSkin`).
constexpr double kFront{0.2};
constexpr double kSkin{1e-4};

/// \brief Wheel radius of the test robot.
constexpr double kRadius{0.1};

/// \brief Settings of one test robot.
struct RobotOptions
{
  /// \brief Model name. DiffDrive topics derive from it.
  std::string name;

  /// \brief Pose in the world, as SDF text.
  std::string pose{"0 0 0.1 0 0 0"};

  /// \brief True to run the robot on planar_kinematic.
  bool planar{true};

  /// \brief More parameters of SimplifiedPhysicsModel, as SDF text.
  std::string params;

  /// \brief True to give every collision a zero collide_bitmask, which takes
  /// the model out of the collision world.
  bool ghost{false};
};

/// \brief SDF text of a differential drive robot: a 0.4 m by 0.2 m chassis,
/// two 4 cm wide cylinder wheels 0.4 m apart, and a ball caster. The wheel
/// axle is kAxleX ahead of the model origin.
std::string RobotXml(const RobotOptions &_o)
{
  const std::string bitmask = _o.ghost ?
      "<surface><contact><collide_bitmask>0x00</collide_bitmask></contact>"
      "</surface>" : "";
  std::ostringstream out;
  out << R"(
    <model name=")" << _o.name << R"(">
      <pose>)" << _o.pose << R"(</pose>
      <link name="chassis">
        <inertial>
          <mass>2</mass>
          <inertia><ixx>0.0073</ixx><iyy>0.0272</iyy><izz>0.0333</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.4 0.2 0.06</size></box></geometry>)" <<
      bitmask << R"(
        </collision>
      </link>)";
  for (const double side : {1.0, -1.0})
  {
    const char *name = side > 0 ? "left" : "right";
    out << R"(
      <link name=")" << name << R"(_wheel">
        <pose relative_to="chassis">)" << kAxleX << " " << side * 0.2 <<
      R"( 0 -1.570796327 0 0</pose>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.00132</ixx><iyy>0.00132</iyy><izz>0.0025</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>0.04</length></cylinder>
          </geometry>)" << bitmask << R"(
        </collision>
      </link>)";
  }
  out << R"(
      <link name="caster">
        <pose relative_to="chassis">-0.2 0 -0.07 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>0</mu><mu2>0</mu2></ode></friction>)" <<
      (_o.ghost ? "<contact><collide_bitmask>0x00</collide_bitmask></contact>"
                : "") << R"(
          </surface>
        </collision>
      </link>
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
      </joint>
      <plugin filename="gz-sim-diff-drive-system"
              name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.4</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
        <odom_publish_frequency>1000</odom_publish_frequency>
      </plugin>)";
  if (_o.planar)
  {
    out << R"(
      <plugin filename="gz-sim-simplified-physics-system"
              name=")" << kModelPlugin << R"(">
        <mode>planar_kinematic</mode>)" << _o.params << R"(
      </plugin>)";
  }
  out << "\n    </model>\n";
  return out.str();
}

/// \brief SDF text of a static wall: a box with its center at (x, y).
std::string WallXml(const std::string &_name, double _x, double _y,
    double _sizeX, double _sizeY)
{
  std::ostringstream out;
  out << R"(
    <model name=")" << _name << R"(">
      <static>true</static>
      <pose>)" << _x << " " << _y << R"( 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>)" << _sizeX << " " << _sizeY <<
      R"( 1</size></box></geometry>
        </collision>
      </link>
    </model>
)";
  return out.str();
}

/// \brief SDF text of a 20 kg box that the default physics system simulates.
std::string BoxXml(const std::string &_name, const std::string &_pose)
{
  return R"(
    <model name=")" + _name + R"(">
      <pose>)" + _pose + R"(</pose>
      <link name="link">
        <inertial>
          <mass>20</mass>
          <inertia>
            <ixx>0.8</ixx><iyy>0.8</iyy><izz>0.8</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
        </collision>
      </link>
    </model>
)";
}

/// \brief SDF text of a world with a ground and the models.
std::string WorldXml(const std::string &_models)
{
  return R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="collision_test">
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

/// \brief What the odometry of one robot says, by message time.
class OdometryLog
{
  /// \brief Subscribe to the odometry topic of a robot.
  public: void Subscribe(const std::string &_model)
  {
    ASSERT_TRUE(this->node.Subscribe("/model/" + _model + "/odometry",
        &OdometryLog::OnMessage, this));
  }

  /// \brief Position and heading of a message.
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

/// \brief The pose and the left wheel of a model after every step.
struct Track
{
  /// \brief First pose.
  math::Pose3d initial;

  /// \brief Poses by simulation time in nanoseconds.
  std::map<std::int64_t, math::Pose3d> byStamp;

  /// \brief Poses in the order of the steps.
  std::vector<math::Pose3d> poses;

  /// \brief JointPosition of the left wheel after every step.
  std::vector<double> leftWheel;

  /// \brief JointVelocity of the left wheel after every step.
  std::vector<double> leftVelocity;

  /// \brief Last pose.
  const math::Pose3d &Last() const
  {
    return this->poses.back();
  }

  /// \brief Distance from the pose of step i to the pose of step j.
  double Travel(std::size_t _i, std::size_t _j) const
  {
    return this->poses.at(_i).Pos().Distance(this->poses.at(_j).Pos());
  }
};

/// \brief Pose of the wheel axle midpoint, given the model pose.
math::Pose3d Axle(const math::Pose3d &_model)
{
  return _model * math::Pose3d(kAxleX, 0, 0, 0, 0, 0);
}

/// \brief Largest x of the footprint of the test robot: the chassis corners
/// and the wheels.
double FrontMostX(const math::Pose3d &_pose)
{
  double most = -1e9;
  for (const double sx : {-0.2, 0.2})
  {
    for (const double sy : {-0.1, 0.1})
    {
      most = std::max(most,
          _pose.CoordPositionAdd(math::Vector3d(sx, sy, 0)).X());
    }
  }
  for (const double sx : {-0.1, 0.1})
  {
    for (const double sy : {-0.22, -0.18, 0.18, 0.22})
    {
      most = std::max(most,
          _pose.CoordPositionAdd(math::Vector3d(kAxleX + sx, sy, 0)).X());
    }
  }
  return most;
}

/// \brief Test fixture.
class SimplifiedPhysicsCollisionTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture<::testing::Test>::SetUp();
    common::setenv("GZ_SIM_RESOURCE_PATH", common::joinPaths(
        std::string(PROJECT_SOURCE_PATH), "test", "worlds", "models").c_str());
  }

  /// \brief Server configuration for a world made of the given models.
  protected: static ServerConfig Config(const std::string &_models)
  {
    ServerConfig config;
    config.SetSdfString(WorldXml(_models));
    return config;
  }

  /// \brief Record the pose of models after every step, and the left wheel
  /// of the ones that have one.
  protected: static void Record(TestFixture &_fixture,
      const std::vector<std::string> &_names,
      std::map<std::string, Track> &_tracks)
  {
    // A consumer such as a joint state publisher asks for the velocity, and
    // the provider only fills components that exist.
    _fixture.OnConfigure([_names](const Entity &,
        const std::shared_ptr<const sdf::Element> &,
        EntityComponentManager &_ecm, EventManager &)
        {
          for (const auto &name : _names)
          {
            const Model model(_ecm.EntityByComponents(
                components::Model(), components::Name(name)));
            if (!model.Valid(_ecm))
              continue;
            const auto left = model.JointByName(_ecm, "left_wheel_joint");
            if (left != kNullEntity)
              _ecm.CreateComponent(left, components::JointVelocity());
          }
        })
    .OnPostUpdate([_names, &_tracks](const UpdateInfo &_info,
        const EntityComponentManager &_ecm)
        {
          const auto stamp = static_cast<std::int64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  _info.simTime).count());
          for (const auto &name : _names)
          {
            const auto entity = _ecm.EntityByComponents(
                components::Model(), components::Name(name));
            if (entity == kNullEntity)
              continue;
            auto &track = _tracks[name];
            const auto pose = worldPose(entity, _ecm);
            if (track.poses.empty())
              track.initial = pose;
            track.poses.push_back(pose);
            track.byStamp[stamp] = pose;

            const Model model(entity);
            const auto left = model.JointByName(_ecm, "left_wheel_joint");
            const auto *position = left == kNullEntity ? nullptr :
                _ecm.Component<components::JointPosition>(left);
            const auto *velocity = left == kNullEntity ? nullptr :
                _ecm.Component<components::JointVelocity>(left);
            track.leftWheel.push_back(position && !position->Data().empty() ?
                position->Data()[0] : std::nan(""));
            track.leftVelocity.push_back(velocity && !velocity->Data().empty()
                ? velocity->Data()[0] : std::nan(""));
          }
        });
  }

  /// \brief Send a velocity to the DiffDrive of robots, and wait until the
  /// plugins have it, so that every robot gets the command in the same step.
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

  /// \brief Create a model from SDF text with the create service.
  protected: static void Create(const std::string &_models)
  {
    transport::Node node;
    msgs::EntityFactory req;
    req.set_sdf("<?xml version='1.0'?><sdf version='1.9'>" + _models +
        "</sdf>");
    msgs::Boolean rep;
    bool result{false};
    ASSERT_TRUE(node.Request("/world/collision_test/create", req, 2000, rep,
        result));
    ASSERT_TRUE(result);
  }

  /// \brief Remove a model with the remove service.
  protected: static void Remove(const std::string &_name)
  {
    transport::Node node;
    msgs::Entity req;
    req.set_name(_name);
    req.set_type(msgs::Entity::MODEL);
    msgs::Boolean rep;
    bool result{false};
    ASSERT_TRUE(node.Request("/world/collision_test/remove", req, 2000, rep,
        result));
    ASSERT_TRUE(result);
  }
};

/// \brief Check that a robot's wheels and odometry followed the motion that
/// it made: the left wheel of a robot that only drives straight turned by the
/// distance of its axle over the radius, and the last odometry message
/// equals the ground truth.
void ExpectOdometryEqualsGroundTruth(const Track &_track,
    OdometryLog &_log)
{
  const auto samples = _log.Samples();
  ASSERT_FALSE(samples.empty());
  const auto start = Axle(_track.initial);
  std::size_t compared{0};
  double worstPosition{0.0};
  double worstYaw{0.0};
  for (const auto &[stamp, sample] : samples)
  {
    const auto pose = _track.byStamp.find(stamp);
    if (pose == _track.byStamp.end())
      continue;
    const auto axle = start.Inverse() * Axle(pose->second);
    worstPosition = std::max(worstPosition,
        axle.Pos().Distance(sample.position));
    worstYaw = std::max(worstYaw, std::abs(math::Angle(
        axle.Rot().Yaw() - sample.yaw).Normalized().Radian()));
    ++compared;
  }
  EXPECT_GT(compared, _track.poses.size() / 2);
  EXPECT_LT(worstPosition, 1e-6);
  EXPECT_LT(worstYaw, 1e-6);
}
}

/////////////////////////////////////////////////
/// A DiffDrive robot that drives into a wall stops 0.1 mm from it, in both
/// collision responses, and its wheels and odometry stop with it.
TEST_F(SimplifiedPhysicsCollisionTest, StopsAtAWall)
{
  for (const std::string response : {"stop", "slide"})
  {
    RobotOptions robot;
    robot.name = "dd";
    robot.params = "<collision_response>" + response +
        "</collision_response>";
    // Wall face at x = 2.9. The robot reaches it at 5.4 s.
    TestFixture fixture(Config(RobotXml(robot) +
        WallXml("wall", 3.0, 0.0, 0.2, 10.0)));

    std::map<std::string, Track> tracks;
    Record(fixture, {"dd"}, tracks);
    fixture.Finalize();

    OdometryLog odometry;
    odometry.Subscribe("dd");
    this->Command({"dd"}, 0.5, 0.0);
    fixture.Server()->Run(true, 8000, false);

    const auto &track = tracks["dd"];
    ASSERT_EQ(8000u, track.poses.size());
    const double stop = 2.9 - kFront;
    EXPECT_NEAR(stop - kSkin, track.Last().Pos().X(), 1e-6) << response;
    EXPECT_NEAR(0.0, track.Last().Pos().Y(), 1e-9) << response;
    EXPECT_NEAR(0.0, track.Last().Rot().Yaw(), 1e-9) << response;

    // The robot never goes through the wall, and it never backs off.
    for (std::size_t i = 1; i < track.poses.size(); ++i)
    {
      EXPECT_LE(track.poses[i].Pos().X(), stop) << response << " " << i;
      EXPECT_GE(track.poses[i].Pos().X(),
          track.poses[i - 1].Pos().X() - 1e-12) << response << " " << i;
    }

    // At rest for the last two seconds, wheels included.
    EXPECT_EQ(track.poses[6000].Pos().X(), track.Last().Pos().X());
    EXPECT_EQ(track.leftWheel[6000], track.leftWheel.back()) << response;
    EXPECT_EQ(0.0, track.leftVelocity.back()) << response;

    // The wheel turned for the distance that the robot moved, no more.
    EXPECT_NEAR(track.Last().Pos().X() / kRadius, track.leftWheel.back(),
        1e-6) << response;
    ExpectOdometryEqualsGroundTruth(track, odometry);
  }
}

/////////////////////////////////////////////////
/// Driving at a wall at an angle: Stop ends where the corner touches. Slide
/// keeps the part of the motion along the wall and never goes through it. The
/// wheels only roll along the heading, so a robot that slides sideways along
/// a wall has wheel odometry that cannot see the sideways part.
TEST_F(SimplifiedPhysicsCollisionTest, ObliqueApproachStopsOrSlides)
{
  for (const std::string response : {"stop", "slide"})
  {
    RobotOptions robot;
    robot.name = "dd";
    robot.pose = "0 -2 0.1 0 0 0.5";
    robot.params = "<collision_response>" + response +
        "</collision_response>";
    TestFixture fixture(Config(RobotXml(robot) +
        WallXml("wall", 3.0, 0.0, 0.2, 20.0)));
    std::map<std::string, Track> tracks;
    Record(fixture, {"dd"}, tracks);
    fixture.Finalize();

    this->Command({"dd"}, 0.5, 0.0);
    fixture.Server()->Run(true, 9000, false);
    const auto &track = tracks["dd"];

    for (std::size_t i = 0; i < track.poses.size(); ++i)
      ASSERT_LE(FrontMostX(track.poses[i]), 2.9) << response << " " << i;

    // The heading does not change: nothing commands a turn.
    EXPECT_NEAR(0.5, track.Last().Rot().Yaw(), 1e-9) << response;

    // The motion in the last two seconds.
    const auto n = track.poses.size();
    const double lastTwoSeconds = track.Travel(n - 2001, n - 1);
    if (response == "stop")
    {
      EXPECT_EQ(0.0, lastTwoSeconds);
      EXPECT_EQ(0.0, track.leftVelocity.back());
    }
    else
    {
      // The robot drives at 0.5 m/s along its heading of 0.5 rad. The
      // component along the wall (y) is 0.24 m/s, a little less to keep the
      // gap. The wall component of the robot's own x motion is dropped.
      EXPECT_NEAR(2.0 * 0.5 * std::sin(0.5), lastTwoSeconds, 0.01);
      EXPECT_NEAR(track.poses[n - 2001].Pos().X(), track.Last().Pos().X(),
          2e-4);
      EXPECT_GT(track.Last().Pos().Y(), track.poses[n - 2001].Pos().Y());
    }
  }
}

/////////////////////////////////////////////////
/// Two planar robots that drive at each other stop face to face without
/// overlap, and do not pass each other.
TEST_F(SimplifiedPhysicsCollisionTest, HeadOnRobotsDoNotOverlap)
{
  RobotOptions a;
  a.name = "a";
  a.pose = "0 0 0.1 0 0 0";
  RobotOptions b;
  b.name = "b";
  b.pose = "4 0 0.1 0 0 3.141592653589793";
  TestFixture fixture(Config(RobotXml(a) + RobotXml(b)));
  std::map<std::string, Track> tracks;
  Record(fixture, {"a", "b"}, tracks);
  fixture.Finalize();

  this->Command({"a", "b"}, 0.5, 0.0);
  fixture.Server()->Run(true, 7000, false);

  const auto &ta = tracks["a"];
  const auto &tb = tracks["b"];
  ASSERT_EQ(7000u, ta.poses.size());
  double smallest = 1e9;
  for (std::size_t i = 0; i < ta.poses.size(); ++i)
  {
    smallest = std::min(smallest,
        ta.poses[i].Pos().Distance(tb.poses[i].Pos()));
  }

  // Chassis front to chassis front: 0.4 m between origins.
  EXPECT_GE(smallest, 2 * kFront);
  const double gap = ta.Last().Pos().Distance(tb.Last().Pos());
  EXPECT_NEAR(2 * kFront, gap, 5e-4);

  // They meet in the middle, and are at rest.
  EXPECT_NEAR(1.8, ta.Last().Pos().X(), 0.01);
  EXPECT_NEAR(2.2, tb.Last().Pos().X(), 0.01);
  EXPECT_EQ(ta.poses[6000].Pos().X(), ta.Last().Pos().X());
  EXPECT_EQ(tb.poses[6000].Pos().X(), tb.Last().Pos().X());
  EXPECT_EQ(0.0, ta.leftVelocity.back());
  EXPECT_EQ(0.0, tb.leftVelocity.back());
}

/////////////////////////////////////////////////
/// A planar robot sees a model that the default physics system simulates as
/// an immovable obstacle: it stops in front of it and does not move it.
TEST_F(SimplifiedPhysicsCollisionTest, StopsAtAFullPhysicsBox)
{
  RobotOptions robot;
  robot.name = "dd";
  TestFixture fixture(Config(RobotXml(robot) +
      BoxXml("box", "2 0 0.25 0 0 0")));
  std::map<std::string, Track> tracks;
  Record(fixture, {"dd", "box"}, tracks);
  fixture.Finalize();

  OdometryLog odometry;
  odometry.Subscribe("dd");
  this->Command({"dd"}, 0.5, 0.0);
  fixture.Server()->Run(true, 7000, false);

  const auto &dd = tracks["dd"];
  const auto &box = tracks["box"];
  // Box face at x = 1.75.
  EXPECT_NEAR(1.75 - kFront - kSkin, dd.Last().Pos().X(), 1e-6);
  EXPECT_EQ(dd.poses[6000].Pos().X(), dd.Last().Pos().X());

  // Physics does not see the planar robot, so the box stays where it was,
  // and the robot does not push it.
  EXPECT_NEAR(2.0, box.Last().Pos().X(), 1e-6);
  EXPECT_NEAR(0.0, box.Last().Pos().Y(), 1e-6);
  ExpectOdometryEqualsGroundTruth(dd, odometry);
}

/////////////////////////////////////////////////
/// A model that Physics moves is followed: the planar robot stops at the
/// place the box was moved to, one step after Physics wrote the new pose.
TEST_F(SimplifiedPhysicsCollisionTest, FollowsAFullPhysicsBoxThatMoves)
{
  RobotOptions robot;
  robot.name = "dd";
  TestFixture fixture(Config(RobotXml(robot) +
      BoxXml("box", "2 5 0.25 0 0 0")));
  std::map<std::string, Track> tracks;
  Record(fixture, {"dd", "box"}, tracks);

  Entity box{kNullEntity};
  std::int64_t steps{0};
  fixture.OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        if (box == kNullEntity)
        {
          box = _ecm.EntityByComponents(components::Model(),
              components::Name("box"));
        }
        // Move the box into the lane after one second.
        if (++steps == 1000)
        {
          _ecm.SetComponentData<components::WorldPoseCmd>(box,
              math::Pose3d(2.0, 0.0, 0.25, 0, 0, 0));
        }
      });
  fixture.Finalize();

  this->Command({"dd"}, 0.5, 0.0);
  fixture.Server()->Run(true, 7000, false);

  const auto &dd = tracks["dd"];
  const auto &box2 = tracks["box"];
  EXPECT_NEAR(2.0, box2.Last().Pos().X(), 1e-3);
  EXPECT_NEAR(0.0, box2.Last().Pos().Y(), 1e-3);
  EXPECT_NEAR(1.75 - kFront - kSkin, dd.Last().Pos().X(), 1e-6);
  EXPECT_EQ(dd.poses[6000].Pos().X(), dd.Last().Pos().X());

  // The robot was free until the box arrived, at 0.5 m/s.
  EXPECT_NEAR(0.5 * 1.0, dd.poses[999].Pos().X(), 1e-2);
}

/////////////////////////////////////////////////
/// A wall and a robot that appear while the world runs take part in the
/// collisions at once, and a wall that is removed no longer blocks.
TEST_F(SimplifiedPhysicsCollisionTest, SpawnAndRemoveAtRuntime)
{
  RobotOptions robot;
  robot.name = "first";
  TestFixture fixture(Config(RobotXml(robot)));
  std::map<std::string, Track> tracks;
  Record(fixture, {"first", "late"}, tracks);
  fixture.Finalize();

  this->Command({"first"}, 0.5, 0.0);
  fixture.Server()->Run(true, 1000, false);
  EXPECT_NEAR(0.5, tracks["first"].Last().Pos().X(), 0.01);

  // A wall in the way. The robot is 0.5 m along and the face is at 2.9.
  this->Create(WallXml("wall", 3.0, 0.0, 0.2, 10.0));
  fixture.Server()->Run(true, 6000, false);
  const double stop = 2.9 - kFront - kSkin;
  EXPECT_NEAR(stop, tracks["first"].Last().Pos().X(), 1e-6);

  // A robot that appears later stops at the same wall.
  RobotOptions late;
  late.name = "late";
  late.pose = "0 3 0.1 0 0 0";
  this->Create(RobotXml(late));
  fixture.Server()->Run(true, 100, false);
  this->Command({"late"}, 0.5, 0.0);
  fixture.Server()->Run(true, 6500, false);
  ASSERT_FALSE(tracks["late"].poses.empty());
  EXPECT_NEAR(stop, tracks["late"].Last().Pos().X(), 1e-6);
  EXPECT_NEAR(3.0, tracks["late"].Last().Pos().Y(), 1e-9);

  // Without the wall both drive on.
  this->Remove("wall");
  fixture.Server()->Run(true, 2000, false);
  EXPECT_GT(tracks["first"].Last().Pos().X(), stop + 0.9);
  EXPECT_GT(tracks["late"].Last().Pos().X(), stop + 0.9);
}

/////////////////////////////////////////////////
/// After a world reset the robot starts again, and the wall still stops it.
TEST_F(SimplifiedPhysicsCollisionTest, Reset)
{
  RobotOptions robot;
  robot.name = "dd";
  TestFixture fixture(Config(RobotXml(robot) +
      WallXml("wall", 3.0, 0.0, 0.2, 10.0)));
  std::map<std::string, Track> tracks;
  Record(fixture, {"dd"}, tracks);
  fixture.Finalize();

  this->Command({"dd"}, 0.5, 0.0);
  fixture.Server()->Run(true, 7000, false);
  const double stop = 2.9 - kFront - kSkin;
  EXPECT_NEAR(stop, tracks["dd"].Last().Pos().X(), 1e-6);

  msgs::WorldControl req;
  msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  transport::Node node;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/collision_test/control", req, 1000, rep,
      result));
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());

  // DiffDrive does not drive again after a reset, so it gets a command.
  fixture.Server()->Run(true, 100, false);
  EXPECT_LT(tracks["dd"].Last().Pos().X(), 0.1);
  this->Command({"dd"}, 0.5, 0.0);
  fixture.Server()->Run(true, 7000, false);
  EXPECT_NEAR(stop, tracks["dd"].Last().Pos().X(), 1e-6);
  EXPECT_NEAR(0.0, tracks["dd"].Last().Pos().Y(), 1e-9);
}

/////////////////////////////////////////////////
/// A model whose collisions have a zero collide_bitmask is not in the
/// collision world: it drives through the wall, and costs nothing there.
TEST_F(SimplifiedPhysicsCollisionTest, ZeroBitmaskIsNotAnObstacleOrAnObject)
{
  RobotOptions ghost;
  ghost.name = "ghost";
  ghost.ghost = true;
  RobotOptions solid;
  solid.name = "solid";
  solid.pose = "0 3 0.1 0 0 0";
  TestFixture fixture(Config(RobotXml(ghost) + RobotXml(solid) +
      WallXml("wall", 3.0, 1.5, 0.2, 10.0)));
  std::map<std::string, Track> tracks;
  Record(fixture, {"ghost", "solid"}, tracks);
  fixture.Finalize();

  this->Command({"ghost", "solid"}, 0.5, 0.0);
  fixture.Server()->Run(true, 7000, false);
  EXPECT_NEAR(3.5, tracks["ghost"].Last().Pos().X(), 0.01);
  EXPECT_NEAR(2.9 - kFront - kSkin, tracks["solid"].Last().Pos().X(), 1e-6);
}

/////////////////////////////////////////////////
/// A teleport moves the robot in the collision world too: it drives past
/// the end of a short wall from where it was put, and not into the wall that
/// it would have met from its old place. Put back in front of the wall, it
/// stops at it.
TEST_F(SimplifiedPhysicsCollisionTest, TeleportMovesTheRobotInThatWorld)
{
  RobotOptions robot;
  robot.name = "dd";
  TestFixture fixture(Config(RobotXml(robot) +
      WallXml("wall", 3.0, 0.0, 0.2, 2.0)));
  std::map<std::string, Track> tracks;
  Record(fixture, {"dd"}, tracks);

  Entity model{kNullEntity};
  std::int64_t steps{0};
  fixture.OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        if (model == kNullEntity)
        {
          model = _ecm.EntityByComponents(components::Model(),
              components::Name("dd"));
        }
        if (++steps == 500)
        {
          _ecm.SetComponentData<components::WorldPoseCmd>(model,
              math::Pose3d(1.0, 2.0, 0.1, 0, 0, 0));
        }
        else if (steps == 6001)
        {
          _ecm.SetComponentData<components::WorldPoseCmd>(model,
              math::Pose3d(1.0, 0.0, 0.1, 0, 0, 0));
        }
      });
  fixture.Finalize();

  this->Command({"dd"}, 0.5, 0.0);
  fixture.Server()->Run(true, 5500, false);

  // The wall ends at y = 1, and the robot with its wheels is 0.44 m wide, so
  // at y = 2 it clears the wall and drives on past x = 3.
  const auto &track = tracks["dd"];
  EXPECT_NEAR(2.0, track.Last().Pos().Y(), 1e-9);
  EXPECT_NEAR(1.0 + 5.0 * 0.5, track.Last().Pos().X(), 0.02);

  fixture.Server()->Run(true, 4500, false);
  EXPECT_NEAR(0.0, track.Last().Pos().Y(), 1e-9);
  EXPECT_NEAR(2.9 - kFront - kSkin, track.Last().Pos().X(), 1e-6);
}
