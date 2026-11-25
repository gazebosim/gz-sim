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

#include <cmath>
#include <memory>
#include <string>

#include <gz/math/Vector3.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class TrajectoryFollowerTest : public InternalFixture<::testing::Test>
{
};

namespace
{
/// \brief Check the first wrench for the given inertial rotation and waypoint.
void ExpectFirstWrench(const std::string &_inertialRpy,
    const std::string &_waypoint, const math::Vector3d &_expectedForce,
    const math::Vector3d &_expectedTorque)
{
  SCOPED_TRACE(_inertialRpy);

  // The link has a 45-degree roll and a 90-degree yaw in the world.
  ServerConfig config;
  config.SetSdfString(R"(
<sdf version="1.9">
  <world name="trajectory_follower">
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <model name="vehicle">
      <link name="body">
        <pose>0 0 0 0.7853981633974483 0 1.5707963267948966</pose>
        <inertial>
          <pose>0 0 0 )" + _inertialRpy + R"(</pose>
          <mass>1</mass>
        </inertial>
      </link>
      <plugin filename="gz-sim-trajectory-follower-system"
              name="gz::sim::systems::TrajectoryFollower">
        <link_name>body</link_name>
        <force>10</force>
        <torque>6</torque>
        <waypoints>
          <waypoint>)" + _waypoint + R"(</waypoint>
        </waypoints>
      </plugin>
    </model>
  </world>
</sdf>)");

  Entity link = kNullEntity;
  bool observed = false;
  TestFixture fixture(config);
  fixture.OnConfigure([&](const Entity &_world,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &_ecm, EventManager &)
  {
    const World world(_world);
    const Model vehicle(world.ModelByName(_ecm, "vehicle"));
    link = vehicle.LinkByName(_ecm, "body");
    ASSERT_NE(kNullEntity, link);

    // Seed WorldPose so the first step has the SDF orientation before Physics
    // updates the component.
    _ecm.SetComponentData<components::WorldPose>(link, worldPose(link, _ecm));
  });
  fixture.OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
  {
    // The follower runs before this hook. Physics clears the wrench in Update.
    const auto *wrench =
        _ecm.Component<components::ExternalWorldWrenchCmd>(link);
    ASSERT_NE(nullptr, wrench);
    observed = true;

    const auto &force = wrench->Data().force();
    const auto &torque = wrench->Data().torque();
    constexpr double tolerance = 1e-9;
    EXPECT_NEAR(_expectedForce.X(), force.x(), tolerance);
    EXPECT_NEAR(_expectedForce.Y(), force.y(), tolerance);
    EXPECT_NEAR(_expectedForce.Z(), force.z(), tolerance);
    EXPECT_NEAR(_expectedTorque.X(), torque.x(), tolerance);
    EXPECT_NEAR(_expectedTorque.Y(), torque.y(), tolerance);
    EXPECT_NEAR(_expectedTorque.Z(), torque.z(), tolerance);
  });
  fixture.Finalize();

  ASSERT_TRUE(fixture.Server()->Run(true, 1, false));
  EXPECT_TRUE(observed);
}
}  // namespace

/////////////////////////////////////////////////
TEST_F(TrajectoryFollowerTest, FirstAlignedWrench)
{
  // Link +X points along world +Y.
  const math::Vector3d force(0, 10, 0);
  ExpectFirstWrench("0 0 0", "0 8", force, math::Vector3d::Zero);
  ExpectFirstWrench("0 0 1.5707963267948966", "0 8", force,
      math::Vector3d::Zero);
}

/////////////////////////////////////////////////
TEST_F(TrajectoryFollowerTest, FirstTurningWrench)
{
  // The waypoint is to the right. Link -Z projects onto world -X and -Z.
  const double torqueComponent = -6.0 / std::sqrt(2.0);
  const math::Vector3d torque(torqueComponent, 0, torqueComponent);
  ExpectFirstWrench("0 0 0", "8 0", math::Vector3d::Zero, torque);
  ExpectFirstWrench("1.5707963267948966 0 0", "8 0", math::Vector3d::Zero,
      torque);
}
