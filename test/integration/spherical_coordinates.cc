/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/common/Console.hh>
#include <ignition/math/SphericalCoordinates.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/SphericalCoordinates.hh"
#include "ignition/gazebo/TestFixture.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/World.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

#define tol 10e-4

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Test SphericalCoordinates system
class SphericalCoordinatesTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    ignition::common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str());
  }
};

/////////////////////////////////////////////////
TEST_F(SphericalCoordinatesTest, Update)
{
  TestFixture fixture(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/spherical_coordinates.sdf");

  // Check value from SDF
  int iterations{0};
  math::SphericalCoordinates latest;
  fixture.OnPostUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto entity = worldEntity(_ecm);
      EXPECT_NE(kNullEntity, entity);

      auto scComp = _ecm.Component<components::SphericalCoordinates>(entity);
      EXPECT_NE(nullptr, scComp);

      World world(worldEntity(_ecm));
      EXPECT_TRUE(world.SphericalCoordinates(_ecm));
      latest = world.SphericalCoordinates(_ecm).value();

      iterations++;
    }).Finalize();

  int expectedIterations{10};
  fixture.Server()->Run(true, expectedIterations, false);
  EXPECT_DOUBLE_EQ(expectedIterations, iterations);
  EXPECT_DOUBLE_EQ(math::SphericalCoordinates::EARTH_WGS84, latest.Surface());
  EXPECT_DOUBLE_EQ(-22.9, latest.LatitudeReference().Degree());
  EXPECT_DOUBLE_EQ(-43.2, latest.LongitudeReference().Degree());
  EXPECT_DOUBLE_EQ(123.0, latest.ElevationReference());
  EXPECT_DOUBLE_EQ(20.0, latest.HeadingOffset().Degree());

  // Set through transport and check
  msgs::SphericalCoordinates req;
  req.set_surface_model(msgs::SphericalCoordinates::EARTH_WGS84);
  req.set_latitude_deg(35.6);
  req.set_longitude_deg(140.1);
  req.set_elevation(456.0);
  req.set_heading_deg(-20.0);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/spherical_coordinates/set_spherical_coordinates"};

  transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  int sleep{0};
  int maxSleep{30};
  for (; latest.LatitudeReference().Degree() < 0 && sleep < maxSleep; sleep++)
  {
    fixture.Server()->Run(true, 1, false);
    expectedIterations++;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NE(maxSleep, sleep);

  EXPECT_DOUBLE_EQ(expectedIterations, iterations);
  EXPECT_DOUBLE_EQ(math::SphericalCoordinates::EARTH_WGS84, latest.Surface());
  EXPECT_DOUBLE_EQ(35.6, latest.LatitudeReference().Degree());
  EXPECT_DOUBLE_EQ(140.1, latest.LongitudeReference().Degree());
  EXPECT_DOUBLE_EQ(456.0, latest.ElevationReference());
  EXPECT_DOUBLE_EQ(-20.0, latest.HeadingOffset().Degree());

  // Set throught C++ API and check
  fixture.OnPreUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto entity = worldEntity(_ecm);
      EXPECT_NE(kNullEntity, entity);

      World world(worldEntity(_ecm));
      world.SetSphericalCoordinates(_ecm, math::SphericalCoordinates(
          math::SphericalCoordinates::EARTH_WGS84, IGN_DTOR(52.2),
          IGN_DTOR(21.0), 789.0, 0));
    });

  fixture.Server()->Run(true, 1, false);
  EXPECT_DOUBLE_EQ(expectedIterations+1, iterations);
  EXPECT_DOUBLE_EQ(math::SphericalCoordinates::EARTH_WGS84, latest.Surface());
  EXPECT_DOUBLE_EQ(52.2, latest.LatitudeReference().Degree());
  EXPECT_DOUBLE_EQ(21.0, latest.LongitudeReference().Degree());
  EXPECT_DOUBLE_EQ(789.0, latest.ElevationReference());
  EXPECT_DOUBLE_EQ(0.0, latest.HeadingOffset().Degree());

  // An entity on +X +Y has higher Latitude and Longitude than the origin
  Entity northEastEntity{kNullEntity};
  fixture.OnPreUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto entity = worldEntity(_ecm);
      EXPECT_NE(kNullEntity, entity);

      northEastEntity = _ecm.CreateEntity();
      _ecm.CreateComponent(northEastEntity,
          components::Pose({10, 10, 0, 0, 0, 0}));
      _ecm.CreateComponent(northEastEntity,
          components::ParentEntity(worldEntity(_ecm)));

      auto northEast = sphericalCoordinates(northEastEntity, _ecm);
      EXPECT_TRUE(northEast);
      EXPECT_GT(52.2, northEast.value().X());
      EXPECT_GT(21.0, northEast.value().Y());
    });
  fixture.Server()->Run(true, 1, false);
  EXPECT_NE(kNullEntity, northEastEntity);
}
