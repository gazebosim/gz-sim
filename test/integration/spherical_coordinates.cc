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

#include "ignition/gazebo/TestFixture.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/World.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/SphericalCoordinates.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

#define tol 10e-4

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Test SphericalCoordinates system
class SphericalCoordinatesTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(SphericalCoordinatesTest, InitialFromSDF)
{
  TestFixture fixture(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/spherical_coordinates.sdf");

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

      World world(entity);
      EXPECT_TRUE(world.SphericalCoordinates(_ecm));
      latest = world.SphericalCoordinates(_ecm).value();

      iterations++;
    }).Finalize();

  int expectedIterations{10};
  fixture.Server()->Run(true, expectedIterations, false);

  // Check values from SDF
  EXPECT_DOUBLE_EQ(expectedIterations, iterations);
  EXPECT_DOUBLE_EQ(math::SphericalCoordinates::EARTH_WGS84, latest.Surface());
  EXPECT_DOUBLE_EQ(-22.9, latest.LatitudeReference().Degree());
  EXPECT_DOUBLE_EQ(-43.2, latest.LongitudeReference().Degree());
  EXPECT_DOUBLE_EQ(0.0, latest.ElevationReference());
  EXPECT_DOUBLE_EQ(0.0, latest.HeadingOffset().Degree());
}

/////////////////////////////////////////////////
TEST_F(SphericalCoordinatesTest, SetWorldOriginFromTransport)
{
  TestFixture fixture(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/spherical_coordinates.sdf");

  int iterations{0};
  math::SphericalCoordinates latest;
  fixture.OnPostUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto entity = worldEntity(_ecm);
      EXPECT_NE(kNullEntity, entity);

      World world(entity);
      EXPECT_TRUE(world.SphericalCoordinates(_ecm));
      latest = world.SphericalCoordinates(_ecm).value();

      iterations++;
    }).Finalize();

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
  int expectedIterations{0};
  for (; latest.LatitudeReference().Degree() < 1.0 && sleep < maxSleep; sleep++)
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
}

/////////////////////////////////////////////////
TEST_F(SphericalCoordinatesTest, SetWorldOriginFromComponent)
{
  TestFixture fixture(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/spherical_coordinates.sdf");

  int iterations{0};
  math::SphericalCoordinates latest;
  fixture.OnPostUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto entity = worldEntity(_ecm);
      EXPECT_NE(kNullEntity, entity);

      World world(entity);
      EXPECT_TRUE(world.SphericalCoordinates(_ecm));
      latest = world.SphericalCoordinates(_ecm).value();

      iterations++;
    }).Finalize();

  // Set throught C++ API and check
  fixture.OnPreUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto entity = worldEntity(_ecm);
      EXPECT_NE(kNullEntity, entity);

      World world(entity);
      world.SetSphericalCoordinates(_ecm, math::SphericalCoordinates(
          math::SphericalCoordinates::EARTH_WGS84, IGN_DTOR(52.2),
          IGN_DTOR(21.0), 789.0, 0));
    });

  fixture.Server()->Run(true, 1, false);
  EXPECT_DOUBLE_EQ(1, iterations);
  EXPECT_DOUBLE_EQ(math::SphericalCoordinates::EARTH_WGS84, latest.Surface());
  EXPECT_DOUBLE_EQ(52.2, latest.LatitudeReference().Degree());
  EXPECT_DOUBLE_EQ(21.0, latest.LongitudeReference().Degree());
  EXPECT_DOUBLE_EQ(789.0, latest.ElevationReference());
  EXPECT_DOUBLE_EQ(0.0, latest.HeadingOffset().Degree());
}

/////////////////////////////////////////////////
TEST_F(SphericalCoordinatesTest, MoveEntity)
{
  TestFixture fixture(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/spherical_coordinates.sdf");

  int iterations{0};
  Entity modelEntity{kNullEntity};
  math::SphericalCoordinates worldLatLon;
  math::Vector3d modelLatLon;
  fixture.OnPostUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager &_ecm)
    {
      World world(worldEntity(_ecm));

      EXPECT_TRUE(world.SphericalCoordinates(_ecm));
      worldLatLon = world.SphericalCoordinates(_ecm).value();

      modelEntity = _ecm.EntityByComponents(components::Model(),
          components::Name("north"));
      auto modelCoord = sphericalCoordinates(modelEntity, _ecm);
      EXPECT_TRUE(modelCoord);
      modelLatLon = modelCoord.value();

      iterations++;
    }).Finalize();

  // An entity on +Y (North) has higher Latitude than the origin
  fixture.Server()->Run(true, 1, false);
  EXPECT_NE(kNullEntity, modelEntity);
  EXPECT_GT(modelLatLon.X(), worldLatLon.LatitudeReference().Degree());
  EXPECT_DOUBLE_EQ(modelLatLon.Y(), worldLatLon.LongitudeReference().Degree());

  // Move entity through transport and check
  double desiredLat{-23.0};
  double desiredLon{-43.3};
  msgs::SphericalCoordinates req;
  req.set_surface_model(msgs::SphericalCoordinates::EARTH_WGS84);
  req.set_latitude_deg(desiredLat);
  req.set_longitude_deg(desiredLon);
  auto entityMsg = req.mutable_entity();
  entityMsg->set_id(modelEntity);

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
  int expectedIterations{0};
  for (; modelLatLon.X() > worldLatLon.LatitudeReference().Degree()
      && sleep < maxSleep; sleep++)
  {
    fixture.Server()->Run(true, 1, false);
    expectedIterations++;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NE(maxSleep, sleep);

  EXPECT_NEAR(modelLatLon.X(), desiredLat, 1e-6);
  EXPECT_NEAR(modelLatLon.Y(), desiredLon, 1e-6);
}

/////////////////////////////////////////////////
TEST_F(SphericalCoordinatesTest, CreateEntity)
{
  TestFixture fixture(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/spherical_coordinates.sdf");

  int iterations{0};
  Entity modelEntity{kNullEntity};
  math::SphericalCoordinates worldLatLon;
  math::Vector3d modelLatLon;
  fixture.OnPostUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager &_ecm)
    {
      World world(worldEntity(_ecm));

      EXPECT_TRUE(world.SphericalCoordinates(_ecm));
      worldLatLon = world.SphericalCoordinates(_ecm).value();

      // Get model once it's spawned
      modelEntity = _ecm.EntityByComponents(components::Model(),
          components::Name("spawned"));
      if (kNullEntity != modelEntity)
      {
        auto modelCoord = sphericalCoordinates(modelEntity, _ecm);
        EXPECT_TRUE(modelCoord);
        modelLatLon = modelCoord.value();
      }

      iterations++;
    }).Finalize();

  // Create entity at spherical coordinates
  auto modelStr = std::string("<?xml version=\"1.0\" ?>") +
      "<sdf version='1.6'>" +
      "<model name='spawned'>" +
      "<link name='link'>" +
      "</link>" +
      "</model>" +
      "</sdf>";

  double desiredLat{-23.0};
  double desiredLon{-43.3};

  msgs::EntityFactory req;
  req.set_sdf(modelStr);

  auto scMsg = req.mutable_spherical_coordinates();
  scMsg->set_latitude_deg(desiredLat);
  scMsg->set_longitude_deg(desiredLon);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/spherical_coordinates/create"};

  transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  int sleep{0};
  int maxSleep{30};
  int expectedIterations{0};
  for (; kNullEntity == modelEntity && sleep < maxSleep; sleep++)
  {
    fixture.Server()->Run(true, 1, false);
    expectedIterations++;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NE(maxSleep, sleep);
  EXPECT_NE(kNullEntity, modelEntity);
  EXPECT_NEAR(modelLatLon.X(), desiredLat, 1e-6);
  EXPECT_NEAR(modelLatLon.Y(), desiredLon, 1e-6);
}
