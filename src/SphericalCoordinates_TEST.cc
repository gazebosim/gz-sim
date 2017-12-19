/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "ignition/math/SphericalCoordinates.hh"

using namespace ignition;

//////////////////////////////////////////////////
// Test different constructors, default parameters
TEST(SphericalCoordinatesTest, Constructor)
{
  // Default surface type
  math::SphericalCoordinates::SurfaceType st =
    math::SphericalCoordinates::EARTH_WGS84;

  // No arguments, default parameters
  {
    math::SphericalCoordinates sc;
    EXPECT_EQ(sc.Surface(), st);
    EXPECT_EQ(sc.LatitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.LongitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.HeadingOffset(), ignition::math::Angle());
    EXPECT_NEAR(sc.ElevationReference(), 0.0, 1e-6);
  }

  // SurfaceType argument, default parameters
  {
    math::SphericalCoordinates sc(st);
    EXPECT_EQ(sc.Surface(), st);
    EXPECT_EQ(sc.LatitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.LongitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.HeadingOffset(), ignition::math::Angle());
    EXPECT_NEAR(sc.ElevationReference(), 0.0, 1e-6);
  }

  // All arguments
  {
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    math::SphericalCoordinates sc(st, lat, lon, elev, heading);
    EXPECT_EQ(sc.Surface(), st);
    EXPECT_EQ(sc.LatitudeReference(), lat);
    EXPECT_EQ(sc.LongitudeReference(), lon);
    EXPECT_EQ(sc.HeadingOffset(), heading);
    EXPECT_NEAR(sc.ElevationReference(), elev, 1e-6);

    // Copy constructor
    math::SphericalCoordinates sc2(sc);
    EXPECT_EQ(sc, sc2);
  }
}

//////////////////////////////////////////////////
// SurfaceType Convert function
TEST(SphericalCoordinatesTest, Convert)
{
  // Default surface type
  math::SphericalCoordinates::SurfaceType st =
    math::SphericalCoordinates::EARTH_WGS84;

  EXPECT_EQ(math::SphericalCoordinates::Convert("EARTH_WGS84"), st);

  EXPECT_EQ(math::SphericalCoordinates::EARTH_WGS84,
            math::SphericalCoordinates::Convert("OTHER-COORD"));
}

//////////////////////////////////////////////////
// Test Set functions
TEST(SphericalCoordinatesTest, SetFunctions)
{
  // Default surface type
  math::SphericalCoordinates::SurfaceType st =
    math::SphericalCoordinates::EARTH_WGS84;

  // Default parameters
  math::SphericalCoordinates sc;
  EXPECT_EQ(sc.Surface(), st);
  EXPECT_EQ(sc.LatitudeReference(), ignition::math::Angle());
  EXPECT_EQ(sc.LongitudeReference(), ignition::math::Angle());
  EXPECT_EQ(sc.HeadingOffset(), ignition::math::Angle());
  EXPECT_NEAR(sc.ElevationReference(), 0.0, 1e-6);

  {
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    sc.SetSurface(st);
    sc.SetLatitudeReference(lat);
    sc.SetLongitudeReference(lon);
    sc.SetHeadingOffset(heading);
    sc.SetElevationReference(elev);

    EXPECT_EQ(sc.Surface(), st);
    EXPECT_EQ(sc.LatitudeReference(), lat);
    EXPECT_EQ(sc.LongitudeReference(), lon);
    EXPECT_EQ(sc.HeadingOffset(), heading);
    EXPECT_NEAR(sc.ElevationReference(), elev, 1e-6);
  }
}

//////////////////////////////////////////////////
// Test coordinate transformations
TEST(SphericalCoordinatesTest, CoordinateTransforms)
{
  // Default surface type
  math::SphericalCoordinates::SurfaceType st =
    math::SphericalCoordinates::EARTH_WGS84;

  {
    // Parameters
    ignition::math::Angle lat(0.3), lon(-1.2),
      heading(ignition::math::Angle::HalfPi);
    double elev = 354.1;
    math::SphericalCoordinates sc(st, lat, lon, elev, heading);

    // Check GlobalFromLocal with heading offset of 90 degrees
    {
      // local frame
      ignition::math::Vector3d xyz;
      // east, north, up
      ignition::math::Vector3d enu;

      xyz.Set(1, 0, 0);
      enu = sc.GlobalFromLocalVelocity(xyz);
      EXPECT_NEAR(enu.Y(), xyz.X(), 1e-6);
      EXPECT_NEAR(enu.X(), -xyz.Y(), 1e-6);
      EXPECT_EQ(xyz, sc.LocalFromGlobalVelocity(enu));

      xyz.Set(0, 1, 0);
      enu = sc.GlobalFromLocalVelocity(xyz);
      EXPECT_NEAR(enu.Y(), xyz.X(), 1e-6);
      EXPECT_NEAR(enu.X(), -xyz.Y(), 1e-6);
      EXPECT_EQ(xyz, sc.LocalFromGlobalVelocity(enu));

      xyz.Set(1, -1, 0);
      enu = sc.GlobalFromLocalVelocity(xyz);
      EXPECT_NEAR(enu.Y(), xyz.X(), 1e-6);
      EXPECT_NEAR(enu.X(), -xyz.Y(), 1e-6);
      EXPECT_EQ(xyz, sc.LocalFromGlobalVelocity(enu));

      xyz.Set(2243.52334, 556.35, 435.6553);
      enu = sc.GlobalFromLocalVelocity(xyz);
      EXPECT_NEAR(enu.Y(), xyz.X(), 1e-6);
      EXPECT_NEAR(enu.X(), -xyz.Y(), 1e-6);
      EXPECT_EQ(xyz, sc.LocalFromGlobalVelocity(enu));
    }

    // Check SphericalFromLocal
    {
      // local frame
      ignition::math::Vector3d xyz;
      // spherical coordinates
      ignition::math::Vector3d sph;

      // No offset
      xyz.Set(0, 0, 0);
      sph = sc.SphericalFromLocalPosition(xyz);
      // latitude
      EXPECT_NEAR(sph.X(), lat.Degree(), 1e-6);
      // longitude
      EXPECT_NEAR(sph.Y(), lon.Degree(), 1e-6);
      // elevation
      EXPECT_NEAR(sph.Z(), elev, 1e-6);

      // 200 km offset in x (pi/2 heading offset means North). We use
      // SphericalFromLocal, which means that xyz is a linear movement on
      // a plane (not along the curvature of Earth). This will result in
      // a large height offset.
      xyz.Set(2e5, 0, 0);
      sph = sc.SphericalFromLocalPosition(xyz);
      // increase in latitude about 1.8 degrees
      EXPECT_NEAR(sph.X(), lat.Degree() + 1.8, 0.008);
      // no change in longitude
      EXPECT_NEAR(sph.Z(), 3507.024791, 1e-6);

      ignition::math::Vector3d xyz2 = sc.LocalFromSphericalPosition(sph);
      EXPECT_EQ(xyz, xyz2);
    }

    // Check position projection
    {
      // WGS84 coordinate obtained from online mapping software
      // > gdaltransform -s_srs WGS84 -t_srs EPSG:4978
      // > latitude longitude altitude
      // > X Y Z
      ignition::math::Vector3d tmp;
      ignition::math::Vector3d osrf_s(37.3877349, -122.0651166, 32.0);
      ignition::math::Vector3d osrf_e(
          -2693701.91434394, -4299942.14687992, 3851691.0393571);
      ignition::math::Vector3d goog_s(37.4216719, -122.0821853, 30.0);
      ignition::math::Vector3d goog_e(
          -2693766.71906146, -4297199.59926038, 3854681.81878812);

      // Local tangent plane coordinates (ENU = GLOBAL) coordinates of
      // Google when OSRF is taken as the origin:
      // > proj +ellps=WGS84  +proj=tmerc
      // +lat_0=37.3877349 +lon_0=-122.0651166 +k=1 +x_0=0 +y_0=0
      // > -122.0821853 37.4216719 (LON,LAT)
      // > -1510.88 3766.64 (EAST,NORTH)
      ignition::math::Vector3d vec(-1510.88, 3766.64, -3.29);

      // Convert degrees to radians
      osrf_s.X() *= 0.0174532925;
      osrf_s.Y() *= 0.0174532925;

      // Set the ORIGIN to be the Open Source Robotics Foundation
      math::SphericalCoordinates sc2(st, ignition::math::Angle(osrf_s.X()),
          ignition::math::Angle(osrf_s.Y()), osrf_s.Z(),
          ignition::math::Angle::Zero);

      // Check that SPHERICAL -> ECEF works
      tmp = sc2.PositionTransform(osrf_s,
          math::SphericalCoordinates::SPHERICAL,
          math::SphericalCoordinates::ECEF);

      EXPECT_NEAR(tmp.X(), osrf_e.X(), 8e-2);
      EXPECT_NEAR(tmp.Y(), osrf_e.Y(), 8e-2);
      EXPECT_NEAR(tmp.Z(), osrf_e.Z(), 1e-2);

      // Check that ECEF -> SPHERICAL works
      tmp = sc2.PositionTransform(tmp,
          math::SphericalCoordinates::ECEF,
          math::SphericalCoordinates::SPHERICAL);

      EXPECT_NEAR(tmp.X(), osrf_s.X(), 1e-2);
      EXPECT_NEAR(tmp.Y(), osrf_s.Y(), 1e-2);
      EXPECT_NEAR(tmp.Z(), osrf_s.Z(), 1e-2);

      // Check that SPHERICAL -> LOCAL works
      tmp = sc2.LocalFromSphericalPosition(goog_s);
      EXPECT_NEAR(tmp.X(), vec.X(), 8e-2);
      EXPECT_NEAR(tmp.Y(), vec.Y(), 8e-2);
      EXPECT_NEAR(tmp.Z(), vec.Z(), 1e-2);

      // Check that SPHERICAL -> LOCAL -> SPHERICAL works
      tmp = sc2.SphericalFromLocalPosition(tmp);
      EXPECT_NEAR(tmp.X(), goog_s.X(), 8e-2);
      EXPECT_NEAR(tmp.Y(), goog_s.Y(), 8e-2);
      EXPECT_NEAR(tmp.Z(), goog_s.Z(), 1e-2);
    }
  }
}

//////////////////////////////////////////////////
// Test distance
TEST(SphericalCoordinatesTest, Distance)
{
  ignition::math::Angle latA, longA, latB, longB;
  latA.Degree(46.250944);
  longA.Degree(-122.249972);
  latB.Degree(46.124953);
  longB.Degree(-122.251683);
  double d = math::SphericalCoordinates::Distance(latA, longA, latB, longB);

  EXPECT_NEAR(14002, d, 20);
}

//////////////////////////////////////////////////
TEST(SphericalCoordinatesTest, BadSetSurface)
{
  math::SphericalCoordinates sc;
  sc.SetSurface(static_cast<math::SphericalCoordinates::SurfaceType>(2));
  EXPECT_EQ(sc.Surface(), 2);
}

//////////////////////////////////////////////////
TEST(SphericalCoordinatesTest, Transform)
{
  math::SphericalCoordinates sc;
  math::Vector3d vel(1, 2, -4);
  math::Vector3d result = sc.VelocityTransform(vel,
      math::SphericalCoordinates::ECEF,
      math::SphericalCoordinates::ECEF);

  EXPECT_EQ(result, vel);

  math::Vector3d pos(-1510.88, 2, -4);
  result = sc.PositionTransform(pos,
      math::SphericalCoordinates::ECEF,
      math::SphericalCoordinates::GLOBAL);

  EXPECT_NEAR(result.X(), 2, 1e-6);
  EXPECT_NEAR(result.Y(), -4, 1e-6);
  EXPECT_NEAR(result.Z(), -6379647.8799999999, 1e-6);

  std::cout << "NEW POS[" << result << "]\n";
}

//////////////////////////////////////////////////
TEST(SphericalCoordinatesTest, BadCoordinateType)
{
  math::SphericalCoordinates sc;
  math::Vector3d pos(1, 2, -4);
  math::Vector3d result = sc.PositionTransform(pos,
      static_cast<math::SphericalCoordinates::CoordinateType>(5),
      static_cast<math::SphericalCoordinates::CoordinateType>(6));

  EXPECT_EQ(result, pos);

  result = sc.PositionTransform(pos,
      static_cast<math::SphericalCoordinates::CoordinateType>(4),
      static_cast<math::SphericalCoordinates::CoordinateType>(6));

  EXPECT_EQ(result, pos);

  result = sc.VelocityTransform(pos,
      math::SphericalCoordinates::SPHERICAL,
      math::SphericalCoordinates::ECEF);
  EXPECT_EQ(result, pos);

  result = sc.VelocityTransform(pos,
      math::SphericalCoordinates::ECEF,
      math::SphericalCoordinates::SPHERICAL);
  EXPECT_EQ(result, pos);

  result = sc.VelocityTransform(pos,
      static_cast<math::SphericalCoordinates::CoordinateType>(5),
      math::SphericalCoordinates::ECEF);
  EXPECT_EQ(result, pos);

  result = sc.VelocityTransform(pos,
      math::SphericalCoordinates::ECEF,
      static_cast<math::SphericalCoordinates::CoordinateType>(5));
  EXPECT_EQ(result, pos);
}

//////////////////////////////////////////////////
// Test [in]equality operators.
TEST(SphericalCoordinatesTest, EqualityOps)
{
  // Default surface type
  math::SphericalCoordinates::SurfaceType st =
    math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  math::SphericalCoordinates sc1(st, lat, lon, elev, heading);

  math::SphericalCoordinates sc2(st, lat, lon, elev, heading);
  EXPECT_TRUE(sc1 == sc2);
  EXPECT_FALSE(sc1 != sc2);
  math::SphericalCoordinates sc3(st, ignition::math::Angle::Zero, lon, elev,
    heading);
  EXPECT_FALSE(sc1 == sc3);
  EXPECT_TRUE(sc1 != sc3);
  math::SphericalCoordinates sc4(st, lat, ignition::math::Angle::Zero, elev,
    heading);
  EXPECT_FALSE(sc1 == sc4);
  EXPECT_TRUE(sc1 != sc4);
  math::SphericalCoordinates sc5(st, lat, lon, elev + 1, heading);
  EXPECT_FALSE(sc1 == sc5);
  EXPECT_TRUE(sc1 != sc5);
  math::SphericalCoordinates sc6(st, lat, lon, elev,
    ignition::math::Angle::Zero);
  EXPECT_FALSE(sc1 == sc6);
  EXPECT_TRUE(sc1 != sc6);
}

//////////////////////////////////////////////////
// Test assignment operator.
TEST(SphericalCoordinatesTest, AssignmentOp)
{
  // Default surface type
  math::SphericalCoordinates::SurfaceType st =
    math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  math::SphericalCoordinates sc1(st, lat, lon, elev, heading);

  math::SphericalCoordinates sc2 = sc1;
  EXPECT_EQ(sc1, sc2);
}
