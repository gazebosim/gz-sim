# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import ignition
from ignition.math import Angle, SphericalCoordinates, Vector3d
import math

class TestSphericalCoordinates(unittest.TestCase):

    def test_constructor(self):
        # Default surface type
        st = SphericalCoordinates.EARTH_WGS84

        # No arguments, default parameters
        sc = SphericalCoordinates()
        self.assertEqual(sc.surface(), st)
        self.assertEqual(sc.latitude_reference(), Angle())
        self.assertEqual(sc.longitude_reference(), Angle())
        self.assertEqual(sc.heading_offset(), Angle())
        self.assertAlmostEqual(sc.elevation_reference(), 0.0, delta=1e-6)

        # SurfaceType argument, default parameters
        sc = SphericalCoordinates(st)
        self.assertEqual(sc.surface(), st)
        self.assertEqual(sc.latitude_reference(), Angle())
        self.assertEqual(sc.longitude_reference(), Angle())
        self.assertEqual(sc.heading_offset(), Angle())
        self.assertAlmostEqual(sc.elevation_reference(), 0.0, delta=1e-6)

        # All arguments
        lat = Angle(0.3)
        lon = Angle(-1.2)
        heading = Angle(0.5)
        elev = 354.1
        sc = SphericalCoordinates(st, lat, lon, elev, heading)
        self.assertEqual(sc.surface(), st)
        self.assertEqual(sc.latitude_reference(), lat)
        self.assertEqual(sc.longitude_reference(), lon)
        self.assertEqual(sc.heading_offset(), heading)
        self.assertAlmostEqual(sc.elevation_reference(), elev, delta=1e-6)

        # Copy constructor
        sc2 = SphericalCoordinates(sc)
        self.assertEqual(sc, sc2)

    def test_convert(self):
        # Default surface type
        st = SphericalCoordinates.EARTH_WGS84

        self.assertEqual(SphericalCoordinates.convert("EARTH_WGS84"), st)

        self.assertEqual(SphericalCoordinates.EARTH_WGS84,
                         SphericalCoordinates.convert("OTHER-COORD"))

        self.assertEqual("EARTH_WGS84", SphericalCoordinates.convert(st))

    def test_set_functions(self):
        # Default surface type
        st = SphericalCoordinates.EARTH_WGS84

        # Default parameters
        sc = SphericalCoordinates()
        self.assertEqual(sc.surface(), st)
        self.assertEqual(sc.latitude_reference(), Angle())
        self.assertEqual(sc.longitude_reference(), Angle())
        self.assertEqual(sc.heading_offset(), Angle())
        self.assertAlmostEqual(sc.elevation_reference(), 0.0, delta=1e-6)

        lat = Angle(0.3)
        lon = Angle(-1.2)
        heading = Angle(0.5)
        elev = 354.1
        sc.set_surface(st)
        sc.set_latitude_reference(lat)
        sc.set_longitude_reference(lon)
        sc.set_heading_offset(heading)
        sc.set_elevation_reference(elev)

        self.assertEqual(sc.surface(), st)
        self.assertEqual(sc.latitude_reference(), lat)
        self.assertEqual(sc.longitude_reference(), lon)
        self.assertEqual(sc.heading_offset(), heading)
        self.assertAlmostEqual(sc.elevation_reference(), elev, delta=1e-6)

    def test_coordinate_transforms(self):
        # Default surface type
        st = SphericalCoordinates.EARTH_WGS84

        # Parameters
        lat = Angle(0.3)
        lon = Angle(-1.2)
        heading = Angle(Angle.HALF_PI)
        elev = 354.1
        sc = SphericalCoordinates(st, lat, lon, elev, heading)

        # Check GlobalFromLocal with heading offset of 90 degrees
        # Heading 0:  X == East, Y == North, Z == Up
        # Heading 90: X == North, Y == West , Z == Up
        # local frame
        xyz = Vector3d()
        # east, north, up
        enu = Vector3d()

        xyz.set(1, 0, 0)
        enu = sc.global_from_local_velocity(xyz)
        self.assertAlmostEqual(enu.y(), xyz.x(), delta=1e-6)
        self.assertAlmostEqual(enu.x(), -xyz.y(), delta=1e-6)
        self.assertEqual(xyz, sc.local_from_global_velocity(enu))

        xyz.set(0, 1, 0)
        enu = sc.global_from_local_velocity(xyz)
        self.assertAlmostEqual(enu.y(), xyz.x(), delta=1e-6)
        self.assertAlmostEqual(enu.x(), -xyz.y(), delta=1e-6)
        self.assertEqual(xyz, sc.local_from_global_velocity(enu))

        xyz.set(1, -1, 0)
        enu = sc.global_from_local_velocity(xyz)
        self.assertAlmostEqual(enu.y(), xyz.x(), delta=1e-6)
        self.assertAlmostEqual(enu.x(), -xyz.y(), delta=1e-6)
        self.assertEqual(xyz, sc.local_from_global_velocity(enu))

        xyz.set(2243.52334, 556.35, 435.6553)
        enu = sc.global_from_local_velocity(xyz)
        self.assertAlmostEqual(enu.y(), xyz.x(), delta=1e-6)
        self.assertAlmostEqual(enu.x(), -xyz.y(), delta=1e-6)
        self.assertEqual(xyz, sc.local_from_global_velocity(enu))

        # Check SphericalFromLocal
        # local frame
        xyz = Vector3d()
        # spherical coordinates
        sph = Vector3d()

        # No offset
        xyz.set(0, 0, 0)
        sph = sc.spherical_from_local_position(xyz)
        # latitude
        self.assertAlmostEqual(sph.x(), lat.degree(), delta=1e-6)
        # longitude
        self.assertAlmostEqual(sph.y(), lon.degree(), delta=1e-6)
        # elevation
        self.assertAlmostEqual(sph.z(), elev, delta=1e-6)

        # 200 km offset in x (pi/2 heading offset means North). We use
        # SphericalFromLocal, which means that xyz is a linear movement on
        # a plane (not along the curvature of Earth). This will result in
        # a large height offset.
        xyz.set(2e5, 0, 0)
        sph = sc.spherical_from_local_position(xyz)
        # increase in latitude about 1.8 degrees
        self.assertAlmostEqual(sph.x(), lat.degree() + 1.8, delta=0.008)
        # no change in longitude
        self.assertAlmostEqual(sph.z(), 3507.024791, delta=1e-6)

        xyz2 = sc.local_from_spherical_position(sph)
        self.assertEqual(xyz, xyz2)

        # Check position projection
        # WGS84 coordinate obtained from online mapping software
        # > gdaltransform -s_srs WGS84 -t_srs EPSG:4978
        # > latitude longitude altitude
        # > X Y Z
        tmp = Vector3d()
        osrf_s = Vector3d(37.3877349, -122.0651166, 32.0)
        osrf_e = Vector3d(-2693701.91434394, -4299942.14687992, 3851691.0393571)
        goog_s = Vector3d(37.4216719, -122.0821853, 30.0)

        # Local tangent plane coordinates (ENU = GLOBAL) coordinates of
        # Google when OSRF is taken as the origin:
        # > proj +ellps=WGS84  +proj=tmerc
        # +lat_0=37.3877349 +lon_0=-122.0651166 +k=1 +x_0=0 +y_0=0
        # > -122.0821853 37.4216719 (LON,LAT)
        # > -1510.88 3766.64 (EAST,NORTH)
        vec = Vector3d(-1510.88, 3766.64, -3.29)

        # Convert degrees to radians
        osrf_s.x(osrf_s.x() * 0.0174532925)
        osrf_s.y(osrf_s.y() * 0.0174532925)

        # Set the ORIGIN to be the Open Source Robotics Foundation
        sc2 = SphericalCoordinates(st, Angle(osrf_s.x()),
                                   Angle(osrf_s.y()), osrf_s.z(), Angle.ZERO)

        # Check that SPHERICAL -> ECEF works
        tmp = sc2.position_transform(osrf_s, SphericalCoordinates.SPHERICAL,
                                     SphericalCoordinates.ECEF)

        self.assertAlmostEqual(tmp.x(), osrf_e.x(), delta=8e-2)
        self.assertAlmostEqual(tmp.y(), osrf_e.y(), delta=8e-2)
        self.assertAlmostEqual(tmp.z(), osrf_e.z(), delta=1e-2)

        # Check that ECEF -> SPHERICAL works
        tmp = sc2.position_transform(tmp, SphericalCoordinates.ECEF, SphericalCoordinates.SPHERICAL)

        self.assertAlmostEqual(tmp.x(), osrf_s.x(), delta=1e-2)
        self.assertAlmostEqual(tmp.y(), osrf_s.y(), delta=1e-2)
        self.assertAlmostEqual(tmp.z(), osrf_s.z(), delta=1e-2)

        # Check that SPHERICAL -> LOCAL works
        tmp = sc2.local_from_spherical_position(goog_s)
        self.assertAlmostEqual(tmp.x(), vec.x(), delta=8e-2)
        self.assertAlmostEqual(tmp.y(), vec.y(), delta=8e-2)
        self.assertAlmostEqual(tmp.z(), vec.z(), delta=1e-2)

        # Check that SPHERICAL -> LOCAL -> SPHERICAL works
        tmp = sc2.spherical_from_local_position(tmp)
        self.assertAlmostEqual(tmp.x(), goog_s.x(), delta=8e-2)
        self.assertAlmostEqual(tmp.y(), goog_s.y(), delta=8e-2)
        self.assertAlmostEqual(tmp.z(), goog_s.z(), delta=1e-2)

        # Give no heading offset to confirm ENU frame
        lat = Angle(0.3)
        lon = Angle(-1.2)
        heading = Angle(0.0)
        elev = 354.1
        sc = SphericalCoordinates(st, lat, lon, elev, heading)

        # Check GlobalFromLocal with no heading offset
        # local frame
        xyz = Vector3d()
        # east, north, up
        enu = Vector3d()

        xyz.set(1, 0, 0)
        enu = sc.velocity_transform(xyz, SphericalCoordinates.LOCAL2, SphericalCoordinates.GLOBAL)
        self.assertEqual(xyz, enu)
        self.assertEqual(xyz, sc.local_from_global_velocity(enu))

        xyz.set(0, 1, 0)
        enu = sc.velocity_transform(xyz, SphericalCoordinates.LOCAL2, SphericalCoordinates.GLOBAL)
        self.assertEqual(xyz, enu)
        self.assertEqual(xyz, sc.local_from_global_velocity(enu))

        xyz.set(1, -1, 0)
        enu = sc.velocity_transform(xyz, SphericalCoordinates.LOCAL2, SphericalCoordinates.GLOBAL)
        self.assertEqual(xyz, enu)
        self.assertEqual(xyz, sc.local_from_global_velocity(enu))

        xyz.set(2243.52334, 556.35, 435.6553)
        enu = sc.velocity_transform(xyz, SphericalCoordinates.LOCAL2, SphericalCoordinates.GLOBAL)
        self.assertEqual(xyz, enu)
        self.assertEqual(xyz, sc.local_from_global_velocity(enu))

    def test_distance(self):
        latA = Angle()
        longA = Angle()
        latB = Angle()
        longB = Angle()
        latA.set_degree(46.250944)
        longA.set_degree(-122.249972)
        latB.set_degree(46.124953)
        longB.set_degree(-122.251683)
        d = SphericalCoordinates.distance(latA, longA, latB, longB)

        self.assertAlmostEqual(14002, d, delta=20)

    def test_bad_set_surface(self):
        sc = SphericalCoordinates()
        sc.set_surface(SphericalCoordinates.SurfaceType(2))
        self.assertEqual(sc.surface(), SphericalCoordinates.SurfaceType(2))

    def test_transform(self):
        sc = SphericalCoordinates()
        vel = Vector3d(1, 2, -4)
        result = sc.velocity_transform(
            vel,
            SphericalCoordinates.ECEF,
            SphericalCoordinates.ECEF)

        self.assertEqual(result, vel)

        pos = Vector3d(-1510.88, 2, -4)
        result = sc.position_transform(
            pos,
            SphericalCoordinates.ECEF,
            SphericalCoordinates.GLOBAL)

        self.assertAlmostEqual(result.x(), 2, delta=1e-6)
        self.assertAlmostEqual(result.y(), -4, delta=1e-6)
        self.assertAlmostEqual(result.z(), -6379647.8799999999, delta=1e-6)

        print('NEW POS[', result.x(), ' ', result.y(), ' ', result.z(), ']\n')

    def test_bad_coordinate_type(self):
        sc = SphericalCoordinates()
        pos = Vector3d(1, 2, -4)
        result = sc.position_transform(pos,
                                       SphericalCoordinates.CoordinateType(7),
                                       SphericalCoordinates.CoordinateType(6))

        self.assertEqual(result, pos)

        result = sc.position_transform(pos,
                                       SphericalCoordinates.CoordinateType(4),
                                       SphericalCoordinates.CoordinateType(6))

        self.assertEqual(result, pos)

        result = sc.velocity_transform(
            pos,
            SphericalCoordinates.SPHERICAL,
            SphericalCoordinates.ECEF)
        self.assertEqual(result, pos)

        result = sc.velocity_transform(
            pos,
            SphericalCoordinates.ECEF,
            SphericalCoordinates.SPHERICAL)
        self.assertEqual(result, pos)

        result = sc.velocity_transform(pos,
                                       SphericalCoordinates.CoordinateType(7),
                                       SphericalCoordinates.ECEF)
        self.assertEqual(result, pos)

        result = sc.velocity_transform(pos,
                                       SphericalCoordinates.ECEF,
                                       SphericalCoordinates.CoordinateType(7))
        self.assertEqual(result, pos)

    def test_equality_ops(self):
        # Default surface type
        st = SphericalCoordinates.EARTH_WGS84
        lat = Angle(0.3)
        lon = Angle(-1.2)
        heading = Angle(0.5)
        elev = 354.1
        sc1 = SphericalCoordinates(st, lat, lon, elev, heading)

        sc2 = SphericalCoordinates(st, lat, lon, elev, heading)
        self.assertTrue(sc1 == sc2)
        self.assertFalse(sc1 != sc2)
        sc3 = SphericalCoordinates(st, Angle.ZERO, lon, elev, heading)
        self.assertFalse(sc1 == sc3)
        self.assertTrue(sc1 != sc3)
        sc4 = SphericalCoordinates(st, lat, Angle.ZERO, elev, heading)
        self.assertFalse(sc1 == sc4)
        self.assertTrue(sc1 != sc4)
        sc5 = SphericalCoordinates(st, lat, lon, elev + 1, heading)
        self.assertFalse(sc1 == sc5)
        self.assertTrue(sc1 != sc5)
        sc6 = SphericalCoordinates(st, lat, lon, elev, Angle.ZERO)
        self.assertFalse(sc1 == sc6)
        self.assertTrue(sc1 != sc6)

    def test_assigment_op(self):
        # Default surface type
        st = SphericalCoordinates.EARTH_WGS84
        lat = Angle(0.3)
        lon = Angle(-1.2)
        heading = Angle(0.5)
        elev = 354.1
        sc1 = SphericalCoordinates(st, lat, lon, elev, heading)

        sc2 = sc1
        self.assertEqual(sc1, sc2)

    def test_no_heading(self):
        # Default heading
        st = SphericalCoordinates.EARTH_WGS84
        lat = Angle(-22.9 * math.pi / 180.0)
        lon = Angle(-43.2 * math.pi / 180.0)
        heading = Angle(0.0)
        elev = 0
        sc = SphericalCoordinates(st, lat, lon, elev, heading)

        # Origin matches input
        latLonAlt = sc.spherical_from_local_position(Vector3d(0, 0, 0))
        self.assertEqual(lat.degree(), latLonAlt.x())
        self.assertEqual(lon.degree(), latLonAlt.y())
        self.assertEqual(elev, latLonAlt.z())

        xyzOrigin = sc.local_from_spherical_position(latLonAlt)
        self.assertEqual(Vector3d.ZERO, xyzOrigin)

        # Check how different lat/lon affect the local position

        # Increase latitude == go North == go +Y
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree() + 1.0, lon.degree(), elev))
        self.assertAlmostEqual(xyzOrigin.x(), xyz.x(), delta=1e-6)
        self.assertLess(xyzOrigin.y(), xyz.y())

        # Decrease latitude == go South == go -Y
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree() - 1.0, lon.degree(), elev))
        self.assertAlmostEqual(xyzOrigin.x(), xyz.x(), delta=1e-6)
        self.assertGreater(xyzOrigin.y(), xyz.y())

        # Increase longitude == go East == go +X
        # Also move a bit -Y because this is the Southern Hemisphere
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree(), lon.degree() + 1.0, elev))
        self.assertLess(xyzOrigin.x(), xyz.x())
        self.assertGreater(xyzOrigin.y(), xyz.y())

        # Decrease longitude == go West == go -X
        # Also move a bit -Y because this is the Southern Hemisphere
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree(), lon.degree() - 1.0, elev))
        self.assertGreater(xyzOrigin.x(), xyz.x())
        self.assertGreater(xyzOrigin.y(), xyz.y())

        # Increase altitude
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree(), lon.degree(), elev + 10.0))
        self.assertAlmostEqual(xyzOrigin.x(), xyz.x(), delta=1e-6)
        self.assertAlmostEqual(xyzOrigin.y(), xyz.y(), delta=1e-6)
        self.assertAlmostEqual(xyzOrigin.z() + 10.0, xyz.z(), delta=1e-6)

        # Decrease altitude
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree(), lon.degree(), elev - 10.0))
        self.assertAlmostEqual(xyzOrigin.x(), xyz.x(), delta=1e-6)
        self.assertAlmostEqual(xyzOrigin.y(), xyz.y(), delta=1e-6)
        self.assertAlmostEqual(xyzOrigin.z() - 10.0, xyz.z(), delta=1e-6)

        # Check how global and local velocities are connected

        # Velocity in
        # +X (East), +Y (North), -X (West), -Y (South), +Z (up), -Z (down)
        for global_var in [Vector3d.UNIT_X, Vector3d.UNIT_Y, Vector3d.UNIT_Z,
                           -Vector3d.UNIT_X, -Vector3d.UNIT_Y, -Vector3d.UNIT_Z]:
            local = sc.local_from_global_velocity(global_var)
            self.assertEqual(global_var, local)

            # This function is broken for horizontal velocities
            global_var = sc.global_from_local_velocity(local)
            if abs(global_var.z()) < 0.1:
                self.assertNotEqual(global_var, local)
            else:
                self.assertEqual(global_var, local)

            # Directly call fixed version
            global_var = sc.velocity_transform(
                local,
                SphericalCoordinates.LOCAL2,
                SphericalCoordinates.GLOBAL)
            self.assertEqual(global_var, local)

    def test_with_heading(self):
        # Heading 90 deg: X == North, Y == West , Z == Up
        st = SphericalCoordinates.EARTH_WGS84
        lat = Angle(-22.9 * math.pi / 180.0)
        lon = Angle(-43.2 * math.pi / 180.0)
        heading = Angle(90.0 * math.pi / 180.0)
        elev = 0
        sc = SphericalCoordinates(st, lat, lon, elev, heading)

        # Origin matches input
        latLonAlt = sc.spherical_from_local_position(Vector3d(0, 0, 0))
        self.assertEqual(lat.degree(), latLonAlt.x())
        self.assertEqual(lon.degree(), latLonAlt.y())
        self.assertEqual(elev, latLonAlt.z())

        xyzOrigin = sc.local_from_spherical_position(latLonAlt)
        self.assertEqual(Vector3d.ZERO, xyzOrigin)

        # Check how different lat/lon affect the local position

        # Increase latitude == go North == go +X
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree() + 1.0, lon.degree(), elev))
        self.assertAlmostEqual(xyzOrigin.y(), xyz.y(), delta=1e-6)
        self.assertLess(xyzOrigin.x(), xyz.x())

        # Decrease latitude == go South == go -X
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree() - 1.0, lon.degree(), elev))
        self.assertAlmostEqual(xyzOrigin.y(), xyz.y(), delta=1e-6)
        self.assertGreater(xyzOrigin.x(), xyz.x())

        # Increase longitude == go East == go -Y (and a bit -X)
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree(), lon.degree() + 1.0, elev))
        self.assertGreater(xyzOrigin.y(), xyz.y())
        self.assertGreater(xyzOrigin.x(), xyz.x())

        # Decrease longitude == go West == go +Y (and a bit -X)
        xyz = sc.local_from_spherical_position(
            Vector3d(lat.degree(), lon.degree() - 1.0, elev))
        self.assertLess(xyzOrigin.y(), xyz.y())
        self.assertGreater(xyzOrigin.x(), xyz.x())

        # Check how global and local velocities are connected

        # Global     | Local
        # ---------- | ------
        # +X (East)  | -Y
        # -X (West)  | +Y
        # +Y (North) | +X
        # -Y (South) | -X
        globalLocal = [
            [Vector3d.UNIT_X, -Vector3d.UNIT_Y],
            [-Vector3d.UNIT_X, Vector3d.UNIT_Y],
            [Vector3d.UNIT_Y, Vector3d.UNIT_X],
            [-Vector3d.UNIT_Y, -Vector3d.UNIT_X]]
        for [global_var, local] in globalLocal:
            localRes = sc.local_from_global_velocity(global_var)
            self.assertEqual(local, localRes)

            # Directly call fixed version
            globalRes = sc.velocity_transform(
                local,
                SphericalCoordinates.LOCAL2,
                SphericalCoordinates.GLOBAL)
            self.assertEqual(global_var, globalRes)

    def test_inverse(self):
        st = SphericalCoordinates.EARTH_WGS84
        lat = Angle(0.3)
        lon = Angle(-1.2)
        heading = Angle(0.5)
        elev = 354.1
        sc = SphericalCoordinates(st, lat, lon, elev, heading)

        # GLOBAL <-> LOCAL2
        in_vector = Vector3d(1, 2, -4)
        out = sc.velocity_transform(
            in_vector,
            SphericalCoordinates.LOCAL2,
            SphericalCoordinates.GLOBAL)
        self.assertNotEqual(in_vector, out)
        reverse = sc.velocity_transform(
            out,
            SphericalCoordinates.GLOBAL,
            SphericalCoordinates.LOCAL2)
        self.assertEqual(in_vector, reverse)

        in_vector = Vector3d(1, 2, -4)
        out = sc.position_transform(
            in_vector,
            SphericalCoordinates.LOCAL2,
            SphericalCoordinates.GLOBAL)
        self.assertNotEqual(in_vector, out)
        reverse = sc.position_transform(
            out,
            SphericalCoordinates.GLOBAL,
            SphericalCoordinates.LOCAL2)
        self.assertEqual(in_vector, reverse)

        # SPHERICAL <-> LOCAL2
        in_vector = Vector3d(1, 2, -4)
        out = sc.position_transform(
            in_vector,
            SphericalCoordinates.LOCAL2,
            SphericalCoordinates.SPHERICAL)
        self.assertNotEqual(in_vector, out)
        reverse = sc.position_transform(
            out,
            SphericalCoordinates.SPHERICAL,
            SphericalCoordinates.LOCAL2)
        self.assertEqual(in_vector, reverse)


if __name__ == '__main__':
    unittest.main()
