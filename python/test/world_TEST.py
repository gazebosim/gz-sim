#!/usr/bin/env python3
# Copyright (C) 2021 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest

from gz_test_deps.common import set_verbosity
from gz_test_deps.sim import K_NULL_ENTITY, TestFixture, World, world_entity
from gz_test_deps.math import SphericalCoordinates, Vector3d
from gz_test_deps.sdformat import Atmosphere

class TestWorld(unittest.TestCase):
    post_iterations = 0
    iterations = 0
    pre_iterations = 0

    def test_world(self):
        set_verbosity(4)

        file_path = os.path.dirname(os.path.realpath(__file__))
        fixture = TestFixture(os.path.join(file_path, 'world_test.sdf'))

        def on_post_udpate_cb(_info, _ecm):
            self.post_iterations += 1

        def on_pre_udpate_cb(_info, _ecm):
            self.pre_iterations += 1
            world_e = world_entity(_ecm)
            self.assertNotEqual(K_NULL_ENTITY, world_e)
            world = World(world_e)
            # Valid Test
            self.assertTrue(world.valid(_ecm))
            # Name Test
            self.assertEqual('world_test', world.name(_ecm))
            # Gravity Test
            self.assertEqual(Vector3d(0, 0, -10), world.gravity(_ecm))
            # Magnetic Field Test
            self.assertEqual(Vector3d(1, 2, 3), world.magnetic_field(_ecm))
            # Atmosphere Test
            atmosphere = world.atmosphere(_ecm)
            self.assertEqual(300, atmosphere.temperature())
            self.assertEqual(100000, atmosphere.pressure())
            self.assertEqual(-0.005, atmosphere.temperature_gradient())
            # Spherical Coordinates Test
            self.assertEqual(SphericalCoordinates.SurfaceType.EARTH_WGS84, world.spherical_coordinates(_ecm).surface())
            if self.pre_iterations <= 1:
                self.assertAlmostEqual(float(10), world.spherical_coordinates(_ecm).latitude_reference().degree())
                self.assertAlmostEqual(float(15), world.spherical_coordinates(_ecm).longitude_reference().degree())
                self.assertAlmostEqual(float(20), world.spherical_coordinates(_ecm).elevation_reference())
                self.assertAlmostEqual(float(25), world.spherical_coordinates(_ecm).heading_offset().degree())
                world.set_spherical_coordinates(_ecm, SphericalCoordinates())
            else:
                self.assertAlmostEqual(float(0), world.spherical_coordinates(_ecm).latitude_reference().degree())
                self.assertAlmostEqual(float(0), world.spherical_coordinates(_ecm).longitude_reference().degree())
                self.assertAlmostEqual(float(0), world.spherical_coordinates(_ecm).elevation_reference())
                self.assertAlmostEqual(float(0), world.spherical_coordinates(_ecm).heading_offset().degree())
            # Light Test
            self.assertNotEqual(K_NULL_ENTITY, world.light_by_name(_ecm, 'light_point_test'))
            self.assertEqual(1, world.light_count(_ecm))
            # Actor test
            self.assertNotEqual(K_NULL_ENTITY, world.actor_by_name(_ecm, 'actor_test'))
            self.assertEqual(1, world.actor_count(_ecm))
            # Model Test
            self.assertNotEqual(K_NULL_ENTITY, world.model_by_name(_ecm, 'model_test'))
            self.assertEqual(1, world.model_count(_ecm))

        def on_udpate_cb(_info, _ecm):
            self.iterations += 1

        fixture.on_post_update(on_post_udpate_cb)
        fixture.on_update(on_udpate_cb)
        fixture.on_pre_update(on_pre_udpate_cb)
        fixture.finalize()

        server = fixture.server()
        server.run(True, 2, False)

        self.assertEqual(2, self.pre_iterations)
        self.assertEqual(2, self.iterations)
        self.assertEqual(2, self.post_iterations)

if __name__ == '__main__':
    unittest.main()
