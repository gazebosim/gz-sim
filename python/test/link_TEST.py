#!/usr/bin/env python3
# Copyright (C) 2023 Open Source Robotics Foundation

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

from gz.common import set_verbosity
from gz_test_deps.sim import TestFixture, Link, Model, World, world_entity
from gz_test_deps.math import Matrix3d, Vector3d

post_iterations = 0
iterations = 0
pre_iterations = 0

class TestModel(unittest.TestCase):

    def test_model(self):
        set_verbosity(4)

        file_path = os.path.dirname(os.path.realpath(__file__))
        fixture = TestFixture(os.path.join(file_path, 'link_test.sdf'))

        def on_post_udpate_cb(_info, _ecm):
            global post_iterations
            post_iterations += 1

        def on_pre_udpate_cb(_info, _ecm):
            global pre_iterations
            pre_iterations += 1
            world_e = world_entity(_ecm)
            self.assertEqual(1, world_e)
            w = World(world_e)
            m = Model(w.model_by_name(_ecm, 'model_test'))
            link = Link(m.link_by_name(_ecm, 'link_test'))
            # Entity Test
            self.assertEqual(5, link.entity())
            # Valid Test
            self.assertTrue(link.valid(_ecm))
            # Name Test
            self.assertEqual('link_test', link.name(_ecm))
            # Parent Model Test
            self.assertEqual(m.entity(), link.parent_model(_ecm).entity())
            # Canonical test
            self.assertTrue(link.is_canonical(_ecm))
            # Wind Mode test
            self.assertTrue(link.wind_mode(_ecm))
            # Collisions Test
            self.assertEqual(7, link.collision_by_name(_ecm, 'collision_test'))
            self.assertEqual(1, link.collision_count(_ecm))
            # Visuals Test
            self.assertEqual(6, link.visual_by_name(_ecm, 'visual_test'))
            self.assertEqual(1, link.visual_count(_ecm))
            # World Pose Test
            self.assertEqual(None, link.world_pose(_ecm))
            self.assertEqual(None, link.world_inertial_pose(_ecm))
            # World Velocity Test
            self.assertEqual(None, link.world_linear_velocity(_ecm))
            self.assertEqual(None, link.world_angular_velocity(_ecm))
            link.enable_velocity_checks(_ecm, True)
            link.set_linear_velocity(_ecm, Vector3d())
            link.set_angular_velocity(_ecm, Vector3d())
            self.assertEqual(Vector3d(), link.world_linear_velocity(_ecm))
            self.assertEqual(Vector3d(), link.world_angular_velocity(_ecm))
            # World Acceleration Test
            self.assertEqual(None, link.world_linear_acceleration(_ecm))
            self.assertEqual(None, link.world_angular_acceleration(_ecm))
            link.enable_acceleration_checks(_ecm, True)
            self.assertEqual(Vector3d(), link.world_linear_acceleration(_ecm))
            self.assertEqual(Vector3d(), link.world_angular_acceleration(_ecm))
            # World Inertia Matrix Test
            self.assertEqual(Matrix3d(1,0,0,0,1,0,0,0,1), link.world_inertia_matrix(_ecm))
            # World Kinetic Energy Test
            self.assertEqual(0, link.world_kinetic_energy(_ecm))
            link.enable_velocity_checks(_ecm, False)
            link.enable_acceleration_checks(_ecm, False)


        def on_udpate_cb(_info, _ecm):
            global iterations
            iterations += 1

        fixture.on_post_update(on_post_udpate_cb)
        fixture.on_update(on_udpate_cb)
        fixture.on_pre_update(on_pre_udpate_cb)
        fixture.finalize()

        server = fixture.server()
        server.run(True, 1000, False)

        self.assertEqual(1000, pre_iterations)
        self.assertEqual(1000, iterations)
        self.assertEqual(1000, post_iterations)

if __name__ == '__main__':
    unittest.main()
