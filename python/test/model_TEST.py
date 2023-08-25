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

from gz_test_deps.common import set_verbosity
from gz_test_deps.sim import K_NULL_ENTITY, TestFixture, Model, World, world_entity

class TestModel(unittest.TestCase):
    post_iterations = 0
    iterations = 0
    pre_iterations = 0

    def test_model(self):
        set_verbosity(4)

        file_path = os.path.dirname(os.path.realpath(__file__))
        fixture = TestFixture(os.path.join(file_path, 'model_test.sdf'))

        def on_post_udpate_cb(_info, _ecm):
            self.post_iterations += 1

        def on_pre_udpate_cb(_info, _ecm):
            self.pre_iterations += 1
            world_e = world_entity(_ecm)
            self.assertNotEqual(K_NULL_ENTITY, world_e)
            w = World(world_e)
            model = Model(w.model_by_name(_ecm, 'model_test'))
            # Entity Test
            self.assertNotEqual(K_NULL_ENTITY, model.entity())
            # Valid Test
            self.assertTrue(model.valid(_ecm))
            # Name Test
            self.assertEqual('model_test', model.name(_ecm))
            # Static Test
            self.assertTrue(model.static(_ecm))
            # Self Collide Test
            self.assertTrue(model.self_collide(_ecm))
            # Wind Mode Test
            self.assertTrue(model.wind_mode(_ecm))
            # Get Joints Test
            self.assertNotEqual(K_NULL_ENTITY, model.joint_by_name(_ecm, 'model_joint_test'))
            self.assertEqual(1, model.joint_count(_ecm))
            # Get Links Test
            self.assertNotEqual(K_NULL_ENTITY, model.link_by_name(_ecm, 'model_link_test_1'))
            self.assertNotEqual(K_NULL_ENTITY, model.link_by_name(_ecm, 'model_link_test_2'))
            self.assertEqual(2, model.link_count(_ecm))

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
