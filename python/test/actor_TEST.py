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
import datetime
import unittest

from gz_test_deps.common import set_verbosity
from gz_test_deps.sim import (Actor, K_NULL_ENTITY,
                              TestFixture, World, world_entity)
from gz_test_deps.math import Pose3d


class TestActor(unittest.TestCase):
    post_iterations = 0
    iterations = 0
    pre_iterations = 0

    def test_model(self):
        set_verbosity(4)

        file_path = os.path.dirname(os.path.realpath(__file__))
        fixture = TestFixture(os.path.join(file_path, 'actor_test.sdf'))

        def on_post_udpate_cb(_info, _ecm):
            self.post_iterations += 1

        def on_pre_udpate_cb(_info, _ecm):
            self.pre_iterations += 1
            world_e = world_entity(_ecm)
            self.assertNotEqual(K_NULL_ENTITY, world_e)
            w = World(world_e)
            actor = Actor(w.actor_by_name(_ecm, 'actor_test'))
            # Entity Test
            self.assertNotEqual(K_NULL_ENTITY, actor.entity())
            # Valid Test
            self.assertTrue(actor.valid(_ecm))
            # Name Test
            self.assertEqual('actor_test', actor.name(_ecm))
            # Pose Test
            self.assertEqual(Pose3d(1, 1, 0, 0, 0, 0), actor.pose(_ecm))
            # Trajectory Pose Test
            if self.pre_iterations == 0:
                self.assertEqual(None, actor.trajectory_pose(_ecm))
            actor.set_trajectory_pose(_ecm, Pose3d(2, 2, 0, 0, 0, 0))
            self.assertEqual(Pose3d(2, 2, 0, 0, 0, 0),
                             actor.trajectory_pose(_ecm))
            # World Pose Test
            # The entity doesn't have a components::WorldPose component,
            # therefore, it will return None.
            self.assertEqual(None, actor.world_pose(_ecm))
            # Animation Name Test
            if self.pre_iterations == 0:
                self.assertEqual(None, actor.animation_name(_ecm))
            actor.set_animation_name(_ecm, 'walking_test')
            self.assertEqual('walking_test', actor.animation_name(_ecm))
            # Animation Time Test
            if self.pre_iterations == 0:
                self.assertEqual(None, actor.animation_time(_ecm))
            actor.set_animation_time(_ecm,
                                     datetime.timedelta(milliseconds=100))
            self.assertEqual(100,
                             actor.animation_time(_ecm).total_seconds()*1000)

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
