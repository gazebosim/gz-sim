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
from gz.sim8 import Light, TestFixture, World, world_entity
from gz.math7 import Angle, Color, Pose3d, Vector3d

post_iterations = 0
iterations = 0
pre_iterations = 0

class TestLight(unittest.TestCase):

    def test_light(self):
        set_verbosity(4)

        file_path = os.path.dirname(os.path.realpath(__file__))
        fixture = TestFixture(os.path.join(file_path, 'light_test.sdf'))

        def on_post_udpate_cb(_info, _ecm):
            global post_iterations
            post_iterations += 1

        def on_pre_udpate_cb(_info, _ecm):
            global pre_iterations
            pre_iterations += 1
            world_e = world_entity(_ecm)
            self.assertEqual(1, world_e)
            w = World(world_e)
            light = Light(w.light_by_name(_ecm, 'light_test'))
            # Entity Test
            self.assertEqual(4, light.entity())
            # Valid Test
            self.assertTrue(light.valid(_ecm))
            # Name Test
            self.assertEqual('light_test', light.name(_ecm))
            # Pose Test
            self.assertEqual(Pose3d(0, 2, 2, 0, 0, 0), light.pose(_ecm))
            light.set_pose(_ecm, Pose3d(4, 2, 2, 0, 0, 0))
            self.assertEqual(Pose3d(0, 2, 2, 0, 0, 0), light.pose(_ecm))
            # Type Test
            self.assertEqual('point', light.type(_ecm))
            # Diffuse Color Test
            self.assertEqual(Color(1, 0, 0, 1), light.diffuse_color(_ecm))
            light.set_diffuse_color(_ecm, Color(1, 1, 0, 1))
            self.assertEqual(Color(1, 0, 0, 1), light.diffuse_color(_ecm))
            # Specular Color Test
            self.assertEqual(Color(0.2, 0, 0, 1), light.specular_color(_ecm))
            light.set_specular_color(_ecm, Color(0.2, 0.2, 0, 1))
            self.assertEqual(Color(0.2, 0, 0, 1), light.specular_color(_ecm))
            # Cast Shadows Test
            self.assertTrue(light.cast_shadows(_ecm))
            light.set_cast_shadows(_ecm, False)
            self.assertTrue(light.cast_shadows(_ecm))
            # Intensity Test
            self.assertEqual(2, light.intensity(_ecm))
            light.set_intensity(_ecm, 5)
            self.assertEqual(2, light.intensity(_ecm))
            # Direction Test
            self.assertEqual(Vector3d(0, 0, -1), light.direction(_ecm))
            light.set_direction(_ecm, Vector3d(1, 0, -1))
            self.assertEqual(Vector3d(0, 0, -1), light.direction(_ecm))
            # Attenuation Range Test
            self.assertEqual(20, light.attenuation_range(_ecm))
            light.set_attenuation_range(_ecm, 30)
            self.assertEqual(20, light.attenuation_range(_ecm))
            # Attenuation Constant Test
            self.assertEqual(0.8, light.attenuation_constant(_ecm))
            light.set_attenuation_constant(_ecm, 1.2)
            self.assertEqual(0.8, light.attenuation_constant(_ecm))
            # Attenuation Linear Test
            self.assertEqual(0.2, light.attenuation_linear(_ecm))
            light.set_attenuation_linear(_ecm, 0.5)
            self.assertEqual(0.2, light.attenuation_linear(_ecm))
            # Attenuation Quadratric Test
            self.assertEqual(0.01, light.attenuation_quadratic(_ecm))
            light.set_attenuation_quadratic(_ecm, 0.05)
            self.assertEqual(0.01, light.attenuation_quadratic(_ecm))
            # Spot Inner Angle Test
            self.assertEqual(Angle(), light.spot_inner_angle(_ecm))
            light.set_spot_inner_angle(_ecm, Angle(10))
            self.assertEqual(Angle(), light.spot_inner_angle(_ecm))
            # Spot Outer Angle Test
            self.assertEqual(Angle(), light.spot_outer_angle(_ecm))
            light.set_spot_outer_angle(_ecm, Angle(5))
            self.assertEqual(Angle(), light.spot_outer_angle(_ecm))
            # Spot Falloff Test
            self.assertEqual(0, light.spot_falloff(_ecm))
            light.set_spot_falloff(_ecm, 2)
            self.assertEqual(0, light.spot_falloff(_ecm))

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
