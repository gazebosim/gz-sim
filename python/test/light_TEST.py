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

# This class test the Light class API created with pybind11. Since the setters methods
# require other systems to work (eg. Rendering sensors), those methods will not be fully
# tested, just verifying the method is being called.  
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
            light_point = Light(w.light_by_name(_ecm, 'light_point_test'))
            light_directional = Light(w.light_by_name(_ecm, 'light_directional_test'))
            light_spot = Light(w.light_by_name(_ecm, 'light_spot_test'))
            # Entity Test
            self.assertEqual(4, light_point.entity())
            self.assertEqual(6, light_directional.entity())
            self.assertEqual(8, light_spot.entity())
            # Valid Test
            self.assertTrue(light_point.valid(_ecm))
            self.assertTrue(light_directional.valid(_ecm))
            self.assertTrue(light_spot.valid(_ecm))
            # Name Test
            self.assertEqual('light_point_test', light_point.name(_ecm))
            self.assertEqual('light_directional_test', light_directional.name(_ecm))
            self.assertEqual('light_spot_test', light_spot.name(_ecm))
            # Pose Test
            self.assertEqual(Pose3d(0, 2, 2, 0, 0, 0), light_point.pose(_ecm))
            light_point.set_pose(_ecm, Pose3d(4, 2, 2, 0, 0, 0))
            # Type Test
            self.assertEqual('point', light_point.type(_ecm))
            self.assertEqual('directional', light_directional.type(_ecm))
            self.assertEqual('spot', light_spot.type(_ecm))
            ## Diffuse Color Test
            self.assertEqual(Color(1, 0, 0, 1), light_point.diffuse_color(_ecm))
            light_point.set_diffuse_color(_ecm, Color(1, 1, 0, 1))
            # Specular Color Test
            self.assertEqual(Color(0.2, 0, 0, 1), light_point.specular_color(_ecm))
            light_point.set_specular_color(_ecm, Color(0.2, 0.2, 0, 1))
            # Cast Shadows Test
            self.assertTrue(light_point.cast_shadows(_ecm))
            light_point.set_cast_shadows(_ecm, False)
            # Intensity Test
            self.assertEqual(2, light_point.intensity(_ecm))
            light_point.set_intensity(_ecm, 5)
            # Direction Test
            self.assertEqual(Vector3d(0.5, 0.5, -1), light_directional.direction(_ecm))
            light_directional.set_direction(_ecm, Vector3d(1, 0, -1))
            # Attenuation Range Test
            self.assertEqual(20, light_point.attenuation_range(_ecm))
            light_point.set_attenuation_range(_ecm, 30)
            # Attenuation Constant Test
            self.assertEqual(0.8, light_point.attenuation_constant(_ecm))
            light_point.set_attenuation_constant(_ecm, 1.2)
            # Attenuation Linear Test
            self.assertEqual(0.2, light_point.attenuation_linear(_ecm))
            light_point.set_attenuation_linear(_ecm, 0.5)
            # Attenuation Quadratric Test
            self.assertEqual(0.01, light_point.attenuation_quadratic(_ecm))
            light_point.set_attenuation_quadratic(_ecm, 0.05)
            # Spot Inner Angle Test
            self.assertEqual(Angle(10), light_spot.spot_inner_angle(_ecm))
            light_spot.set_spot_inner_angle(_ecm, Angle(15))
            # Spot Outer Angle Test
            self.assertEqual(Angle(5), light_spot.spot_outer_angle(_ecm))
            light_spot.set_spot_outer_angle(_ecm, Angle(10))
            # Spot Falloff Test
            self.assertEqual(2, light_spot.spot_falloff(_ecm))
            light_spot.set_spot_falloff(_ecm, 4)
            
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
