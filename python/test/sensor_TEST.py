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
from gz.sim8 import TestFixture, Joint, Model, Sensor, World, world_entity
from gz.math7 import Pose3d

post_iterations = 0
iterations = 0
pre_iterations = 0

class TestSensor(unittest.TestCase):

    def test_model(self):
        set_verbosity(4)

        file_path = os.path.dirname(os.path.realpath(__file__))
        fixture = TestFixture(os.path.join(file_path, 'joint_test.sdf'))

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
            j = Joint(m.joint_by_name(_ecm, 'joint_test'))
            sensor = Sensor(j.sensor_by_name(_ecm, 'sensor_test'))
            # Entity Test
            self.assertEqual(8, sensor.entity())
            # Valid Test
            self.assertTrue(sensor.valid(_ecm))
            # Name Test
            self.assertEqual('sensor_test', sensor.name(_ecm))
            # Pose Test
            self.assertEqual(Pose3d(0, 1, 0, 0, 0, 0), sensor.pose(_ecm))
            # Topic Test
            # This is None because the entity does not have a components::SensorTopic component.
            self.assertEqual(None, sensor.topic(_ecm))
            # Parent Test
            self.assertEqual(j.entity(), sensor.parent(_ecm))

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
