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
import time
import unittest

from ignition.common import set_verbosity
from ignition.gazebo import TestFixture, World, world_entity
from ignition.math import Vector3d
from sdformat import Element

post_iterations = 0
iterations = 0
pre_iterations = 0

class TestTestFixture(unittest.TestCase):

    def test_test_fixture(self):
        set_verbosity(4)

        file_path = os.path.dirname(os.path.realpath(__file__))
        helper = TestFixture(os.path.join(file_path, 'gravity.sdf'))

        def on_post_udpate_cb(_info, _ecm):
            global post_iterations
            post_iterations += 1

        def on_pre_udpate_cb(_info, _ecm):
            global pre_iterations
            pre_iterations += 1
            world_e = world_entity(_ecm);
            self.assertEqual(1, world_e)
            w = World(world_e)
            v = w.gravity(_ecm)
            self.assertEqual(Vector3d(0, 0, -9.8), v)

        def on_udpate_cb(_info, _ecm):
            global iterations
            iterations += 1

        helper.on_post_update(on_post_udpate_cb)
        helper.on_update(on_udpate_cb)
        helper.on_pre_update(on_pre_udpate_cb)
        helper.finalize()

        server = helper.server()
        server.run(False, 1000, False)

        while(server.is_running()):
            time.sleep(0.1)

        self.assertEqual(1000, pre_iterations)
        self.assertEqual(1000, iterations)
        self.assertEqual(1000, post_iterations)

if __name__ == '__main__':
    unittest.main()
