#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
from datetime import timedelta
from gz.sim import Server, ServerConfig

class TestServer(unittest.TestCase):
    def test_server_ecm(self):
        config = ServerConfig()
        server = Server(config)

        # Initial state: 3 default entities (world, etc.)
        self.assertEqual(3, server.entity_count())

        # Test peek_ecm
        def peek_cb(ecm):
            self.assertGreaterEqual(ecm.entity_count(), 3)
        server.peek_ecm(peek_cb)

        # Test poke_ecm (create an entity)
        def poke_cb(ecm):
            ecm.create_entity()
        server.poke_ecm(poke_cb)

        # Check that entity was created
        self.assertEqual(4, server.entity_count())

    def test_server_stats(self):
        config = ServerConfig()
        server = Server(config)

        self.assertEqual(0, server.iteration_count())
        self.assertEqual(timedelta(0), server.sim_time())
        self.assertEqual(3, server.entity_count())
        self.assertEqual(2, server.system_count())

if __name__ == '__main__':
    unittest.main()
