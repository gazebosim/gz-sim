#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import datetime
import unittest

from gz.sim import EntityComponentManager, Server, ServerConfig, UpdateInfo


class ServerTest(unittest.TestCase):

    def test_ecm_context_manager_read(self):
        config = ServerConfig()
        server = Server(config)
        self.assertFalse(server.is_running())
        self.assertEqual(3, server.entity_count(0))

        with server.ecm() as ecm:
            self.assertIsInstance(ecm, EntityComponentManager)
            self.assertEqual(3, ecm.entity_count())
            self.assertTrue(ecm.has_entity(1))

    def test_ecm_context_manager_write(self):
        config = ServerConfig()
        server = Server(config)
        self.assertEqual(3, server.entity_count(0))

        with server.ecm() as ecm:
            self.assertIsInstance(ecm, EntityComponentManager)
            e = ecm.create_entity()
            self.assertTrue(ecm.has_entity(e))

        # Confirm entity count reflects the mutation
        self.assertEqual(4, server.entity_count(0))
        with server.ecm() as ecm:
            self.assertEqual(4, ecm.entity_count())
            self.assertTrue(ecm.has_entity(e))

    def test_guard_exception_safety(self):
        config = ServerConfig()
        server = Server(config)

        # Verify exception is propagated (not swallowed by __exit__)
        with self.assertRaises(RuntimeError):
            with server.ecm() as ecm:
                ecm.create_entity()
                raise RuntimeError('Test exception inside context manager')

        # Verify lock was released and subsequent operations work
        with server.ecm() as ecm:
            self.assertIsInstance(ecm, EntityComponentManager)

    def test_invalid_runner_id(self):
        config = ServerConfig()
        server = Server(config)

        # Out of bounds runner ID raises ValueError when entered
        with self.assertRaises(ValueError):
            with server.ecm(999):
                pass

    def test_server_statistics(self):
        config = ServerConfig()
        server = Server(config)

        self.assertEqual(0, server.iteration_count(0))
        self.assertEqual(3, server.entity_count(0))
        self.assertEqual(3, server.system_count(0))

        info = server.current_info(0)
        self.assertIsInstance(info, UpdateInfo)
        self.assertEqual(0, info.iterations)
        self.assertEqual(datetime.timedelta(0), info.sim_time)
        self.assertTrue(info.paused)

        self.assertIsNone(server.current_info(999))

        # Step simulation
        self.assertTrue(server.run(True, 10, False))
        self.assertEqual(10, server.iteration_count(0))
        info_after = server.current_info(0)
        self.assertIsInstance(info_after, UpdateInfo)
        self.assertEqual(10, info_after.iterations)
        self.assertGreater(info_after.sim_time, datetime.timedelta(0))

if __name__ == '__main__':
    unittest.main()
