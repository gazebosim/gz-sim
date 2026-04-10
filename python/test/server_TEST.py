import unittest
import os
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
