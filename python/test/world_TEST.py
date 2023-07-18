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

import unittest

from gz.sim8 import EntityComponentManager, World, world_entity
from gz.math7 import SphericalCoordinates


class WorldTEST(unittest.TestCase):
    def setUp(self):
        self.ecm = EntityComponentManager()
        self.world_e = world_entity(self.ecm)
        self.world = World(self.world_e)

    def test_world_entity(self):
        self.assertEqual(self.world_e, self.world.entity())

    def test_world_valid(self):
        self.assertFalse(self.world.valid(self.ecm))
        # Missing a way to set up the world component in the entity

    def test_world_name(self):
        self.assertEqual(None, self.world.name(self.ecm))
        # Missing a way to set up the name for the world

    def test_world_gravity(self):
        self.assertEqual(None, self.world.gravity(self.ecm))
        # Missing a way to set up the gravity for the world

    def test_world_atmosphere(self):
        self.assertEqual(None, self.world.atmosphere(self.ecm))
        # Missing a way to set up the atmosphere for the world

    def test_world_magnetic_field(self):
        self.assertEqual(None, self.world.magnetic_field(self.ecm))
        # Missing a way to set up the magnetic field for the world

    def test_world_spherical_coordinates(self):
        self.assertEqual(None, self.world.spherical_coordinates(self.ecm))
        self.world.set_spherical_coordinates(self.ecm, SphericalCoordinates())
        # This assertion should be `self.assertEqual(SphericalCoordinates(), self.world.spherical_coordinates(self.ecm))`
        self.assertEqual(None, self.world.spherical_coordinates(self.ecm))
