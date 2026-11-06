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

import unittest

from gz.math import Pose3d
from gz.sim import EntityComponentManager, components


class TestEcmEach(unittest.TestCase):

    def test_each_no_types(self):
        """each_data() with no types yields a 1-tuple per live entity."""
        ecm = EntityComponentManager()
        self.assertEqual([], ecm.each_data())

        entities = {ecm.create_entity() for _ in range(3)}
        # Components must not matter for a zero-argument query.
        ecm.create_component(next(iter(entities)), components.Name, "a")

        rows = ecm.each_data()
        self.assertEqual(3, len(rows))
        for row in rows:
            self.assertIsInstance(row, tuple)
            self.assertEqual(1, len(row))
        self.assertEqual(entities, {row[0] for row in rows})

    def test_each_single_component(self):
        """each_data(Comp) yields (entity, value) for entities with Comp."""
        ecm = EntityComponentManager()
        named = ecm.create_entity()
        ecm.create_component(named, components.Name, "named")
        unnamed = ecm.create_entity()

        rows = ecm.each_data(components.Name)
        self.assertEqual(1, len(rows))
        self.assertEqual(2, len(rows[0]))
        self.assertEqual((named, "named"), rows[0])
        self.assertNotIn(unnamed, [row[0] for row in rows])

    def test_each_multiple_components(self):
        """each_data(A, B) yields values in the order the types were given."""
        ecm = EntityComponentManager()

        both = ecm.create_entity()
        ecm.create_component(both, components.Name, "both")
        ecm.create_component(both, components.Pose, Pose3d(1.0, 2.0, 3.0,
                                                           0.0, 0.0, 0.0))

        name_only = ecm.create_entity()
        ecm.create_component(name_only, components.Name, "name_only")

        pose_only = ecm.create_entity()
        ecm.create_component(pose_only, components.Pose, Pose3d())

        rows = ecm.each_data(components.Name, components.Pose)
        self.assertEqual(1, len(rows))
        entity, name, pose = rows[0]
        self.assertEqual(both, entity)
        self.assertEqual("both", name)
        self.assertAlmostEqual(1.0, pose.x())

        # Reversing the requested types reverses the values.
        rows = ecm.each_data(components.Pose, components.Name)
        self.assertEqual(1, len(rows))
        entity, pose, name = rows[0]
        self.assertEqual(both, entity)
        self.assertEqual("both", name)
        self.assertAlmostEqual(1.0, pose.x())

    def test_each_tag_component(self):
        """Tag (NoData) components yield their component type, not data."""
        ecm = EntityComponentManager()
        model = ecm.create_entity()
        ecm.create_component(model, components.Model)
        ecm.create_component(model, components.Name, "model")
        ecm.create_entity()

        rows = ecm.each_data(components.Model)
        self.assertEqual(1, len(rows))
        self.assertEqual(model, rows[0][0])
        self.assertIsInstance(rows[0][1], components.ComponentProxy)
        self.assertEqual(components.Model, rows[0][1])

        rows = ecm.each_data(components.Model, components.Name)
        self.assertEqual([(model, components.Model, "model")], rows)

    def test_each_unattached_component(self):
        """A component type that no entity has yields an empty list."""
        ecm = EntityComponentManager()
        e = ecm.create_entity()
        ecm.create_component(e, components.Name, "only_a_name")

        self.assertEqual([], ecm.each_data(components.Pose))
        self.assertEqual([], ecm.each_data(components.Name, components.Pose))

    def test_each_new_and_removed_data(self):
        """each_new_data()/each_removed_data() follow the entity lifecycle."""
        ecm = EntityComponentManager()
        e1 = ecm.create_entity()
        e2 = ecm.create_entity()
        ecm.create_component(e1, components.Name, "e1")
        ecm.create_component(e2, components.Name, "e2")

        # Both entities are new, none is marked for removal.
        self.assertEqual({(e1, "e1"), (e2, "e2")},
                         set(ecm.each_new_data(components.Name)))
        self.assertEqual(2, len(ecm.each_new_data()))
        self.assertEqual([], ecm.each_removed_data(components.Name))

        ecm.request_remove_entity(e1)

        self.assertEqual([(e1, "e1")], ecm.each_removed_data(components.Name))
        self.assertEqual([(e1,)], ecm.each_removed_data())

        # each_data() deliberately still sees entities marked for removal,
        # which mirrors the C++ Each<T...>() behavior.
        self.assertEqual({(e1, "e1"), (e2, "e2")},
                         set(ecm.each_data(components.Name)))

    def test_each_snapshot_semantics(self):
        """Values returned by each_data() are copies, not ECM aliases."""
        ecm = EntityComponentManager()
        e = ecm.create_entity()
        ecm.create_component(e, components.Pose,
                             Pose3d(1.0, 2.0, 3.0, 0.0, 0.0, 0.0))

        pose = ecm.each_data(components.Pose)[0][1]
        self.assertAlmostEqual(1.0, pose.x())

        # Mutating the returned value must not reach the ECM.
        pose.set_x(42.0)
        self.assertAlmostEqual(1.0, ecm.each_data(components.Pose)[0][1].x())
        self.assertAlmostEqual(1.0, ecm.component_data(e, components.Pose).x())

        # The supported write path is visible to the next query.
        self.assertTrue(ecm.set_component_data(
            e, components.Pose, Pose3d(42.0, 2.0, 3.0, 0.0, 0.0, 0.0)))
        self.assertAlmostEqual(42.0, ecm.each_data(components.Pose)[0][1].x())

    def test_each_type_errors(self):
        """Non component-type arguments raise TypeError."""
        ecm = EntityComponentManager()
        e = ecm.create_entity()
        ecm.create_component(e, components.Name, "a")

        # A list of component types is not unpacked implicitly.
        with self.assertRaises(TypeError):
            ecm.each_data([components.Name])

        with self.assertRaises(TypeError):
            ecm.each_data("Name")

        with self.assertRaises(TypeError):
            ecm.each_data(components.Name, 12345)


if __name__ == "__main__":
    unittest.main()
