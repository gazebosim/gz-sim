# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
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
from ignition.math import Vector3d
from ignition.math import Vector3Stats


class TestVector3Stats(unittest.TestCase):

    stats = Vector3Stats()

    def x(self, _name):
        return self.stats.x().map()[_name]

    def y(self, _name):
        return self.stats.y().map()[_name]

    def z(self, _name):
        return self.stats.z().map()[_name]

    def mag(self, _name):
        return self.stats.mag().map()[_name]

    def test_vector3stats_constructor(self):
        # Constructor
        v3stats = Vector3Stats()
        self.assertTrue(len(v3stats.x().map()) == 0)
        self.assertTrue(len(v3stats.y().map()) == 0)
        self.assertTrue(len(v3stats.z().map()) == 0)
        self.assertTrue(len(v3stats.mag().map()) == 0)
        self.assertAlmostEqual(v3stats.x().count(), 0)
        self.assertAlmostEqual(v3stats.y().count(), 0)
        self.assertAlmostEqual(v3stats.z().count(), 0)
        self.assertAlmostEqual(v3stats.mag().count(), 0)

        # Reset
        v3stats.reset()
        self.assertTrue(len(v3stats.x().map()) == 0)
        self.assertTrue(len(v3stats.y().map()) == 0)
        self.assertTrue(len(v3stats.z().map()) == 0)
        self.assertTrue(len(v3stats.mag().map()) == 0)
        self.assertAlmostEqual(v3stats.x().count(), 0)
        self.assertAlmostEqual(v3stats.y().count(), 0)
        self.assertAlmostEqual(v3stats.z().count(), 0)
        self.assertAlmostEqual(v3stats.mag().count(), 0)

        # InsertStatistics
        v3stats = Vector3Stats()
        self.assertTrue(len(v3stats.x().map()) == 0)
        self.assertTrue(len(v3stats.y().map()) == 0)
        self.assertTrue(len(v3stats.z().map()) == 0)
        self.assertTrue(len(v3stats.mag().map()) == 0)

        self.assertTrue(v3stats.insert_statistics("maxAbs"))
        self.assertFalse(v3stats.insert_statistics("maxAbs"))
        self.assertFalse(v3stats.insert_statistic("maxAbs"))
        self.assertFalse(len(v3stats.x().map()) == 0)
        self.assertFalse(len(v3stats.y().map()) == 0)
        self.assertFalse(len(v3stats.z().map()) == 0)
        self.assertFalse(len(v3stats.mag().map()) == 0)

        # Map with no data
        map = v3stats.x().map()

        self.assertAlmostEqual(len(map), 1)
        self.assertTrue("maxAbs" in map, 1)

        map = v3stats.y().map()
        self.assertAlmostEqual(len(map), 1)
        self.assertTrue("maxAbs" in map, 1)

        map = v3stats.z().map()
        self.assertAlmostEqual(len(map), 1)
        self.assertTrue("maxAbs" in map, 1)

        map = v3stats.mag().map()
        self.assertAlmostEqual(len(map), 1)
        self.assertTrue("maxAbs" in map, 1)

        # Insert some data
        self.assertAlmostEqual(v3stats.x().count(), 0)
        self.assertAlmostEqual(v3stats.y().count(), 0)
        self.assertAlmostEqual(v3stats.z().count(), 0)
        self.assertAlmostEqual(v3stats.mag().count(), 0)

        v3stats.insert_data(Vector3d.UNIT_X)
        v3stats.insert_data(Vector3d.UNIT_X)
        v3stats.insert_data(Vector3d.UNIT_Y)

        self.assertAlmostEqual(v3stats.x().count(), 3)
        self.assertAlmostEqual(v3stats.y().count(), 3)
        self.assertAlmostEqual(v3stats.z().count(), 3)
        self.assertAlmostEqual(v3stats.mag().count(), 3)

        self.assertAlmostEqual(v3stats.x().map()["maxAbs"], 1.0, delta=1e-10)
        self.assertAlmostEqual(v3stats.y().map()["maxAbs"], 1.0, delta=1e-10)
        self.assertAlmostEqual(v3stats.z().map()["maxAbs"], 0.0)
        self.assertAlmostEqual(v3stats.mag().map()["maxAbs"], 1.0, delta=1e-10)

    def test_vector3stats_const_accessor(self):
        # Const accessors
        self.assertTrue(len(self.stats.x().map()) == 0)
        self.assertTrue(len(self.stats.y().map()) == 0)
        self.assertTrue(len(self.stats.z().map()) == 0)
        self.assertTrue(len(self.stats.mag().map()) == 0)

        name = "maxAbs"
        self.assertTrue(self.stats.insert_statistics(name))

        self.stats.insert_data(Vector3d.UNIT_X)
        self.stats.insert_data(Vector3d.UNIT_X)
        self.stats.insert_data(Vector3d.UNIT_Y)

        self.assertAlmostEqual(self.stats.x().count(), 3)
        self.assertAlmostEqual(self.stats.y().count(), 3)
        self.assertAlmostEqual(self.stats.z().count(), 3)
        self.assertAlmostEqual(self.stats.mag().count(), 3)

        self.assertAlmostEqual(self.stats.x().map()[name], 1.0, delta=1e-10)
        self.assertAlmostEqual(self.stats.y().map()[name], 1.0, delta=1e-10)
        self.assertAlmostEqual(self.stats.z().map()[name], 0.0)
        self.assertAlmostEqual(self.stats.mag().map()[name], 1.0, delta=1e-10)

        self.assertAlmostEqual(self.x(name), 1.0, delta=1e-10)
        self.assertAlmostEqual(self.y(name), 1.0, delta=1e-10)
        self.assertAlmostEqual(self.z(name), 0.0)
        self.assertAlmostEqual(self.mag(name), 1.0, delta=1e-10)


if __name__ == '__main__':
    unittest.main()
