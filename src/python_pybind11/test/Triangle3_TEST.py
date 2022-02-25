# Copyright (C) 2021 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http:#www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import unittest
from ignition.math import Line3d
from ignition.math import Triangle3d
from ignition.math import Vector3d


class TestTriangle(unittest.TestCase):

    def test_constructor(self):
        # Constructor
        tri = Triangle3d()
        self.assertAlmostEqual(tri[0], Vector3d(0, 0, 0))
        self.assertAlmostEqual(tri[1], Vector3d(0, 0, 0))
        self.assertAlmostEqual(tri[2], Vector3d(0, 0, 0))

        # Construct from three points
        tri = Triangle3d(Vector3d(0, 0, 0),
                         Vector3d(0, 1, 0),
                         Vector3d(1, 0, 0))

        self.assertTrue(tri.valid())

        self.assertAlmostEqual(tri[0], Vector3d(0, 0, 0))
        self.assertAlmostEqual(tri[1], Vector3d(0, 1, 0))
        self.assertAlmostEqual(tri[2], Vector3d(1, 0, 0))
        self.assertAlmostEqual(tri[3], tri[2])

        # Construct degenerate from 3 collinear points
        tri = Triangle3d(Vector3d(0, 0, 0),
                         Vector3d(0, 1, 0),
                         Vector3d(0, 2, 0))

        # Expect not valid
        self.assertFalse(tri.valid())

    def test_set(self):
        tri = Triangle3d()

        tri.set(0, Vector3d(3, 4, 1))
        tri.set(1, Vector3d(5, 6, 2))
        tri.set(2, Vector3d(7, 8, -3))
        self.assertAlmostEqual(tri[0], Vector3d(3, 4, 1))
        self.assertAlmostEqual(tri[1], Vector3d(5, 6, 2))
        self.assertAlmostEqual(tri[2], Vector3d(7, 8, -3))

        tri.set(Vector3d(0.1, 0.2, -0.3),
                Vector3d(0.3, 0.4, 0.5),
                Vector3d(1.5, 2.6, 3.7))
        self.assertAlmostEqual(tri[0], Vector3d(0.1, 0.2, -0.3))
        self.assertAlmostEqual(tri[1], Vector3d(0.3, 0.4, 0.5))
        self.assertAlmostEqual(tri[2], Vector3d(1.5, 2.6, 3.7))

    def test_side(self):
        tri = Triangle3d(Vector3d(0, 0, 0),
                         Vector3d(0, 1, 1),
                         Vector3d(1, 0, 2))

        self.assertTrue(tri.side(0) == Line3d(0, 0, 0, 0, 1, 1))
        self.assertTrue(tri.side(1) == Line3d(0, 1, 1, 1, 0, 2))
        self.assertTrue(tri.side(2) == Line3d(1, 0, 2, 0, 0, 0))

    def test_contains_line(self):
        tri = Triangle3d(Vector3d(0, 0, 0),
                         Vector3d(0, 1, 0),
                         Vector3d(1, 0, 0))

        self.assertTrue(tri.contains(tri.side(0)))
        self.assertTrue(tri.contains(tri.side(1)))
        self.assertTrue(tri.contains(tri.side(2)))

        self.assertTrue(tri.contains(Line3d(0.1, 0.1, 0.0, 0.5, 0.5, 0.0)))

        self.assertFalse(tri.contains(Line3d(0.1, 0.1, 0.0, 0.6, 0.6, 0.0)))
        self.assertFalse(tri.contains(Line3d(-0.1, -0.1, 0.0, 0.5, 0.5, 0.0)))
        self.assertFalse(tri.contains(Line3d(0.1, 0.1, 0.1, 0.5, 0.5, 0.1)))
        self.assertFalse(tri.contains(Line3d(0.1, 0.1, -0.1, 0.1, 0.1, 0.1)))

    def test_intersects(self):
        pt1 = Vector3d()
        pt2 = Vector3d()
        tri = Triangle3d(Vector3d(0, 0, 0),
                         Vector3d(0, 1, 0),
                         Vector3d(1, 0, 0))

        self.assertTrue(tri.intersects(tri.side(0), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0, 0, 0))

        self.assertTrue(tri.intersects(tri.side(1), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0, 1, 0))

        self.assertTrue(tri.intersects(tri.side(2), pt1))
        self.assertAlmostEqual(pt1, Vector3d(1, 0, 0))

        self.assertTrue(tri.intersects(Line3d(0.1, 0.1, 0.0, 0.5, 0.5, 0.0), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.1, 0.1, 0.0))

        self.assertTrue(tri.intersects(Line3d(0.1, 0.1, 0.0, 0.6, 0.6, 0.0), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.5, 0.5, 0.0))

        self.assertTrue(tri.intersects(Line3d(0.6, 0.6, 0, 0.1, 0.1, 0), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.5, 0.5, 0.0))

        self.assertTrue(tri.intersects(Line3d(0.1, 0.1, 0, -0.6, 0.1, 0), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.0, 0.1, 0))

        self.assertTrue(tri.intersects(Line3d(0.1, 0.1, 0, 0.1, -0.1, 0), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.1, 0.0, 0))

        self.assertTrue(tri.intersects(Line3d(-0.1, -0.1, 0, 0.5, 0.5, 0), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.0, 0.0, 0))

        self.assertTrue(tri.intersects(Line3d(-2, -2, 0, 0.2, 0.2, 0), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.0, 0.0, 0))

        self.assertTrue(tri.intersects(Line3d(0.1, 0.1, -1, 0.1, 0.1, 1), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.1, 0.1, 0))

        self.assertTrue(tri.intersects(Line3d(0.1, 0.1, -1, 0.3, 0.3, 1), pt1))
        self.assertAlmostEqual(pt1, Vector3d(0.2, 0.2, 0))

        self.assertFalse(tri.intersects(Line3d(-0.1, 0, 0, -0.1, 1, 0), pt1))

    def test_contains_pt(self):
        tri = Triangle3d(Vector3d(0, 0, 0.0),
                         Vector3d(0, 1, 0.0),
                         Vector3d(1, 0, 0.0))

        self.assertTrue(tri.contains(tri[0]))
        self.assertTrue(tri.contains(tri[1]))
        self.assertTrue(tri.contains(tri[2]))

        self.assertTrue(tri.contains(Vector3d(0.1, 0.1, 0.0)))
        self.assertTrue(tri.contains(Vector3d(0.0, 0.5, 0.0)))
        self.assertTrue(tri.contains(Vector3d(0.5, 0.0, 0.0)))
        self.assertTrue(tri.contains(Vector3d(0.5, 0.5, 0.0)))

        self.assertFalse(tri.contains(Vector3d(-0.01, -0.01, 0.0)))
        self.assertFalse(tri.contains(Vector3d(1.01, 0.0, 0.0)))
        self.assertFalse(tri.contains(Vector3d(0, 1.01, 0.0)))
        self.assertFalse(tri.contains(Vector3d(0.1, 0.1, 0.1)))
        self.assertFalse(tri.contains(Vector3d(0.1, 0.1, -0.1)))

    def test_perimeter(self):
        tri = Triangle3d(Vector3d(0, 0, 0),
                         Vector3d(0, 1, 0),
                         Vector3d(1, 0, 0))

        self.assertAlmostEqual(tri.perimeter(), 2.0 + math.sqrt(2.0))

        tri = Triangle3d(Vector3d(0, 0, 1),
                         Vector3d(0, 1, 0),
                         Vector3d(1, 0, 0))

        self.assertAlmostEqual(tri.perimeter(), 3.0 * math.sqrt(2.0))

    def test_area(self):
        tri = Triangle3d(Vector3d(0, 0, 0),
                         Vector3d(0, 1, 0),
                         Vector3d(1, 0, 0))

        self.assertAlmostEqual(tri.area(), 0.5, delta=1e-6)

        tri = Triangle3d(Vector3d(0, 0, 1),
                         Vector3d(0, 1, 0),
                         Vector3d(1, 0, 0))

        self.assertAlmostEqual(tri.area(), (math.sqrt(2.0) * math.sqrt(1.5))*0.5, delta=1e-6)


if __name__ == '__main__':
    unittest.main()
