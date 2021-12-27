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
from ignition.math import Line2d
from ignition.math import Triangled
from ignition.math import Vector2d


class TestTriangle(unittest.TestCase):

    def test_constructor(self):
        # Constructor
        tri = Triangled()
        self.assertAlmostEqual(tri[0], Vector2d(0, 0))
        self.assertAlmostEqual(tri[1], Vector2d(0, 0))
        self.assertAlmostEqual(tri[2], Vector2d(0, 0))

        # Construct from three points
        tri = Triangled(Vector2d(0, 0),
                        Vector2d(0, 1),
                        Vector2d(1, 0))

        self.assertTrue(tri.valid())

        self.assertAlmostEqual(tri[0], Vector2d(0, 0))
        self.assertAlmostEqual(tri[1], Vector2d(0, 1))
        self.assertAlmostEqual(tri[2], Vector2d(1, 0))
        self.assertAlmostEqual(tri[3], tri[2])

        # Construct degenerate from 3 collinear points
        tri = Triangled(Vector2d(0, 0),
                        Vector2d(0, 1),
                        Vector2d(0, 2))

        # Expect not valid
        self.assertFalse(tri.valid())

    def test_set(self):
        tri = Triangled()

        tri.set(0, Vector2d(3, 4))
        tri.set(1, Vector2d(5, 6))
        tri.set(2, Vector2d(7, 8))
        self.assertAlmostEqual(tri[0], Vector2d(3, 4))
        self.assertAlmostEqual(tri[1], Vector2d(5, 6))
        self.assertAlmostEqual(tri[2], Vector2d(7, 8))

        tri.set(Vector2d(0.1, 0.2),
                Vector2d(0.3, 0.4),
                Vector2d(1.5, 2.6))
        self.assertAlmostEqual(tri[0], Vector2d(0.1, 0.2))
        self.assertAlmostEqual(tri[1], Vector2d(0.3, 0.4))
        self.assertAlmostEqual(tri[2], Vector2d(1.5, 2.6))

    def test_side(self):
        tri = Triangled(Vector2d(0, 0),
                        Vector2d(0, 1),
                        Vector2d(1, 0))

        self.assertTrue(tri.side(0) == Line2d(0, 0, 0, 1))
        self.assertTrue(tri.side(1) == Line2d(0, 1, 1, 0))
        self.assertTrue(tri.side(2) == Line2d(1, 0, 0, 0))

    def test_contains_line(self):
        tri = Triangled(Vector2d(0, 0),
                        Vector2d(0, 1),
                        Vector2d(1, 0))

        self.assertTrue(tri.contains(tri.side(0)))
        self.assertTrue(tri.contains(tri.side(1)))
        self.assertTrue(tri.contains(tri.side(2)))

        self.assertTrue(tri.contains(Line2d(0.1, 0.1, 0.5, 0.5)))

        self.assertFalse(tri.contains(Line2d(0.1, 0.1, 0.6, 0.6)))
        self.assertFalse(tri.contains(Line2d(-0.1, -0.1, 0.5, 0.5)))

    def test_intersects(self):
        pt1 = Vector2d()
        pt2 = Vector2d()
        tri = Triangled(Vector2d(0, 0),
                        Vector2d(0, 1),
                        Vector2d(1, 0))

        self.assertTrue(tri.intersects(tri.side(0), pt1, pt2))
        self.assertAlmostEqual(pt1, Vector2d(0, 0))
        self.assertAlmostEqual(pt2, Vector2d(0, 1))

        self.assertTrue(tri.intersects(tri.side(1), pt1, pt2))
        self.assertAlmostEqual(pt1, Vector2d(0, 1))
        self.assertAlmostEqual(pt2, Vector2d(1, 0))

        self.assertTrue(tri.intersects(tri.side(2), pt1, pt2))
        self.assertAlmostEqual(pt1, Vector2d(1, 0))
        self.assertAlmostEqual(pt2, Vector2d(0, 0))

        self.assertTrue(tri.intersects(Line2d(0.1, 0.1, 0.5, 0.5), pt1, pt2))
        self.assertAlmostEqual(pt1, Vector2d(0.1, 0.1))
        self.assertAlmostEqual(pt2, Vector2d(0.5, 0.5))

        self.assertTrue(tri.intersects(Line2d(0.1, 0.1, 0.6, 0.6), pt1, pt2))
        self.assertAlmostEqual(pt1, Vector2d(0.5, 0.5))
        self.assertAlmostEqual(pt2, Vector2d(0.1, 0.1))

        self.assertTrue(tri.intersects(Line2d(-0.1, -0.1, 0.5, 0.5), pt1, pt2))
        self.assertAlmostEqual(pt1, Vector2d(0.0, 0.0))
        self.assertAlmostEqual(pt2, Vector2d(0.5, 0.5))

        self.assertTrue(tri.intersects(Line2d(-2, -2, 0.2, 0.2), pt1, pt2))
        self.assertAlmostEqual(pt1, Vector2d(0.0, 0.0))
        self.assertAlmostEqual(pt2, Vector2d(0.2, 0.2))

        self.assertFalse(tri.intersects(Line2d(-0.1, 0, -0.1, 1), pt1, pt2))

    def test_contains_pt(self):
        tri = Triangled(Vector2d(0, 0),
                        Vector2d(0, 1),
                        Vector2d(1, 0))

        self.assertTrue(tri.contains(tri[0]))
        self.assertTrue(tri.contains(tri[1]))
        self.assertTrue(tri.contains(tri[2]))

        self.assertTrue(tri.contains(Vector2d(0.1, 0.1)))
        self.assertTrue(tri.contains(Vector2d(0, 0.5)))
        self.assertTrue(tri.contains(Vector2d(0.5, 0)))
        self.assertTrue(tri.contains(Vector2d(0.5, 0.5)))

        self.assertFalse(tri.contains(Vector2d(-0.01, -0.01)))
        self.assertFalse(tri.contains(Vector2d(1.01, 0)))
        self.assertFalse(tri.contains(Vector2d(0, 1.01)))

    def test_perimeter(self):
        tri = Triangled(Vector2d(0, 0),
                        Vector2d(0, 1),
                        Vector2d(1, 0))

        self.assertAlmostEqual(tri.perimeter(), 2.0 + math.sqrt(2.0))

    def test_area(self):
        tri = Triangled(Vector2d(0, 0),
                        Vector2d(0, 1),
                        Vector2d(1, 0))

        self.assertAlmostEqual(tri.area(), 0.499999, delta=1e-6)


if __name__ == '__main__':
    unittest.main()
