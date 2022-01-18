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

import math
import unittest
from ignition.math import Line2d
from ignition.math import Vector2d


class TestLine2d(unittest.TestCase):

    def test_construction(self):
        line_a = Line2d(0, 0, 10, 10)
        self.assertAlmostEqual(line_a[0].x(), 0.0)
        self.assertAlmostEqual(line_a[0].y(), 0.0)
        self.assertAlmostEqual(line_a[1].x(), 10.0)
        self.assertAlmostEqual(line_a[1].y(), 10.0)

        line_b = Line2d(Vector2d(1, 2), Vector2d(3, 4))
        self.assertAlmostEqual(line_b[0].x(), 1.0)
        self.assertAlmostEqual(line_b[0].y(), 2.0)
        self.assertAlmostEqual(line_b[1].x(), 3.0)
        self.assertAlmostEqual(line_b[1].y(), 4.0)

        self.assertAlmostEqual(line_b[2].x(), line_b[1].x())

    def test_length(self):
        line_a = Line2d(0, 0, 10, 10)
        self.assertAlmostEqual(line_a.length(), math.sqrt(200), delta=1e-10)

    def test_slope(self):
        line = Line2d(0, 0, 10, 10)
        self.assertAlmostEqual(line.slope(), 1.0, delta=1e-10)

        line = Line2d(0, 0, 0, 10)
        self.assertTrue(math.isnan(line.slope()))

        line = Line2d(-10, 0, 100, 0)
        self.assertAlmostEqual(line.slope(), 0.0)

    def test_parallel_line(self):
        # Line is always parallel with itself
        line = Line2d(0, 0, 10, 0)
        self.assertTrue(line.parallel(line, 1e-10))

        # Degenerate line segment
        # Still expect Line is parallel with itself
        line = Line2d(0, 0, 0, 0)
        self.assertTrue(line.parallel(line, 1e-10))

        line_a = Line2d(0, 0, 10, 0)
        line_b = Line2d(0, 0, 10, 0)
        self.assertTrue(line_a.parallel(line_b, 1e-10))

        line_b.set(0, 0, 0, 10)
        self.assertFalse(line_a.parallel(line_b))

        line_b.set(0, 10, 10, 10)
        self.assertTrue(line_a.parallel(line_b))

        line_b.set(0, 10, 10, 10.00001)
        self.assertFalse(line_a.parallel(line_b, 1e-10))
        self.assertFalse(line_a.parallel(line_b))
        self.assertTrue(line_a.parallel(line_b, 1e-3))

    def test_collinear_line(self):
        # Line is always collinear with itself
        line = Line2d(0, 0, 10, 0)
        self.assertTrue(line.collinear(line, 1e-10))

        line_a = Line2d(0, 0, 10, 0)
        line_b = Line2d(0, 0, 10, 0)
        self.assertTrue(line_a.collinear(line_b, 1e-10))

        line_b.set(0, 10, 10, 10)
        self.assertFalse(line_a.collinear(line_b))

        line_b.set(9, 0, 10, 0.00001)
        self.assertFalse(line_a.collinear(line_b, 1e-10))
        self.assertFalse(line_a.collinear(line_b))
        self.assertTrue(line_a.collinear(line_b, 1e-3))

    def test_collinear_point(self):
        line_a = Line2d(0, 0, 10, 0)
        pt = Vector2d(0, 0)
        self.assertTrue(line_a.collinear(pt))

        pt_line = Line2d(pt, pt)
        self.assertTrue(line_a.collinear(pt_line))

        pt.set(1000, 0)
        self.assertTrue(line_a.collinear(pt, 1e-10))

        pt_line = Line2d(pt, pt)
        self.assertTrue(line_a.parallel(pt_line))
        self.assertFalse(line_a.intersect(pt_line))
        self.assertFalse(line_a.collinear(pt_line, 1e-10))

        pt.set(10, 0)
        pt_line.set(pt, pt)
        self.assertTrue(line_a.collinear(pt_line, 1e-10))

        pt.set(0, 0.00001)
        self.assertFalse(line_a.collinear(pt))
        self.assertTrue(line_a.collinear(pt, 1e-3))

        pt_line = Line2d(pt, pt)
        self.assertFalse(line_a.collinear(pt_line))
        self.assertTrue(line_a.parallel(pt_line))
        self.assertFalse(line_a.intersect(pt_line))
        self.assertTrue(line_a.intersect(pt_line, 1e-2))
        self.assertTrue(line_a.collinear(pt_line, 1e-3))

        pt.set(0, -0.00001)
        self.assertFalse(line_a.collinear(pt))
        self.assertTrue(line_a.collinear(pt, 1e-3))

        pt_line = Line2d(pt, pt)
        self.assertFalse(line_a.collinear(pt_line))
        self.assertTrue(line_a.collinear(pt_line, 1e-4))

    def test_intersect(self):
        pt = Vector2d()

        # parallel horizontal lines
        line_a = Line2d(1, 1, 2, 1)
        line_b = Line2d(1, 2, 2, 2)
        self.assertFalse(line_a.intersect(line_b, pt))

        # parallel vertical lines
        line_a.set(1, 1, 1, 10)
        line_b.set(2, 1, 2, 10)
        self.assertFalse(line_a.intersect(line_b, pt))

        # Two lines that form an inverted T with a gap
        line_a.set(1, 1, 1, 10)
        line_b.set(0, 0, 2, 0)
        self.assertFalse(line_a.intersect(line_b, pt))

        # Two lines that form a T with a gap
        line_a.set(1, 1, 1, 10)
        line_b.set(0, 10.1, 2, 10.1)
        self.assertFalse(line_a.intersect(line_b, pt))

        # Two lines that form an inverted T with a gap
        line_a.set(0, -10, 0, 10)
        line_b.set(1, 0, 10, 0)
        self.assertFalse(line_a.intersect(line_b, pt))

        # Two lines that form a T with a gap
        line_a.set(0, -10, 0, 10)
        line_b.set(-1, 0, -10, 0)
        self.assertFalse(line_a.intersect(line_b, pt))

        # Two collinear lines, one starts where the other stopped
        line_a.set(1, 1, 1, 10)
        line_b.set(1, 10, 1, 11)
        self.assertTrue(line_a.intersect(line_b, pt))
        self.assertEqual(pt, Vector2d(1, 10))

        # Two collinear lines, one overlaps the other
        line_a.set(0, 0, 0, 10)
        line_b.set(0, 9, 0, 11)
        self.assertTrue(line_a.intersect(line_b, pt))
        self.assertEqual(pt, Vector2d(0, 9))

        # Two collinear lines, one overlaps the other
        line_a.set(0, 0, 0, 10)
        line_b.set(0, -10, 0, 1)
        self.assertTrue(line_a.intersect(line_b, pt))
        self.assertEqual(pt, Vector2d(0, 1))

        # Two intersecting lines
        line_a.set(0, 0, 10, 10)
        line_b.set(0, 10, 10, 0)
        self.assertTrue(line_a.intersect(line_b, pt))
        self.assertEqual(pt, Vector2d(5, 5))

    def test_equality(self):
        line_a = Line2d(1, 1, 2, 1)
        line_b = Line2d(1, 2, 2, 2)

        self.assertTrue(line_a != line_b)
        self.assertTrue(line_a == line_a)

        line_b.set(1, 1, 2, 1.1)
        self.assertFalse(line_a == line_b)

        line_b.set(1, 1, 2.1, 1)
        self.assertFalse(line_a == line_b)

        line_b.set(1, 1.1, 2, 1)
        self.assertFalse(line_a == line_b)

        line_b.set(1.1, 1, 2, 1)
        self.assertFalse(line_a == line_b)

    def test_serialization(self):
        line = Line2d(0, 1, 2, 3)
        self.assertEqual(str(line), "0 1 2 3")


if __name__ == '__main__':
    unittest.main()
