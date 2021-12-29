# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

from ignition.math import AxisAlignedBox, Planed, PlaneSide, Vector2d, Vector3d


class TestPlane(unittest.TestCase):

    def test_plane_constructor(self):
        plane = Planed(Vector3d(1, 0, 0), 0.1)
        self.assertEqual(plane.normal(), Vector3d(1, 0, 0))
        self.assertAlmostEqual(plane.offset(), 0.1, 1e-6)

        planeCopy = Planed(plane)
        self.assertEqual(plane.normal(), planeCopy.normal())
        self.assertEqual(plane.offset(), planeCopy.offset())
        self.assertEqual(plane.size(), planeCopy.size())

    def test_plane_distance(self):
        plane = Planed(Vector3d(0, 0, 1), 0.1)
        self.assertAlmostEqual(plane.distance(
            Vector3d(0, 0, 0),
            Vector3d(0, 0, 1)), 0.1, delta=1e-6)

        self.assertAlmostEqual(plane.distance(
            Vector3d(0, 0, 0.1),
            Vector3d(0, 0, 1)), 0, delta=1e-6)

        self.assertAlmostEqual(plane.distance(
            Vector3d(0, 0, 0.2),
            Vector3d(0, 0, 1)), -0.1, delta=1e-6)
        self.assertAlmostEqual(plane.distance(
            Vector3d(0, 0, 0.1),
            Vector3d(1, 0, 0)), 0, delta=1e-6)

    def test_plane(self):
        plane = Planed()
        self.assertEqual(plane.offset(), 0.0)
        self.assertEqual(plane.normal(), Vector3d())
        self.assertEqual(plane.size(), Vector2d(0, 0))

        plane = Planed(Vector3d(0, 0, 1), Vector2d(2, 3), 2.0)
        self.assertEqual(plane.offset(), 2.0)
        self.assertEqual(plane.normal(), Vector3d(0, 0, 1))
        self.assertEqual(plane.size(), Vector2d(2, 3))

        self.assertEqual(-1, plane.distance(
            Vector3d(0, 0, 1),
            Vector3d(0, 0, -1)))

        plane.set(Vector3d(1, 0, 0), Vector2d(1, 1), 1.0)
        self.assertEqual(plane.offset(), 1.0)
        self.assertEqual(plane.normal(), Vector3d(1, 0, 0))
        self.assertEqual(plane.size(), Vector2d(1, 1))

        plane = Planed(Vector3d(0, 1, 0), Vector2d(4, 4), 5.0)
        self.assertEqual(plane.offset(), 5.0)
        self.assertEqual(plane.normal(), Vector3d(0, 1, 0))
        self.assertEqual(plane.size(), Vector2d(4, 4))

    def test_side_point(self):
        plane = Planed(Vector3d(0, 0, 1), 1)

        # On the negative side of the plane (below the plane)
        point = Vector3d(0, 0, 0)
        self.assertEqual(plane.side(point), PlaneSide.NEGATIVE_SIDE)

        # Still on the negative side of the plane (below the plane)
        point.set(1, 1, 0)
        self.assertEqual(plane.side(point), PlaneSide.NEGATIVE_SIDE)

        # Above the plane (positive side)
        point.set(1, 1, 2)
        self.assertEqual(plane.side(point), PlaneSide.POSITIVE_SIDE)

        # On the plane
        point.set(0, 0, 1)
        self.assertEqual(plane.side(point), PlaneSide.NO_SIDE)

        # Change the plane, but the point is still on the negative side
        plane.set(Vector3d(1, 0, 0), 4)
        self.assertEqual(plane.side(point), PlaneSide.NEGATIVE_SIDE)

        # Point is now on the positive side
        point.set(4.1, 0, 1)
        self.assertEqual(plane.side(point), PlaneSide.POSITIVE_SIDE)

    def test_side__axis_aligned_box(self):
        plane = Planed(Vector3d(0, 0, 1), 1)

        # On the negative side of the plane (below the plane)
        box = AxisAlignedBox(Vector3d(-.5, -.5, -.5), Vector3d(.5, .5, .5))
        self.assertEqual(plane.side(box), PlaneSide.NEGATIVE_SIDE)

        # Still on the negative side of the plane (below the plane)
        box = AxisAlignedBox(Vector3d(-10, -10, -10), Vector3d(.9, .9, .9))
        self.assertEqual(plane.side(box), PlaneSide.NEGATIVE_SIDE)

        # Above the plane (positive side)
        box = AxisAlignedBox(Vector3d(2, 2, 2), Vector3d(3, 3, 3))
        self.assertEqual(plane.side(box), PlaneSide.POSITIVE_SIDE)

        # On both sides the plane
        box = AxisAlignedBox(Vector3d(0, 0, 0), Vector3d(3, 3, 3))
        self.assertEqual(plane.side(box), PlaneSide.BOTH_SIDE)

    def test_intersection(self):
        plane = Planed(Vector3d(0.5, 0, 1), 1)
        intersect = plane.intersection(Vector3d(0, 0, 0), Vector3d(1, 0, 1))
        self.assertTrue(intersect is not None)
        self.assertAlmostEqual(intersect.dot(plane.normal()), plane.offset(), 1e-6)

        plane.set(Vector3d(1, 0, 0), 2)
        intersect = plane.intersection(Vector3d(0, 0, 0), Vector3d(1, 0, 0))
        self.assertTrue(intersect is not None)
        self.assertEqual(intersect, Vector3d(2, 0, 0))

        intersect = plane.intersection(Vector3d(1, 1, 0), Vector3d(-1, -1, 0))
        self.assertTrue(intersect is not None)
        self.assertEqual(intersect, Vector3d(2, 2, 0))

        # Lines on plane
        intersect = plane.intersection(Vector3d(2, 0, 0), Vector3d(0, 1, 0))
        self.assertTrue(intersect is None)

        intersect = plane.intersection(Vector3d(2, 0, 0), Vector3d(0, 0, 1))
        self.assertTrue(intersect is None)

        intersect = plane.intersection(Vector3d(2, 0, 0), Vector3d(0, 1, 1))
        self.assertTrue(intersect is None)

        # Lines parallel to plane
        intersect = plane.intersection(Vector3d(0, 0, 0), Vector3d(0, 1, 0))
        self.assertTrue(intersect is None)

        intersect = plane.intersection(Vector3d(0, 0, 0), Vector3d(0, 0, 1))
        self.assertTrue(intersect is None)

        intersect = plane.intersection(Vector3d(0, 0, 0), Vector3d(0, 1, 1))
        self.assertTrue(intersect is None)

        # Bounded plane
        planeBounded = Planed(Vector3d(0, 0, 1), Vector2d(0.5, 0.5), 0)
        intersect1 = planeBounded.intersection(Vector3d(0, 0, 0), Vector3d(0, 0, 1))
        self.assertTrue(intersect1 is not None)
        self.assertEqual(intersect1, Vector3d(0, 0, 0))
        intersect2 = planeBounded.intersection(Vector3d(20, 20, 0), Vector3d(0, 0, 1))
        self.assertFalse(intersect2 is not None)


if __name__ == '__main__':
    unittest.main()
