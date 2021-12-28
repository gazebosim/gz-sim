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

import math
import unittest

from ignition.math import AxisAlignedBox, Helpers, Line3d, Vector3d


class TestAxisAlignedBox(unittest.TestCase):

    def test_empty_constructor(self):
        box = AxisAlignedBox()
        self.assertEqual(Vector3d(Helpers.MAX_D, Helpers.MAX_D, Helpers.MAX_D), box.min())
        self.assertEqual(Vector3d(Helpers.LOW_D, Helpers.LOW_D, Helpers.LOW_D), box.max())

    def test_constructor(self):
        box = AxisAlignedBox(Vector3d(0, -1, 2), Vector3d(1, -2, 3))
        self.assertEqual(Vector3d(0, -2, 2), box.min())
        self.assertEqual(Vector3d(1, -1, 3), box.max())

        box1 = AxisAlignedBox(box)
        self.assertEqual(box1.min(), box.min())
        self.assertEqual(box1.max(), box.max())

    def test_manual_set(self):
        box = AxisAlignedBox(Vector3d(0, -1, 2), Vector3d(1, -2, 3))

        box1 = AxisAlignedBox()
        box1.min().set(-2, 2, 3)
        box1.max().set(0, 0, 3)

        box1 += box
        self.assertEqual(box1.min().x(), -2)
        self.assertEqual(box1.min().y(), -2)
        self.assertEqual(box1.min().z(), 2)

        self.assertEqual(box1.max().x(), 1)
        self.assertEqual(box1.max().y(), 0)
        self.assertEqual(box1.max().z(), 3)

        box2 = AxisAlignedBox()
        box2.max().set(1, 1, 1)
        self.assertEqual(box2.size().x(), 0)
        self.assertEqual(box2.size().y(), 0)
        self.assertEqual(box2.size().z(), 0)

        box3 = AxisAlignedBox()
        box3.min().set(1, 1, 1)
        self.assertEqual(box3.size().x(), 0)
        self.assertEqual(box3.size().y(), 0)
        self.assertEqual(box3.size().z(), 0)

    def test_length(self):
        box = AxisAlignedBox(Vector3d(0, -1, 2), Vector3d(1, -2, 3))
        self.assertEqual(box.x_length(), 1)
        self.assertEqual(box.y_length(), 1)
        self.assertEqual(box.z_length(), 1)

    def test_size(self):
        box = AxisAlignedBox(Vector3d(0, -1, 2), Vector3d(1, -2, 3))
        self.assertEqual(box.size(), Vector3d(1, 1, 1))

    def test_center(self):
        box = AxisAlignedBox(Vector3d(0, -1, 2), Vector3d(1, -2, 3))
        self.assertEqual(box.center(), Vector3d(0.5, -1.5, 2.5))

    def test_merge_empty(self):
        box1 = AxisAlignedBox()
        box2 = AxisAlignedBox()
        box1.merge(box2)

        self.assertEqual(Vector3d(Helpers.MAX_D, Helpers.MAX_D, Helpers.MAX_D), box1.min())
        self.assertEqual(Vector3d(Helpers.LOW_D, Helpers.LOW_D, Helpers.LOW_D), box1.max())

    def test_default_constructor(self):
        defaultAxisAlignedBox1 = AxisAlignedBox()
        defaultAxisAlignedBox2 = AxisAlignedBox()
        defaultAxisAlignedBox3 = AxisAlignedBox(Vector3d(0, 0, 0), Vector3d(1, 1, 1))

        self.assertEqual(defaultAxisAlignedBox1.size(), Vector3d(0, 0, 0))

        self.assertEqual(defaultAxisAlignedBox1.x_length(), 0)
        self.assertEqual(defaultAxisAlignedBox1.y_length(), 0)
        self.assertEqual(defaultAxisAlignedBox1.z_length(), 0)

        self.assertEqual(defaultAxisAlignedBox1.center(), Vector3d(0, 0, 0))

        self.assertFalse(defaultAxisAlignedBox1.intersects(defaultAxisAlignedBox2))

        self.assertFalse(defaultAxisAlignedBox1.intersects(defaultAxisAlignedBox3))

        self.assertFalse(defaultAxisAlignedBox1.contains(Vector3d.ZERO))

    def test_minus(self):
        box1 = AxisAlignedBox(1, 2, 3, 4, 5, 6)
        sub = Vector3d(1, 1, 1)

        box2 = box1 - sub
        self.assertEqual(box2.min(), box1.min() - sub)
        self.assertEqual(box2.max(), box1.max() - sub)

    def test_plus(self):
        box1 = AxisAlignedBox(1, 2, 3, 4, 5, 6)
        add = Vector3d(1, 1, 1)
        box2 = box1 + add

        self.assertEqual(box2.min(), box1.min() + add)
        self.assertEqual(box2.max(), box1.max() + add)

    def test_plus_empty(self):
        box1 = AxisAlignedBox()
        box2 = AxisAlignedBox()
        box1 += box2

        self.assertEqual(Vector3d(Helpers.MAX_D, Helpers.MAX_D, Helpers.MAX_D), box1.min())
        self.assertEqual(Vector3d(Helpers.LOW_D, Helpers.LOW_D, Helpers.LOW_D), box1.max())

        box3 = box2 + box1
        self.assertEqual(Vector3d(Helpers.MAX_D, Helpers.MAX_D, Helpers.MAX_D), box3.min())
        self.assertEqual(Vector3d(Helpers.LOW_D, Helpers.LOW_D, Helpers.LOW_D), box3.max())

    def test_merge(self):
        box = AxisAlignedBox(Vector3d(0, -1, 2), Vector3d(1, -2, 3))
        box.merge(AxisAlignedBox(Vector3d(-1, -1, -1), Vector3d(2, 2, 2)))
        self.assertEqual(box, AxisAlignedBox(Vector3d(-1, -2, -1), Vector3d(2, 2, 3)))

    def test_volume(self):
        box = AxisAlignedBox(Vector3d(0, -1, 2), Vector3d(1, -2, 3))
        self.assertEqual(box.volume(), 1.0)

    def test_operator_equal(self):
        box = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3))
        box2 = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(1, 3, 3))
        box3 = AxisAlignedBox(Vector3d(0, 1, 1), Vector3d(1, 3, 3))
        self.assertTrue(box == AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3)))
        self.assertFalse(box == box2)
        self.assertFalse(box3 == box)

    def test_operator_not_equal(self):
        box = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3))
        box2 = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(1, 3, 3))
        box3 = AxisAlignedBox(Vector3d(0, 1, 1), Vector3d(1, 3, 3))
        self.assertFalse(box != AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3)))
        self.assertTrue(box != box2)
        self.assertTrue(box3 != box)

    def test_operator_plus_equal(self):
        box = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3))
        box += AxisAlignedBox(Vector3d(2, 2, 2), Vector3d(4, 4, 4))
        self.assertTrue(box == AxisAlignedBox(Vector3d(1, 1, 1),
                                              Vector3d(4, 4, 4)))

    def test_operator_plus(self):
        box = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3))
        box = box + AxisAlignedBox(Vector3d(-2, -2, -2), Vector3d(4, 4, 4))
        self.assertTrue(box == AxisAlignedBox(Vector3d(-2, -2, -2),
                                              Vector3d(4, 4, 4)))

    def test_intersects(self):
        box = AxisAlignedBox(Vector3d(0, 0, 0), Vector3d(1, 1, 1))

        self.assertFalse(box.intersects(AxisAlignedBox(
              Vector3d(1.1, 0, 0), Vector3d(2, 1, 1))))

        self.assertFalse(box.intersects(AxisAlignedBox(
              Vector3d(0, 1.1, 0), Vector3d(1, 2, 1))))

        self.assertFalse(box.intersects(AxisAlignedBox(
              Vector3d(0, 0, 1.1), Vector3d(1, 1, 2))))

        self.assertFalse(box.intersects(AxisAlignedBox(
              Vector3d(-1, -1, -1), Vector3d(-0.1, 0, 0))))

        self.assertFalse(box.intersects(AxisAlignedBox(
              Vector3d(-1, -1, -1), Vector3d(0, -0.1, 0))))

        self.assertFalse(box.intersects(AxisAlignedBox(
              Vector3d(-1, -1, -1), Vector3d(0, 0, -0.1))))

        self.assertTrue(box.intersects(AxisAlignedBox(
              Vector3d(0, 0, 0), Vector3d(1, 1, 1))))

    def test_contains(self):
        box = AxisAlignedBox(Vector3d(0, 0, 0), Vector3d(1, 1, 1))

        self.assertTrue(box.contains(Vector3d(0, 0, 0)))
        self.assertTrue(box.contains(Vector3d(0, 0, 1)))
        self.assertTrue(box.contains(Vector3d(0, 1, 1)))
        self.assertTrue(box.contains(Vector3d(1, 1, 1)))
        self.assertTrue(box.contains(Vector3d(1, 1, 0)))
        self.assertTrue(box.contains(Vector3d(1, 0, 0)))
        self.assertTrue(box.contains(Vector3d(0.5, 0.5, 0.5)))

        self.assertFalse(box.contains(Vector3d(0, 0, -1)))
        self.assertFalse(box.contains(Vector3d(0, -1, -1)))
        self.assertFalse(box.contains(Vector3d(-1, -1, -1)))
        self.assertFalse(box.contains(Vector3d(-1, -1, 0)))
        self.assertFalse(box.contains(Vector3d(-1, 0, 0)))

        self.assertFalse(box.contains(Vector3d(0.5, 0.5, -0.5)))
        self.assertFalse(box.contains(Vector3d(0.5, -0.5, 0.5)))
        self.assertFalse(box.contains(Vector3d(-0.5, 0.5, 0.5)))
        self.assertFalse(box.contains(Vector3d(-0.5, -0.5, 0.5)))
        self.assertFalse(box.contains(Vector3d(-0.5, -0.5, -0.5)))

        self.assertFalse(box.contains(Vector3d(0, 0, -0.01)))
        self.assertFalse(box.contains(Vector3d(0, -0.01, 0)))
        self.assertFalse(box.contains(Vector3d(-0.01, 0, 0)))

    def test_volume2(self):
        box = AxisAlignedBox()
        self.assertEqual(0.0, box.volume())

        box = AxisAlignedBox(Vector3d(-1, -2, -3), Vector3d(1, 2, 3))
        self.assertEqual(48.0, box.volume())

    def test_intersect(self):
        b = AxisAlignedBox(0, 0, 0, 1, 1, 1)

        intersect = False
        dist = 0
        pt = Vector3d()

        intersect, dist, pt = b.intersect(Vector3d(-1, 0, 0),
                                          Vector3d(1, 0, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(-1, 0, 0),
                        Vector3d(1, 0, 0), 0, 1000))
        self.assertEqual(dist, 1)
        self.assertEqual(b.intersect(Vector3d(-1, 0, 0),
                                     Vector3d(1, 0, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(1, 0, 0),
                                            Vector3d(-1, 0, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(1, 0, 0),
                                          Vector3d(-1, 0, 0), 0, 1000))
        self.assertEqual(dist, 0)
        self.assertEqual(b.intersect_dist(Vector3d(1, 0, 0),
                                          Vector3d(-1, 0, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d(1, 0, 0))

        (intersect, dist, pt) = b.intersect(Vector3d(2, 2, 0),
                                            Vector3d(-1, -1, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(2, 2, 0),
                                          Vector3d(-1, -1, 0), 0, 1000))
        self.assertEqual(dist, math.sqrt(2))
        self.assertEqual(b.intersect_dist(Vector3d(2, 2, 0),
                                          Vector3d(-1, -1, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d(1, 1, 0))

        (intersect, dist, pt) = b.intersect(Vector3d(-10, -10, 0),
                                            Vector3d(1, 1, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(-10, -10, 0),
                                          Vector3d(1, 1, 0), 0, 1000))
        self.assertEqual(dist, math.sqrt(200))
        self.assertEqual(b.intersect_dist(Vector3d(-10, -10, 0),
                                          Vector3d(1, 1, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(-1, -2, 0),
                                            Vector3d(1, 1, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(-1, -2, 0),
                                          Vector3d(1, 1, 0), 0, 1000))
        self.assertEqual(dist, 2*math.sqrt(2))
        self.assertEqual(b.intersect_dist(Vector3d(-1, -2, 0),
                                          Vector3d(1, 1, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d(1, 0, 0))

        (intersect, dist, pt) = b.intersect(Vector3d(2, 1, 0),
                                            Vector3d(-1, -1, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(2, 1, 0),
                                          Vector3d(-1, -1, 0), 0, 1000))
        self.assertEqual(dist, math.sqrt(2))
        self.assertEqual(b.intersect_dist(Vector3d(2, 1, 0),
                                          Vector3d(-1, -1, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d(1, 0, 0))

        (intersect, dist, pt) = b.intersect(Vector3d(0.5, 0.5, 2),
                                            Vector3d(0, 0, -1), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(0.5, 0.5, 2),
                                          Vector3d(0, 0, -1), 0, 1000))
        self.assertEqual(dist, 1)
        self.assertEqual(b.intersect_dist(Vector3d(0.5, 0.5, 2),
                                          Vector3d(0, 0, -1), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d(0.5, 0.5, 1))

        (intersect, dist, pt) = b.intersect(Vector3d(0.5, 0.5, 2),
                                            Vector3d(0, 0, 1), 0, 1000)
        self.assertFalse(intersect)
        self.assertFalse(b.intersect_check(Vector3d(0.5, 0.5, 2),
                                           Vector3d(0, 0, 1), 0, 1000))
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(-1, -1, 1),
                                            Vector3d(0, 0, -1), 0, 1000)
        self.assertFalse(intersect)
        self.assertFalse(b.intersect_check(Vector3d(-1, -1, 1),
                                           Vector3d(0, 0, -1), 0, 1000))
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(2, 2, 0),
                                            Vector3d(1, 1, 0), 0, 1000)
        self.assertFalse(intersect)
        self.assertFalse(b.intersect_check(Vector3d(2, 2, 0),
                                           Vector3d(1, 1, 0), 0, 1000))
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(2, 2, 0),
                                            Vector3d(0, 1, 0), 0, 1000)
        self.assertFalse(intersect)
        self.assertFalse(b.intersect_check(Vector3d(2, 2, 0),
                                           Vector3d(0, 1, 0), 0, 1000))
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(0.1, 0.1, 200),
                                            Vector3d(0, 0, -1), 0, 100)
        self.assertFalse(intersect)
        self.assertFalse(b.intersect_check(Vector3d(0.1, 0.1, 200),
                                           Vector3d(0, 0, -1), 0, 100))
        self.assertEqual(dist, 0)
        self.assertEqual(b.intersect_dist(Vector3d(0.1, 0.1, 200),
                                          Vector3d(0, 0, -1), 0, 100)[1], dist)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(0.1, 0.1, 1),
                                            Vector3d(0, 0, -1), 1.0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(0.1, 0.1, 1),
                                          Vector3d(0, 0, -1), 1.0, 1000))
        self.assertEqual(dist, 0.0)
        self.assertEqual(b.intersect_dist(Vector3d(0.1, 0.1, 1),
                                          Vector3d(0, 0, -1), 1.0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d(0.1, 0.1, 0))

        (intersect, dist, pt) = b.intersect(Vector3d(0.1, 0.1, 1),
                                            Vector3d(0, 0, -1), 1.1, 1000)
        self.assertFalse(intersect)
        self.assertFalse(b.intersect_check(Vector3d(0.1, 0.1, 1),
                                           Vector3d(0, 0, -1), 1.1, 1000))
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(0.1, 0.1, 10),
                                            Vector3d(0, 0, -1), 1.1, 5)
        self.assertFalse(intersect)
        self.assertFalse(b.intersect_check(Vector3d(0.1, 0.1, 10),
                                           Vector3d(0, 0, -1), 1.1, 5))
        self.assertEqual(dist, 0)
        self.assertEqual(b.intersect_dist(Vector3d(0.1, 0.1, 10),
                                          Vector3d(0, 0, -1), 1.1, 5)[1], dist)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(
          Line3d(Vector3d(4, 0, 0.5), Vector3d(0, 10, 0.5)))
        self.assertFalse(intersect)
        self.assertEqual(dist, 0)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(
          Line3d(Vector3d(1, -1, 1.5), Vector3d(0, 1, 1.5)))
        self.assertFalse(intersect)
        self.assertEqual(dist, 0)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(0, 0, 1),
                                            Vector3d(0, 0, -1), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(0, 0, 1),
                                          Vector3d(0, 0, -1), 0, 1000))
        self.assertEqual(dist, 0)
        self.assertEqual(b.intersect_dist(Vector3d(0, 0, 1),
                                          Vector3d(0, 0, -1), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d(0, 0, 1))

        (intersect, dist, pt) = b.intersect(Vector3d(0, 0, 0),
                                            Vector3d(1, 0, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(0, 0, 0),
                                          Vector3d(1, 0, 0), 0, 1000))
        self.assertEqual(dist, 0)
        self.assertEqual(b.intersect_dist(Vector3d(0, 0, 0),
                                          Vector3d(1, 0, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(0, 0, 0),
                                            Vector3d(-1, 0, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(0, 0, 0),
                                          Vector3d(-1, 0, 0), 0, 1000))
        self.assertEqual(dist, 0)
        self.assertEqual(b.intersect_dist(Vector3d(0, 0, 0),
                                          Vector3d(-1, 0, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(0, 0, 0),
                                            Vector3d(0, 1, 0), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(0, 0, 0),
                                          Vector3d(0, 1, 0), 0, 1000))
        self.assertEqual(dist, 0)
        self.assertEqual(b.intersect_dist(Vector3d(0, 0, 0),
                                          Vector3d(0, 1, 0), 0, 1000)[1], dist)
        self.assertEqual(pt, Vector3d.ZERO)

        (intersect, dist, pt) = b.intersect(Vector3d(0.5, 0.5, 0.5),
                                            Vector3d(-.707107, 0, -0.707107), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(0.5, 0.5, 0.5),
                                          Vector3d(-.707107, 0, -0.707107), 0, 1000))
        self.assertAlmostEqual(dist, 0, 1e-5)
        self.assertAlmostEqual(b.intersect(Vector3d(0.5, 0.5, 0.5),
                                           Vector3d(-.707107, 0, -0.707107), 0, 1000)[1],
                               dist,
                               1e-5)
        self.assertEqual(pt, Vector3d(0.5, 0.5, 0.5))

        (intersect, dist, pt) = b.intersect(Vector3d(1.2, 0, 0.5),
                                            Vector3d(-0.707107, 0, -0.707107), 0, 1000)
        self.assertTrue(intersect)
        self.assertTrue(b.intersect_check(Vector3d(1.2, 0, 0.5),
                                          Vector3d(-0.707107, 0, -0.707107), 0, 1000))
        self.assertAlmostEqual(dist, 0.28284, delta=1e-5)
        self.assertAlmostEqual(b.intersect(Vector3d(1.2, 0, 0.5),
                                           Vector3d(-0.707107, 0, -0.707107), 0, 1000)[1],
                               dist,
                               1e-5)
        self.assertEqual(pt, Vector3d(1, 0, 0.3))


if __name__ == '__main__':
    unittest.main()
