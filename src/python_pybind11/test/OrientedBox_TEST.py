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

from ignition.math import OrientedBoxd, MassMatrix3d, Material, Pose3d, Vector3d

g_tolerance = 1e-6


class TestOrientedBox(unittest.TestCase):

    def test_empty_constructor_new(self):
        box = OrientedBoxd()

        self.assertEqual(box.size(), Vector3d.ZERO)
        self.assertEqual(box.pose(), Pose3d.ZERO)
        # self.assertEqual(Material(), box.material())

    def test_size_only_constructor(self):
        box = OrientedBoxd(Vector3d(1, 2, 3))
        self.assertEqual(box.size(), Vector3d(1, 2, 3))
        self.assertEqual(box.pose(), Pose3d.ZERO)

    def test_negative_size_constructor(self):
        box = OrientedBoxd(Vector3d(-1, 0, -3))
        self.assertEqual(box.size(), Vector3d(1, 0, 3))

    def test_size_pose_constructor(self):
        box = OrientedBoxd(Vector3d(1, 2, 3), Pose3d(-1, -2, -3, 0, 1, 2))
        self.assertEqual(box.size(), Vector3d(1, 2, 3))
        self.assertEqual(box.pose(), Pose3d(-1, -2, -3, 0, 1, 2))

    def test_copy_constructor(self):

        box1 = OrientedBoxd(Vector3d(0.1, 0.2, 0.3),
                            Pose3d(-0.1, -0.2, 0.0, 1.1, 1.2, 1.3))
        box2 = OrientedBoxd(box1)

        self.assertEqual(box2.size(), Vector3d(0.1, 0.2, 0.3))
        self.assertEqual(box2.pose(), Pose3d(-0.1, -0.2, 0.0, 1.1, 1.2, 1.3))

    def test_length(self):
        box = OrientedBoxd(Vector3d(0.1, -2.1, 0.0))
        self.assertEqual(box.x_length(), 0.1)
        self.assertEqual(box.y_length(), 2.1)
        self.assertEqual(box.z_length(), 0.0)

    def test_operator_equal(self):
        box = OrientedBoxd(Vector3d(1, 1, 1))
        box2 = OrientedBoxd(Vector3d(1, 1, 1),
                            Pose3d(1, 2, 3, 4, 5, 6))
        box3 = OrientedBoxd(Vector3d(0, 0, 0),
                            Pose3d(1, 2, 3, 4, 5, 6))
        self.assertEqual(box, OrientedBoxd(Vector3d(1, 1, 1)))
        self.assertNotEqual(box, box2)
        self.assertNotEqual(box3, box)

    def test_constains_zero_box(self):
        box = OrientedBoxd()

        self.assertTrue(box.contains(Vector3d(0, 0, 0)))
        self.assertFalse(box.contains(Vector3d(0, 0, 0.0001)))

    def test_constains_zero_pose(self):
        box = OrientedBoxd(Vector3d(1, 2, 3))

        # Vertices
        self.assertTrue(box.contains(Vector3d(-0.5, -1.0, -1.5)))
        self.assertTrue(box.contains(Vector3d(-0.5, -1.0, +1.5)))
        self.assertTrue(box.contains(Vector3d(-0.5, +1.0, -1.5)))
        self.assertTrue(box.contains(Vector3d(-0.5, +1.0, +1.5)))

        self.assertTrue(box.contains(Vector3d(+0.5, -1.0, -1.5)))
        self.assertTrue(box.contains(Vector3d(+0.5, -1.0, +1.5)))
        self.assertTrue(box.contains(Vector3d(+0.5, +1.0, -1.5)))
        self.assertTrue(box.contains(Vector3d(+0.5, +1.0, +1.5)))

        # Edges
        self.assertTrue(box.contains(Vector3d(0.0, -1.0, -1.5)))
        self.assertTrue(box.contains(Vector3d(0.0, -1.0, +1.5)))
        self.assertTrue(box.contains(Vector3d(0.0, +1.0, -1.5)))
        self.assertTrue(box.contains(Vector3d(0.0, +1.0, +1.5)))

        self.assertTrue(box.contains(Vector3d(-0.5, -1.0, 0.0)))
        self.assertTrue(box.contains(Vector3d(-0.5, +1.0, 0.0)))
        self.assertTrue(box.contains(Vector3d(+0.5, -1.0, 0.0)))
        self.assertTrue(box.contains(Vector3d(+0.5, +1.0, 0.0)))

        self.assertTrue(box.contains(Vector3d(-0.5, 0.0, -1.5)))
        self.assertTrue(box.contains(Vector3d(-0.5, 0.0, +1.5)))
        self.assertTrue(box.contains(Vector3d(+0.5, 0.0, -1.5)))
        self.assertTrue(box.contains(Vector3d(+0.5, 0.0, +1.5)))

        # Inside
        self.assertTrue(box.contains(Vector3d(-0.5+g_tolerance,
                                              -1.0+g_tolerance,
                                              -1.5+g_tolerance)))
        self.assertTrue(box.contains(Vector3d(-0.5+g_tolerance,
                                              -1.0+g_tolerance,
                                              +1.5-g_tolerance)))
        self.assertTrue(box.contains(Vector3d(-0.5+g_tolerance,
                                              +1.0-g_tolerance,
                                              -1.5+g_tolerance)))
        self.assertTrue(box.contains(Vector3d(-0.5+g_tolerance,
                                              +1.0-g_tolerance,
                                              +1.5-g_tolerance)))

        self.assertTrue(box.contains(Vector3d(+0.5-g_tolerance,
                                              -1.0+g_tolerance,
                                              -1.5+g_tolerance)))
        self.assertTrue(box.contains(Vector3d(+0.5-g_tolerance,
                                              -1.0+g_tolerance,
                                              +1.5-g_tolerance)))
        self.assertTrue(box.contains(Vector3d(+0.5-g_tolerance,
                                              +1.0-g_tolerance,
                                              -1.5+g_tolerance)))
        self.assertTrue(box.contains(Vector3d(+0.5-g_tolerance,
                                              +1.0-g_tolerance,
                                              +1.5-g_tolerance)))

        # Outside
        self.assertFalse(box.contains(Vector3d(-0.5-g_tolerance,
                                               -1.0-g_tolerance,
                                               -1.5-g_tolerance)))
        self.assertFalse(box.contains(Vector3d(-0.5-g_tolerance,
                                               -1.0-g_tolerance,
                                               +1.5+g_tolerance)))
        self.assertFalse(box.contains(Vector3d(-0.5-g_tolerance,
                                               +1.0+g_tolerance,
                                               -1.5-g_tolerance)))
        self.assertFalse(box.contains(Vector3d(-0.5-g_tolerance,
                                               +1.0+g_tolerance,
                                               +1.5+g_tolerance)))

        self.assertFalse(box.contains(Vector3d(+0.5-g_tolerance,
                                               -1.0-g_tolerance,
                                               -1.5-g_tolerance)))
        self.assertFalse(box.contains(Vector3d(+0.5-g_tolerance,
                                               -1.0-g_tolerance,
                                               +1.5+g_tolerance)))
        self.assertFalse(box.contains(Vector3d(+0.5-g_tolerance,
                                               +1.0+g_tolerance,
                                               -1.5-g_tolerance)))
        self.assertFalse(box.contains(Vector3d(+0.5-g_tolerance,
                                               +1.0+g_tolerance,
                                               +1.5+g_tolerance)))

    def test_contains_oriented_position(self):

        box = OrientedBoxd(Vector3d(1, 2, 3),
                           Pose3d(10, 20, 30, 0, 0, 0))

        # Vertices
        self.assertTrue(box.contains(Vector3d(10-0.5, 20-1.0, 30-1.5)))
        self.assertTrue(box.contains(Vector3d(10-0.5, 20-1.0, 30+1.5)))
        self.assertTrue(box.contains(Vector3d(10-0.5, 20+1.0, 30-1.5)))
        self.assertTrue(box.contains(Vector3d(10-0.5, 20+1.0, 30+1.5)))

        self.assertTrue(box.contains(Vector3d(10+0.5, 20-1.0, 30-1.5)))
        self.assertTrue(box.contains(Vector3d(10+0.5, 20-1.0, 30+1.5)))
        self.assertTrue(box.contains(Vector3d(10+0.5, 20+1.0, 30-1.5)))
        self.assertTrue(box.contains(Vector3d(10+0.5, 20+1.0, 30+1.5)))

        # Edges
        self.assertTrue(box.contains(Vector3d(10.0, 20-1.0, 30-1.5)))
        self.assertTrue(box.contains(Vector3d(10.0, 20-1.0, 30+1.5)))
        self.assertTrue(box.contains(Vector3d(10.0, 20+1.0, 30-1.5)))
        self.assertTrue(box.contains(Vector3d(10.0, 20+1.0, 30+1.5)))

        self.assertTrue(box.contains(Vector3d(10-0.5, 20-1.0, 30.0)))
        self.assertTrue(box.contains(Vector3d(10-0.5, 20+1.0, 30.0)))
        self.assertTrue(box.contains(Vector3d(10+0.5, 20-1.0, 30.0)))
        self.assertTrue(box.contains(Vector3d(10+0.5, 20+1.0, 30.0)))

        self.assertTrue(box.contains(Vector3d(10-0.5, 20.0, 30-1.5)))
        self.assertTrue(box.contains(Vector3d(10-0.5, 20.0, 30+1.5)))
        self.assertTrue(box.contains(Vector3d(10+0.5, 20.0, 30-1.5)))
        self.assertTrue(box.contains(Vector3d(10+0.5, 20.0, 30+1.5)))

        # Inside
        self.assertTrue(box.contains(Vector3d(10, 20, 30)))
        self.assertTrue(box.contains(Vector3d(10-0.25, 20-0.5, 30-0.75)))
        self.assertTrue(box.contains(Vector3d(10+0.25, 20+0.5, 30+0.75)))

        # Outside
        self.assertFalse(box.contains(Vector3d(10-1.0, 20-1.0, 30-1.5)))
        self.assertFalse(box.contains(Vector3d(10-0.5, 20-2.0, 30-1.5)))
        self.assertFalse(box.contains(Vector3d(10-0.5, 20-1.0, 30-2.0)))

        self.assertFalse(box.contains(Vector3d(10+1.0, 20+1.0, 30+1.5)))
        self.assertFalse(box.contains(Vector3d(10+0.5, 20+2.0, 30+1.5)))
        self.assertFalse(box.contains(Vector3d(10+0.5, 20+1.0, 30+2.0)))

    def test_contains_oriented_rotation(self):
        # Rotate PI/2 about +x: swap Z and Y
        box = OrientedBoxd(Vector3d(1, 2, 3), Pose3d(0, 0, 0, math.pi*0.5, 0, 0))

        # Doesn't contain non-rotated vertices
        self.assertFalse(box.contains(Vector3d(-0.5, -1.0, -1.5)))
        self.assertFalse(box.contains(Vector3d(-0.5, -1.0, +1.5)))
        self.assertFalse(box.contains(Vector3d(-0.5, +1.0, -1.5)))
        self.assertFalse(box.contains(Vector3d(-0.5, +1.0, +1.5)))

        self.assertFalse(box.contains(Vector3d(+0.5, -1.0, -1.5)))
        self.assertFalse(box.contains(Vector3d(+0.5, -1.0, +1.5)))
        self.assertFalse(box.contains(Vector3d(+0.5, +1.0, -1.5)))
        self.assertFalse(box.contains(Vector3d(+0.5, +1.0, +1.5)))

        # Inside
        self.assertTrue(box.contains(Vector3d(-0.5+g_tolerance,
                                              -1.5+g_tolerance,
                                              -1.0+g_tolerance)))
        self.assertTrue(box.contains(Vector3d(-0.5+g_tolerance,
                                              -1.5+g_tolerance,
                                              +1.0-g_tolerance)))
        self.assertTrue(box.contains(Vector3d(-0.5+g_tolerance,
                                              +1.5-g_tolerance,
                                              -1.0+g_tolerance)))
        self.assertTrue(box.contains(Vector3d(-0.5+g_tolerance,
                                              +1.5-g_tolerance,
                                              +1.0-g_tolerance)))

        self.assertTrue(box.contains(Vector3d(+0.5-g_tolerance,
                                              -1.5+g_tolerance,
                                              -1.0+g_tolerance)))
        self.assertTrue(box.contains(Vector3d(+0.5-g_tolerance,
                                              -1.5+g_tolerance,
                                              +1.0-g_tolerance)))
        self.assertTrue(box.contains(Vector3d(+0.5-g_tolerance,
                                              +1.5-g_tolerance, -1.0+g_tolerance)))
        self.assertTrue(box.contains(Vector3d(+0.5-g_tolerance,
                                              +1.5-g_tolerance,
                                              +1.0-g_tolerance)))

        # Outside
        self.assertFalse(box.contains(Vector3d(-0.5-g_tolerance,
                                               -1.5-g_tolerance,
                                               -1.0-g_tolerance)))
        self.assertFalse(box.contains(Vector3d(-0.5-g_tolerance,
                                               -1.5-g_tolerance,
                                               +1.0+g_tolerance)))
        self.assertFalse(box.contains(Vector3d(-0.5-g_tolerance,
                                               +1.5+g_tolerance,
                                               -1.0-g_tolerance)))
        self.assertFalse(box.contains(Vector3d(-0.5-g_tolerance,
                                               +1.5+g_tolerance,
                                               +1.0+g_tolerance)))

        self.assertFalse(box.contains(Vector3d(+0.5-g_tolerance,
                                               -1.5-g_tolerance,
                                               -1.0-g_tolerance)))
        self.assertFalse(box.contains(Vector3d(+0.5-g_tolerance,
                                               -1.5-g_tolerance,
                                               +1.0+g_tolerance)))
        self.assertFalse(box.contains(Vector3d(+0.5-g_tolerance,
                                               +1.5+g_tolerance,
                                               -1.0-g_tolerance)))
        self.assertFalse(box.contains(Vector3d(+0.5-g_tolerance,
                                               +1.5+g_tolerance,
                                               +1.0+g_tolerance)))

    def test_volume_and_density(self):
        mass = 1.0
        box = OrientedBoxd(Vector3d(1.0, 0.1, 10.4))
        expectedVolume = 1.0 * 0.1 * 10.4
        self.assertEqual(expectedVolume, box.volume())

        expectedDensity = mass / expectedVolume
        self.assertEqual(expectedDensity, box.density_from_mass(mass))

        # Bad density
        box2 = OrientedBoxd()
        self.assertGreater(0.0, box2.density_from_mass(mass))

    def test_mass(self):
        mass = 2.0
        length = 2.0
        w = 0.1
        h = 34.12
        box = OrientedBoxd(Vector3d(length, w, h))
        box.set_density_from_mass(mass)

        massMat = MassMatrix3d()
        ixx = (1.0/12.0) * mass * (w*w + h*h)
        iyy = (1.0/12.0) * mass * (length * length + h*h)
        izz = (1.0/12.0) * mass * (length * length + w*w)

        expectedMassMat = MassMatrix3d()
        expectedMassMat.set_inertia_matrix(ixx, iyy, izz, 0.0, 0.0, 0.0)
        expectedMassMat.set_mass(mass)

        box.mass_matrix(massMat)
        self.assertEqual(expectedMassMat, massMat)
        self.assertEqual(expectedMassMat.mass(), massMat.mass())


if __name__ == '__main__':
    unittest.main()
