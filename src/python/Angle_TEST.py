# Copyright (C) 2021 Open Source Robotics Foundation

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
import math
from ignition.math import Angle


class TestAngle(unittest.TestCase):

    def test_angle(self):

        angle1 = Angle()
        self.assertEqual(0.0, angle1.radian())

        angle1.set_degree(90.0)
        self.assertTrue(angle1 == Angle.HALF_PI)

        angle1.set_degree(180.0)
        self.assertTrue(angle1 == Angle.PI)
        self.assertFalse(angle1 == Angle.PI + Angle(0.1))
        self.assertTrue(angle1 == Angle.PI + Angle(0.0001))
        self.assertTrue(angle1 == Angle.PI - Angle(0.0001))
        self.assertTrue(Angle(0) == Angle(0))
        self.assertTrue(Angle(0) == Angle(0.001))

        angle1 = Angle(0.1) - Angle(0.3)
        self.assertAlmostEqual(angle1.radian(), -0.2)

        angle = Angle(0.5)
        self.assertEqual(0.5, angle.radian())

        angle.set_radian(math.pi/2)
        self.assertAlmostEqual(math.degrees(math.pi/2), angle.degree())

        angle.set_radian(math.pi)
        self.assertAlmostEqual(math.degrees(math.pi), angle.degree())

    def test_normalized_angles(self):

        angle = Angle(Angle.PI)
        normalized = angle.normalized()

        angle.normalized()
        self.assertEqual(math.degrees(math.pi), angle.degree())
        self.assertEqual(normalized, angle)

    def test_angle_operations(self):

        angle = Angle(0.1) + Angle(0.2)
        self.assertAlmostEqual(0.3, angle.radian())

        angle = Angle(0.1) * Angle(0.2)
        self.assertAlmostEqual(0.02, angle.radian())

        angle = Angle(0.1) / Angle(0.2)
        self.assertAlmostEqual(0.5, angle.radian())

        angle -= Angle(0.1)
        self.assertAlmostEqual(0.4, angle.radian())

        angle += Angle(0.2)
        self.assertAlmostEqual(0.6, angle.radian())

        angle *= Angle(0.5)
        self.assertAlmostEqual(0.3, angle.radian())

        angle /= Angle(0.1)
        self.assertAlmostEqual(3.0, angle.radian())
        self.assertTrue(angle == Angle(3))
        self.assertTrue(angle != Angle(2))
        self.assertTrue(angle < Angle(4))
        self.assertTrue(angle > Angle(2))
        self.assertTrue(angle >= Angle(3))
        self.assertTrue(angle <= Angle(3))


if __name__ == '__main__':
    unittest.main()
