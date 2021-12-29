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
from ignition.math import Quaterniond
from ignition.math import RotationSpline
from ignition.math import Vector3d


class TestRotationSpline(unittest.TestCase):
    def test_rotation_spline(self):
        s = RotationSpline()

        s.add_point(Quaterniond(0, 0, 0))
        self.assertEqual(1, s.point_count())

        s.clear()
        self.assertEqual(0, s.point_count())

        s.add_point(Quaterniond(0, 0, 0))
        self.assertTrue(s.point(0) == Quaterniond(0, 0, 0))
        s.add_point(Quaterniond(.1, .1, .1))
        self.assertTrue(s.point(1) == Quaterniond(.1, .1, .1))

        # update_point
        self.assertFalse(s.update_point(2, Quaterniond(.2, .2, .2)))

        self.assertTrue(s.update_point(1, Quaterniond(.2, .2, .2)))
        s.auto_calculate(False)
        self.assertTrue(s.update_point(0, Quaterniond(
                        Vector3d(-.1, -.1, -.1))))
        s.auto_calculate(True)

        # interpolate
        self.assertTrue(s.interpolate(0.5) ==
                        Quaterniond(0.998089, 0.0315333, 0.0427683, 0.0315333))

        # interpolate
        s.add_point(Quaterniond(.4, .4, .4))
        self.assertFalse(s.interpolate(4, 0.2).is_finite())

        self.assertEqual(s.interpolate(s.point_count()-1, 0.2),
                         s.point(s.point_count()-1))
        self.assertTrue(s.interpolate(1, 0.2) ==
                        Quaterniond(0.978787, 0.107618, 0.137159, 0.107618))
        self.assertEqual(s.interpolate(1, 0.0), s.point(1))
        self.assertEqual(s.interpolate(1, 1.0), s.point(2))

    def test_get_point(self):
        s = RotationSpline()
        self.assertFalse(s.point(0).is_finite())
        self.assertFalse(s.point(1).is_finite())

        s.add_point(Quaterniond(0, 0, 0))
        self.assertTrue(s.point(1).is_finite())

    def test_recalc_tangents(self):
        s = RotationSpline()
        s.add_point(Quaterniond(0, 0, 0))
        s.add_point(Quaterniond(.4, .4, .4))
        s.add_point(Quaterniond(0, 0, 0))

        s.recalc_tangents()
        self.assertEqual(s.interpolate(0, 0.5),
                         Quaterniond(0.987225, 0.077057, 0.11624, 0.077057))

        self.assertEqual(s.interpolate(1, 0.5),
                         Quaterniond(0.987225, 0.077057, 0.11624, 0.077057))


if __name__ == '__main__':
    unittest.main()
