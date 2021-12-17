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

import math
import unittest
from ignition.math import Spline
from ignition.math import Vector3d


class TestSpline(unittest.TestCase):

    def test_spline(self):
        s = Spline()

        s.add_point(Vector3d(0, 0, 0))
        self.assertEqual(1, s.point_count())

        s.clear()
        self.assertEqual(0, s.point_count())

        s.add_point(Vector3d(0, 0, 0))
        self.assertTrue(s.point(0) == Vector3d(0, 0, 0))
        s.add_point(Vector3d(1, 1, 1))
        self.assertTrue(s.point(1) == Vector3d(1, 1, 1))

        # update_point
        self.assertFalse(s.update_point(2, Vector3d(2, 2, 2)))

        self.assertTrue(s.update_point(1, Vector3d(2, 2, 2)))
        s.auto_calculate(False)
        self.assertTrue(s.update_point(0, Vector3d(-1, -1, -1)))
        s.auto_calculate(True)

        # interpolate
        self.assertAlmostEqual(s.interpolate(0.0), Vector3d(-1, -1, -1))
        self.assertAlmostEqual(s.interpolate(0.5), Vector3d(0.5, 0.5, 0.5))
        self.assertAlmostEqual(s.interpolate(1.0), Vector3d(2, 2, 2))

        # interpolate
        s.add_point(Vector3d(4, 4, 4))
        self.assertAlmostEqual(s.interpolate(1, 0.2),
                               Vector3d(2.496, 2.496, 2.496))

    def test_fixed_tangent(self):
        s = Spline()

        # add_point
        s.auto_calculate(False)
        s.add_point(Vector3d(0, 0, 0))
        s.add_point(Vector3d(0, 0.5, 0), Vector3d(0, 1, 0))
        s.add_point(Vector3d(0.5, 1, 0), Vector3d(1, 0, 0))
        s.add_point(Vector3d(1, 1, 0), Vector3d(1, 0, 0))

        # update_point
        s.update_point(0, Vector3d(0, 0, 0), Vector3d(0, 1, 0))

        s.auto_calculate(True)

        s.recalc_tangents()

        # interpolate
        self.assertAlmostEqual(s.interpolate(0, 0.5), Vector3d(0, 0.25, 0))
        self.assertAlmostEqual(s.interpolate_tangent(0, 0.5),
                               Vector3d(0, 0.25, 0))
        self.assertAlmostEqual(s.interpolate(1, 0.5),
                               Vector3d(0.125, 0.875, 0))
        self.assertAlmostEqual(s.interpolate(2, 0.5), Vector3d(0.75, 1, 0))
        self.assertAlmostEqual(s.interpolate_tangent(2, 0.5),
                               Vector3d(0.25, 0, 0))

    def test_arc_length(self):
        s = Spline()
        self.assertFalse(math.isfinite(s.arc_length()))
        s.add_point(Vector3d(1, 1, 1), Vector3d(1, 1, 1))
        self.assertFalse(math.isfinite(s.arc_length(0)))
        s.add_point(Vector3d(3, 3, 3), Vector3d(1, 1, 1))
        s.add_point(Vector3d(4, 4, 4), Vector3d(1, 1, 1))
        self.assertAlmostEqual(s.arc_length(0, 1.0),
                               3.46410161513775, delta=1e-14)
        self.assertAlmostEqual(s.arc_length(), 5.19615242270663, delta=1e-14)
        self.assertAlmostEqual(s.arc_length(), s.arc_length(1.0))
        self.assertFalse(math.isfinite(s.arc_length(-1.0)))
        self.assertFalse(math.isfinite(s.arc_length(4, 0.0)))

    def test_tension(self):
        s = Spline()

        s.tension(0.1)
        self.assertAlmostEqual(s.tension(), 0.1)

    def test_interpolate(self):
        s = Spline()
        self.assertFalse(s.interpolate(0, 0.1).is_finite())

        s.add_point(Vector3d(0, 0, 0))
        self.assertAlmostEqual(s.interpolate(0, 0.1), Vector3d(0, 0, 0))
        self.assertFalse(s.interpolate_tangent(0.1).is_finite())

        s.add_point(Vector3d(1, 2, 3))
        self.assertAlmostEqual(s.interpolate(0, 0.0), s.point(0))
        self.assertFalse(s.interpolate(0, -0.1).is_finite())
        self.assertAlmostEqual(s.interpolate_tangent(0, 0.0), s.tangent(0))

        # Fast and slow call variations
        self.assertAlmostEqual(s.interpolate(0, 0.5), Vector3d(0.5, 1.0, 1.5))
        self.assertAlmostEqual(s.interpolate(0, 1.0), s.point(1))
        self.assertAlmostEqual(s.interpolate_tangent(0, 0.5),
                               Vector3d(1.25, 2.5, 3.75))
        self.assertAlmostEqual(s.interpolate_tangent(0, 1.0), s.tangent(1))
        self.assertAlmostEqual(s.interpolate_mth_derivative(2, 0.5),
                               Vector3d(0, 0, 0))
        self.assertAlmostEqual(s.interpolate_mth_derivative(2, 1.0),
                               Vector3d(-3, -6, -9))
        self.assertAlmostEqual(s.interpolate_mth_derivative(3, 0.5),
                               Vector3d(-6, -12, -18))
        self.assertAlmostEqual(s.interpolate_mth_derivative(3, 1.0),
                               Vector3d(-6, -12, -18))
        self.assertAlmostEqual(s.interpolate_mth_derivative(4, 0.5),
                               Vector3d(0, 0, 0))
        self.assertAlmostEqual(s.interpolate_mth_derivative(4, 1.0),
                               Vector3d(0, 0, 0))

    def test_point(self):
        s = Spline()
        self.assertFalse(s.point(0).is_finite())

    def test_tangent(self):
        s = Spline()
        self.assertFalse(s.tangent(0).is_finite())

        s.add_point(Vector3d(0, 0, 0))
        self.assertFalse(s.tangent(0).is_finite())

        s.add_point(Vector3d(1, 0, 0))
        self.assertAlmostEqual(s.tangent(0), Vector3d(0.5, 0, 0))

        s.add_point(Vector3d(1, 1, 0), Vector3d(-1, 1, 0))
        self.assertAlmostEqual(s.tangent(1), Vector3d(0.5, 0.5, 0))
        self.assertAlmostEqual(s.tangent(2), Vector3d(-1, 1, 0))

    def test_recalc_tangents(self):
        s = Spline()

        s.add_point(Vector3d(0, 0, 0))
        s.add_point(Vector3d(.4, .4, .4))
        s.add_point(Vector3d(0, 0, 0))

        s.recalc_tangents()

        self.assertAlmostEqual(s.interpolate(0, 0.5), Vector3d(0.2, 0.2, 0.2))
        self.assertAlmostEqual(s.interpolate(1, 0.5), Vector3d(0.2, 0.2, 0.2))


if __name__ == '__main__':
    unittest.main()
