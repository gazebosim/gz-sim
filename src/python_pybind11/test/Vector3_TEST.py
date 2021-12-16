# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
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

import copy
import math
import unittest

from ignition.math import Vector3d
from ignition.math import Vector3f


class TestVector3(unittest.TestCase):

    def test_construction(self):
        v = Vector3d()
        self.assertAlmostEqual(0.0, v.x())
        self.assertAlmostEqual(0.0, v.y())
        self.assertAlmostEqual(0.0, v.z())

        vec = Vector3d(1, 0, 0)
        self.assertEqual(vec.x(), 1)
        self.assertEqual(vec.y(), 0)
        self.assertEqual(vec.z(), 0)

        vec2 = Vector3d(vec)
        self.assertEqual(vec2, vec)

        # Copy
        vec3 = vec
        self.assertEqual(vec3, vec)

        # Inequality
        vec4 = Vector3d()
        self.assertNotEqual(vec, vec4)

    def test_vector3(self):
        # Distance, length()
        v = Vector3d(1, 2, 3)
        self.assertEqual(v.length(), v.distance(Vector3d.ZERO))

        # Rounded
        v.set(1.23, 2.34, 3.55)
        self.assertEqual(v.rounded(), Vector3d(1, 2, 4))

        # Round
        v.round()
        self.assertEqual(v.round(), Vector3d(1, 2, 4))

        # DotProd
        self.assertEqual(v.dot(Vector3d(1, 2, 3)), 17.0)

        # DistToLine
        v.set(0, 0, 0)
        self.assertEqual(1.0, v.dist_to_line(Vector3d(1, -1, 0),
                         Vector3d(1, 1, 0)))

        # __truediv__
        v.set(4, 4, 4)
        v = v / Vector3d(1, 2, 4)
        self.assertEqual(v, Vector3d(4, 2, 1))

        # __truediv__ int
        v = v / 2
        self.assertEqual(v, Vector3d(2, 1, 0.5))

        # __mul__
        v = v * Vector3d(2, 3, 4)
        self.assertEqual(v, Vector3d(4, 3, 2))

        v.set(1.23, 2.35, 3.654321)
        v.round(1)
        self.assertEqual(v, Vector3d(1.2, 2.4, 3.7))

        # Abs
        v.set(-1, -2, -3)
        self.assertEqual(v.abs(), Vector3d(1, 2, 3))

        # __truediv__
        v.set(1, 2, 4)
        v /= Vector3d(1, 4, 4)
        self.assertEqual(v, Vector3d(1, 0.5, 1))

        # __mul__
        v.set(1, 2, 4)
        v *= Vector3d(2, 0.5, 0.1)
        self.assertTrue(v.equal(Vector3d(2, 1, 0.4)))

        # Test the static defines.
        self.assertEqual(Vector3d.ZERO, Vector3d(0, 0, 0))

        self.assertEqual(Vector3d.ONE, Vector3d(1, 1, 1))

        self.assertEqual(Vector3d.UNIT_X, Vector3d(1, 0, 0))

        self.assertEqual(Vector3d.UNIT_Y, Vector3d(0, 1, 0))

        self.assertEqual(Vector3d.UNIT_Z, Vector3d(0, 0, 1))

    def test_distance(self):
        vec1 = Vector3d(0, 0, 0)
        vec2 = Vector3d(1, 2, 3)

        dist = vec1.distance(vec2)
        self.assertTrue(abs(dist - 3.74165738677) < 1e-6)

        dist2 = vec1.distance(1, 2, 3)
        self.assertEqual(dist, dist2)

    def test_sum(self):
        vec1 = Vector3d(0, 0, 0)
        vec2 = Vector3d(1, 2, 3)

        sum1 = vec1.sum()
        sum2 = vec2.sum()

        self.assertEqual(sum1, 0)
        self.assertEqual(sum2, 6)

    def test_squared_length(self):
        vec1 = Vector3d(0, 0, 0)
        vec2 = Vector3d(1, 2, 3)

        sum1 = vec1.squared_length()
        sum2 = vec2.squared_length()

        self.assertEqual(sum1, 0)
        self.assertEqual(sum2, 14)

    def test_length(self):
        # Zero vector
        self.assertEqual(Vector3d.ZERO.length(), 0.0)
        self.assertEqual(Vector3d.ZERO.squared_length(), 0.0)

        # UnitXYZ vectorsIgnition::
        self.assertEqual(Vector3d.UNIT_X.length(), 1.0)
        self.assertEqual(Vector3d.UNIT_Y.length(), 1.0)
        self.assertEqual(Vector3d.UNIT_Z.length(), 1.0)
        self.assertEqual(Vector3d.UNIT_X.squared_length(), 1.0)
        self.assertEqual(Vector3d.UNIT_Y.squared_length(), 1.0)
        self.assertEqual(Vector3d.UNIT_Z.squared_length(), 1.0)

        # One vector
        self.assertTrue(Vector3d.ONE.length() -
                        abs(math.sqrt(3.0)) < 1e-10)

        self.assertEqual(Vector3d.ONE.squared_length(), 3.0)

        # Arbitrary vector
        v = Vector3d(0.1, -4.2, 2.5)
        self.assertTrue(abs(v.length() - 4.88876262463) < 1e-10)

        self.assertTrue(abs(v.squared_length() - 23.9) < 1e-10)

    def test_normalize(self):
        vec1 = Vector3d(0, 0, 0)
        vec2 = Vector3d(1, 2, 3)

        vec3 = vec1.normalize()
        self.assertEqual(vec3, vec1)
        self.assertEqual(vec1, Vector3d.ZERO)

        vec3 = vec2.normalize()
        self.assertEqual(vec3, vec2)
        self.assertEqual(vec2, Vector3d(0.267261, 0.534522, 0.801784))

        vecConst = Vector3d(1, 2, 3)
        self.assertEqual(vecConst.normalized(), vec3)
        self.assertEqual(vecConst, Vector3d(1, 2, 3))

    def test_ge_normal(self):
        vec1 = Vector3d(0, 0, 0)
        vec2 = Vector3d(0, 1, 0)
        vec3 = Vector3d(1, 1, 0)

        norm = Vector3d.normal(vec1, vec2, vec3)
        self.assertEqual(norm, Vector3d(0, 0, -1))

    def test_perpendicular(self):
        vec1 = Vector3d(1, 1, 0)
        vec2 = Vector3d(0, 1, 1)
        vec3 = Vector3d(1e-7, 1e-7, 1e-7)
        vec4 = Vector3d(1, 0, 0)

        self.assertEqual(vec1.perpendicular(), Vector3d(0, 0, -1))
        self.assertEqual(vec2.perpendicular(), Vector3d(0, 1, -1))
        self.assertEqual(vec3.perpendicular(), Vector3d(0, 0, 0))
        self.assertEqual(vec4.perpendicular(), Vector3d(0, 0, 1))

    def test_max(self):
        vec1 = Vector3d(0.1, 0.2, 0.3)
        vec2 = Vector3d(0.2, 0.3, 0.4)
        vec3 = Vector3d(0.1, 0.2, 0.3)

        self.assertTrue(abs(vec1.max() - 0.3) < 1e-10)

        vec1.max(vec2)
        self.assertEqual(vec1, Vector3d(0.2, 0.3, 0.4))

        vec1.max(vec3)
        self.assertEqual(vec1, Vector3d(0.2, 0.3, 0.4))

    def test_min(self):
        vec1 = Vector3d(0.1, 0.2, 0.3)
        vec2 = Vector3d(0.2, 0.3, 0.4)
        vec3 = Vector3d(0.05, 0.1, 0.2)

        self.assertTrue(abs(vec1.min() - 0.1) < 1e-10)

        vec1.min(vec2)
        self.assertEqual(vec1, Vector3d(0.1, 0.2, 0.3))

        vec1.min(vec3)
        self.assertEqual(vec1, Vector3d(0.05, 0.1, 0.2))

    def test_add(self):
        vec1 = Vector3d(0.1, 0.2, 0.4)
        vec2 = Vector3d(1.1, 2.2, 3.4)

        vec3 = copy.deepcopy(vec1)
        vec3 += vec2

        self.assertEqual(vec1 + vec2, Vector3d(1.2, 2.4, 3.8))
        self.assertEqual(vec3, Vector3d(1.2, 2.4, 3.8))

        # Add zeros
        # Scalar right
        self.assertEqual(vec1 + 0, vec1)

        # Vector left and right
        self.assertEqual(Vector3d.ZERO + vec1, vec1)
        self.assertEqual(vec1 + Vector3d.ZERO, vec1)

        # Addition assignment
        vec4 = vec1
        vec4 += 0
        self.assertEqual(vec4, vec1)
        vec4 += Vector3d.ZERO
        self.assertEqual(vec4, vec1)

        # Add non-trivial scalar values left and right
        self.assertEqual(vec1 + 2.5, Vector3d(2.6, 2.7, 2.9))

        vec1 = vec4
        vec4 += 2.5
        self.assertEqual(vec4, Vector3d(2.6, 2.7, 2.9))

    def test_sub(self):
        vec1 = Vector3d(0.1, 0.2, 0.4)
        vec2 = Vector3d(1.1, 2.2, 3.4)

        vec3 = copy.deepcopy(vec2)
        vec3 -= vec1

        self.assertEqual(vec2 - vec1, Vector3d(1.0, 2.0, 3.0))
        self.assertEqual(vec3, Vector3d(1.0, 2.0, 3.0))

        # Subtraction with zeros
        # Scalar right
        self.assertEqual(vec1 - 0, vec1)

        # Vector left and right
        self.assertEqual(Vector3d.ZERO - vec1, -vec1)
        self.assertEqual(vec1 - Vector3d.ZERO, vec1)

        # Subtraction assignment
        vec4 = vec1
        vec4 -= 0
        self.assertEqual(vec4, vec1)
        vec4 -= Vector3d.ZERO
        self.assertEqual(vec4, vec1)

        # Subtract non-trivial scalar values left and right
        self.assertEqual(vec1 - 2.5, -Vector3d(2.4, 2.3, 2.1))

        vec4 = vec1
        vec4 -= 2.5
        self.assertEqual(vec4, -Vector3d(2.4, 2.3, 2.1))

    def test_divide(self):
        vec1 = Vector3d(0.1, 0.2, 0.4)

        vec3 = vec1 / 2.0
        self.assertEqual(vec3, Vector3d(0.05, 0.1, 0.2))

        vec3 /= 4.0
        self.assertEqual(vec3, Vector3d(0.0125, 0.025, 0.05))

    def test_multiply(self):
        v = Vector3d(0.1, 0.2, 0.3)

        vec2 = v * 2.0
        self.assertEqual(vec2, Vector3d(0.2, 0.4, 0.6))

        vec2 *= 4.0
        self.assertEqual(vec2, Vector3d(0.8, 1.6, 2.4))

        # Multiply by zero
        # Scalar right
        self.assertEqual(v * 0, Vector3d.ZERO)

        # Element-wise vector multiplication
        self.assertEqual(v * Vector3d.ZERO, Vector3d.ZERO)

        # Multiply by one
        # Scalar right
        self.assertEqual(v * 1, v)

        # Element-wise vector multiplication
        self.assertEqual(v * Vector3d.ONE, v)

        # Multiply by non-trivial scalar value
        scalar = 2.5
        expect = Vector3d(0.25, 0.5, 0.75)
        self.assertEqual(v * scalar, expect)

        # Multiply by itself element-wise
        self.assertEqual(v*v, Vector3d(0.01, 0.04, 0.09))

    def test_not_equal(self):
        vec1 = Vector3d(0.1, 0.2, 0.3)
        vec2 = Vector3d(0.2, 0.2, 0.3)
        vec3 = Vector3d(0.1, 0.2, 0.3)

        self.assertTrue(vec1 != vec2)
        self.assertTrue(not(vec1 != vec3))

    def test_equal(self):
        self.assertTrue(not Vector3d.ZERO.equal(
          Vector3d.ONE, 1e-6))
        self.assertTrue(not Vector3d.ZERO.equal(
          Vector3d.ONE, 1e-3))
        self.assertTrue(not Vector3d.ZERO.equal(
          Vector3d.ONE, 1e-1))

        self.assertTrue(Vector3d.ZERO.equal(
            Vector3d.ONE, 1))
        self.assertTrue(Vector3d.ZERO.equal(
            Vector3d.ONE, 1.1))

    def test_finite(self):
        vec1 = Vector3d(0.1, 0.2, 0.3)

        self.assertTrue(vec1.is_finite())

    def test_nan(self):
        nanVec = Vector3d.NAN
        self.assertFalse(nanVec.is_finite())
        self.assertTrue(math.isnan(nanVec.x()))
        self.assertTrue(math.isnan(nanVec.y()))
        self.assertTrue(math.isnan(nanVec.z()))

        nanVec.correct()
        self.assertEqual(Vector3d.ZERO, nanVec)
        self.assertTrue(nanVec.is_finite())

        nanVecF = Vector3f.NAN
        self.assertFalse(nanVecF.is_finite())
        self.assertTrue(math.isnan(nanVecF.x()))
        self.assertTrue(math.isnan(nanVecF.y()))
        self.assertTrue(math.isnan(nanVecF.z()))

        nanVecF.correct()
        self.assertEqual(Vector3f.ZERO, nanVecF)
        self.assertTrue(nanVecF.is_finite())

if __name__ == '__main__':
    unittest.main()
