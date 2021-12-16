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

from ignition.math import Vector4d
from ignition.math import Vector4f


class TestVector4(unittest.TestCase):

    def test_construction(self):
        v = Vector4d()
        self.assertAlmostEqual(0.0, v.x())
        self.assertAlmostEqual(0.0, v.y())
        self.assertAlmostEqual(0.0, v.z())
        self.assertAlmostEqual(0.0, v.w())

        vec = Vector4d(1, 0, 0, 0)
        self.assertEqual(vec.x(), 1)
        self.assertEqual(vec.y(), 0)
        self.assertEqual(vec.z(), 0)
        self.assertEqual(vec.w(), 0)

        vec2 = Vector4d(vec)
        self.assertEqual(vec2, vec)

        # Copy
        vec3 = vec
        self.assertEqual(vec3, vec)

        # Inequality
        vec4 = Vector4d()
        self.assertNotEqual(vec, vec4)

    def test_vector4(self):
        v = Vector4d()
        # Distance, length()
        v.set(1, 2, 3, 4)
        self.assertEqual(v.length(), v.distance(Vector4d.ZERO))

        # __truediv__
        v.set(4, 4, 4, 4)
        v = v / Vector4d(1, 2, 2, 4)
        self.assertEqual(v, Vector4d(4, 2, 2, 1))

        # __truediv__ int
        v = v / 2
        self.assertEqual(v, Vector4d(2, 1, 1, 0.5))

        # __mul__
        v = v * Vector4d(2, 3, 3, 4)
        self.assertEqual(v, Vector4d(4, 3, 3, 2))

        # __truediv__
        v.set(1, 2, 2, 4)
        v /= Vector4d(1, 4, 8, 4)
        self.assertEqual(v, Vector4d(1, 0.5, 0.25, 1))

        # __mul__
        v.set(1, 2, 2, 4)
        v *= Vector4d(2, 0.5, 0.25, 0.1)
        self.assertEqual(v, Vector4d(2, 1, 0.5, 0.4))

        # Test the static defines.
        self.assertEqual(Vector4d.ZERO,
                         Vector4d(0, 0, 0, 0))

        self.assertEqual(Vector4d.ONE,
                         Vector4d(1, 1, 1, 1))

    def test_distance(self):
        vec1 = Vector4d(0, 0, 0, 0)
        vec2 = Vector4d(1, 2, 3, 4)

        dist = vec1.distance(vec2)
        self.assertTrue(abs(dist - 5.47722557505) < 1e-6)

    def test_squared_length(self):
        vec1 = Vector4d(0, 0, 0, 0)
        vec2 = Vector4d(1, 2, 3, 4)

        sum1 = vec1.squared_length()
        sum2 = vec2.squared_length()

        self.assertEqual(sum1, 0)
        self.assertEqual(sum2, 30)

    def test_length(self):
        # Zero vector
        self.assertEqual(Vector4d.ZERO.length(), 0.0)
        self.assertEqual(Vector4d.ZERO.squared_length(), 0.0)

        # One vector
        self.assertTrue(abs(Vector4d.ONE.length() - math.sqrt(4.0)) < 1e-10)

        self.assertEqual(Vector4d.ONE.squared_length(), 4.0)

        # Arbitrary vector
        v = Vector4d(0.1, -4.2, 2.5, -1.2)
        self.assertTrue(abs(v.length() - 5.03388517946) < 1e-10)

        self.assertTrue(abs(v.squared_length() - 25.34) < 1e-10)

    def test_normalize(self):
        vec1 = Vector4d(0, 0, 0, 0)
        vec2 = Vector4d(1, 2, 3, 4)

        vec3 = vec1
        vec3.normalize()
        self.assertEqual(vec3, vec1)
        self.assertEqual(vec1, Vector4d.ZERO)

        vec3 = vec2
        vec2.normalize()
        self.assertTrue(vec2.equal(Vector4d(0.182575,
                        0.365150, 0.547725, 0.730300), 1e-5))

    def test_add(self):
        vec1 = Vector4d(0.1, 0.2, 0.4, 0.8)
        vec2 = Vector4d(1.1, 2.2, 3.4, 4.3)

        vec3 = copy.deepcopy(vec1)
        vec3 += vec2

        self.assertEqual(vec1 + vec2, Vector4d(1.2, 2.4, 3.8, 5.1))
        self.assertEqual(vec3, Vector4d(1.2, 2.4, 3.8, 5.1))

        # Addition with zeros
        # Scalar right
        self.assertEqual(vec1 + 0, vec1)

        # Vector left and right
        self.assertEqual(Vector4d.ZERO + vec1, vec1)
        self.assertEqual(vec1 + Vector4d.ZERO, vec1)

        # Addition assignment
        vec4 = vec1
        vec4 += 0
        self.assertEqual(vec4, vec1)
        vec4 += Vector4d.ZERO
        self.assertEqual(vec4, vec1)

        # Add non-trivial scalar values left and right
        self.assertEqual(vec1 + 2.5, Vector4d(2.6, 2.7, 2.9, 3.3))

        vec1 = vec4
        vec4 += 2.5
        self.assertEqual(vec4, Vector4d(2.6, 2.7, 2.9, 3.3))

    def test_sub(self):
        vec1 = Vector4d(0.1, 0.2, 0.4, 0.8)
        vec2 = Vector4d(1.1, 2.2, 3.4, 4.3)

        vec3 = copy.deepcopy(vec2)
        vec3 -= vec1

        self.assertEqual(vec2 - vec1, Vector4d(1.0, 2.0, 3.0, 3.5))
        self.assertEqual(vec3, Vector4d(1.0, 2.0, 3.0, 3.5))

        # Subtraction with zeros
        # Scalar right
        self.assertEqual(vec1 - 0, vec1)

        # Vector left and right
        self.assertEqual(Vector4d.ZERO - vec1, -vec1)
        self.assertEqual(vec1 - Vector4d.ZERO, vec1)

        # Subtraction assignment
        vec4 = vec1
        vec4 -= 0
        self.assertEqual(vec4, vec1)
        vec4 -= Vector4d.ZERO
        self.assertEqual(vec4, vec1)

        # Subtract non-trivial scalar values left and right
        self.assertEqual(vec1 - 2.5, -Vector4d(2.4, 2.3, 2.1, 1.7))

        vec4 = vec1
        vec4 -= 2.5
        self.assertEqual(vec4, -Vector4d(2.4, 2.3, 2.1, 1.7))

    def test_divide(self):
        vec1 = Vector4d(0.1, 0.2, 0.4, 0.8)

        vec3 = vec1 / 2.0
        self.assertEqual(vec3, Vector4d(0.05, 0.1, 0.2, 0.4))

        vec3 /= 4.0
        self.assertEqual(vec3, Vector4d(0.0125, 0.025, 0.05, 0.1))

    def test_multiply(self):
        v = Vector4d(0.1, 0.2, 0.3, 0.4)

        vec3 = v * 2.0
        self.assertEqual(vec3, Vector4d(0.2, 0.4, 0.6, 0.8))

        vec3 *= 4.0
        self.assertEqual(vec3, Vector4d(0.8, 1.6, 2.4, 3.2))

        # Multiply by zero
        # Scalar right
        self.assertEqual(v * 0, Vector4d.ZERO)

        # Element-wise vector multiplication
        self.assertEqual(v * Vector4d.ZERO, Vector4d.ZERO)

        # Multiply by one
        # Scalar right
        self.assertEqual(v * 1, v)

        # Element-wise vector multiplication
        self.assertEqual(v * Vector4d.ONE, v,)

        # Multiply by non-trivial scalar value
        scalar = 2.5
        expect = Vector4d(0.25, 0.5, 0.75, 1.0)
        self.assertEqual(v * scalar, expect)

        # Multiply by itself element-wise
        self.assertEqual(v*v, Vector4d(0.01, 0.04, 0.09, 0.16))

    def test_not_equal(self):
        vec1 = Vector4d(0.1, 0.2, 0.3, 0.4)
        vec2 = Vector4d(0.2, 0.2, 0.3, 0.4)
        vec3 = Vector4d(0.1, 0.2, 0.3, 0.4)

        self.assertTrue(vec1 != vec2)
        self.assertTrue(not(vec1 != vec3))

    def test_equal(self):
        self.assertTrue(not Vector4d.ZERO.equal(
                        Vector4d.ONE, 1e-6))
        self.assertTrue(not Vector4d.ZERO.equal(
                        Vector4d.ONE, 1e-3))
        self.assertTrue(not Vector4d.ZERO.equal(
                        Vector4d.ONE, 1e-1))

        self.assertTrue(Vector4d.ZERO.equal(
            Vector4d.ONE, 1))
        self.assertTrue(Vector4d.ZERO.equal(
            Vector4d.ONE, 1.1))

    def test_finite(self):
        vec1 = Vector4d(0.1, 0.2, 0.3, 0.4)
        self.assertTrue(vec1.is_finite())

    def test_nan(self):
        nanVec = Vector4d.NAN
        self.assertFalse(nanVec.is_finite())
        self.assertTrue(math.isnan(nanVec.x()))
        self.assertTrue(math.isnan(nanVec.y()))
        self.assertTrue(math.isnan(nanVec.z()))
        self.assertTrue(math.isnan(nanVec.w()))

        nanVec.correct()
        self.assertEqual(Vector4d.ZERO, nanVec)
        self.assertTrue(nanVec.is_finite())

        nanVecF = Vector4f.NAN
        self.assertFalse(nanVecF.is_finite())
        self.assertTrue(math.isnan(nanVecF.x()))
        self.assertTrue(math.isnan(nanVecF.y()))
        self.assertTrue(math.isnan(nanVecF.z()))
        self.assertTrue(math.isnan(nanVecF.w()))

        nanVecF.correct()
        self.assertEqual(Vector4f.ZERO, nanVecF)
        self.assertTrue(nanVecF.is_finite())

if __name__ == '__main__':
    unittest.main()
