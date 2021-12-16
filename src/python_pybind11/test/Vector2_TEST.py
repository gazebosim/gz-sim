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

from ignition.math import Vector2d
from ignition.math import Vector2f


class TestVector2(unittest.TestCase):

    def test_construction(self):
        v = Vector2d()
        self.assertAlmostEqual(0.0, v.x())
        self.assertAlmostEqual(0.0, v.y())

        vec = Vector2d(1, 0)
        self.assertEqual(vec.x(), 1)
        self.assertEqual(vec.y(), 0)

        vec2 = Vector2d(vec)
        self.assertEqual(vec2, vec)

        # Copy
        vec3 = vec
        self.assertEqual(vec3, vec)

        # Inequality
        vec4 = Vector2d()
        self.assertNotEqual(vec, vec4)

    def test_vector2(self):
        v = Vector2d(1, 2)

        # Distance
        self.assertAlmostEqual(2.236, v.distance(Vector2d()), delta=1e-2)

        # Normalize
        v.normalize()
        self.assertTrue(v.equal(Vector2d(0.447214, 0.894427), 1e-4))

        # Set
        v.set(4, 5)
        self.assertTrue(v.equal(Vector2d(4, 5), 1e-4))

        # Abs
        v.set(-1, -2)
        self.assertTrue(v.abs().equal(Vector2d(1, 2), 1e-4))

        # _eq_
        v = Vector2d(6, 7)
        self.assertTrue(v.equal(Vector2d(6, 7), 1e-4))

        # _add_
        v = v + Vector2d(1, 2)
        self.assertTrue(v.equal(Vector2d(7, 9), 1e-4))

        v += Vector2d(5, 6)
        self.assertTrue(v.equal(Vector2d(12, 15), 1e-4))

        # __sub__
        v = v - Vector2d(2, 4)
        self.assertTrue(v.equal(Vector2d(10, 11), 1e-4))

        v.set(2, 4)
        v -= Vector2d(1, 6)
        self.assertTrue(v.equal(Vector2d(1, -2), 1e-4))

        # __truediv__
        v.set(10, 6)
        v = v / Vector2d(2, 3)
        self.assertTrue(v.equal(Vector2d(5, 2), 1e-4))

        v.set(10, 6)
        v /= Vector2d(2, 3)
        self.assertTrue(v.equal(Vector2d(5, 2), 1e-4))

        # __truediv__ int
        v.set(10, 6)
        v = v / 2
        self.assertTrue(v.equal(Vector2d(5, 3), 1e-4))

        v.set(10, 6)
        v /= 2
        self.assertTrue(v.equal(Vector2d(5, 3), 1e-4))

        # __mul__
        v.set(10, 6)
        v = v * Vector2d(2, 4)
        self.assertTrue(v.equal(Vector2d(20, 24), 1e-4))

        v.set(10, 6)
        v *= Vector2d(2, 4)
        self.assertTrue(v.equal(Vector2d(20, 24), 1e-4))

        # __mul__ int
        v.set(10, 6)
        v = v * 2
        self.assertTrue(v.equal(Vector2d(20, 12), 1e-4))

        v.set(10, 6)
        v *= 2
        self.assertTrue(v.equal(Vector2d(20, 12), 1e-4))

        # is_finite
        self.assertTrue(v.is_finite())

    def test_max(self):
        vec1 = Vector2d(0.1, 0.2)
        vec2 = Vector2d(0.3, 0.5)
        vec3 = Vector2d(0.4, 0.2)

        self.assertAlmostEqual(vec1.max(), 0.2)
        self.assertAlmostEqual(vec3.max(), 0.4)

        vec1.max(vec2)
        self.assertAlmostEqual(vec1, Vector2d(0.3, 0.5))

        vec1.max(vec3)
        self.assertAlmostEqual(vec1, Vector2d(0.4, 0.5))

    def test_min(self):
        vec1 = Vector2d(0.3, 0.5)
        vec2 = Vector2d(0.1, 0.2)
        vec3 = Vector2d(0.05, 0.1)

        self.assertAlmostEqual(vec1.min(), 0.3)
        self.assertAlmostEqual(vec3.min(), 0.05)

        vec1.min(vec2)
        self.assertAlmostEqual(vec1, Vector2d(0.1, 0.2))

        vec1.min(vec3)
        self.assertAlmostEqual(vec1, Vector2d(0.05, 0.1))

    def test_equal_tolerance(self):
        # Test Equal function with specified tolerance
        self.assertFalse(Vector2d.ZERO.equal(Vector2d.ONE, 1e-6))
        self.assertFalse(Vector2d.ZERO.equal(Vector2d.ONE, 1e-3))
        self.assertFalse(Vector2d.ZERO.equal(Vector2d.ONE, 1e-1))
        self.assertTrue(Vector2d.ZERO.equal(Vector2d.ONE, 1))
        self.assertTrue(Vector2d.ZERO.equal(Vector2d.ONE, 1.1))

    def test_dot(self):
        v = Vector2d(1, 2)
        self.assertAlmostEqual(v.dot(Vector2d(3, 4)), 11.0)
        self.assertAlmostEqual(v.dot(Vector2d(0, 0)), 0.0)
        self.assertAlmostEqual(v.dot(Vector2d(1, 0)), 1.0)
        self.assertAlmostEqual(v.dot(Vector2d(0, 1)), 2.0)

    def test_correct(self):
        vec1 = Vector2d(0, float("nan"))
        vec2 = Vector2d(float("inf"), -1)
        vec3 = Vector2d(10, -2)

        vec1.correct()
        vec2.correct()
        vec3.correct()

        self.assertAlmostEqual(vec1, Vector2d(0, 0))
        self.assertAlmostEqual(vec2, Vector2d(0, -1))
        self.assertAlmostEqual(vec3, Vector2d(10, -2))

    def test_abs_dot(self):
        v = Vector2d(1, -2)

        self.assertAlmostEqual(v.abs_dot(Vector2d(3, 4)), 11.0)
        self.assertAlmostEqual(v.abs_dot(Vector2d(0, 0)), 0.0)
        self.assertAlmostEqual(v.abs_dot(Vector2d(1, 0)), 1.0)
        self.assertAlmostEqual(v.abs_dot(Vector2d(0, 1)), 2.0)

    def test_add(self):
        vec1 = Vector2d(0.1, 0.2)
        vec2 = Vector2d(1.1, 2.2)

        vec3 = copy.deepcopy(vec1)
        vec3 += vec2

        self.assertAlmostEqual(vec1 + vec2, Vector2d(1.2, 2.4))
        self.assertAlmostEqual(vec3, Vector2d(1.2, 2.4))

        # Add zero
        # Scalar right
        self.assertEqual(vec1 + 0, vec1)

        # Vector left and right
        self.assertAlmostEqual(Vector2d.ZERO + vec1, vec1)
        self.assertAlmostEqual(vec1 + Vector2d.ZERO, vec1)

        # Addition assigment
        vec4 = Vector2d(vec1)
        vec4 += 0
        self.assertEqual(vec4, vec1)
        vec4 += Vector2d.ZERO
        self.assertAlmostEqual(vec4, vec1)

        # Add non-trivial scalar values left and right
        self.assertEqual(vec1 + 2.5, Vector2d(2.6, 2.7))

        vec1 = vec4
        vec4 += 2.5
        self.assertEqual(vec4, Vector2d(2.6, 2.7))

    def test_sub(self):
        vec1 = Vector2d(0.1, 0.2)
        vec2 = Vector2d(1.1, 2.2)

        vec3 = copy.deepcopy(vec2)
        vec3 -= vec1

        self.assertAlmostEqual(vec2 - vec1, Vector2d(1.0, 2.0))
        self.assertAlmostEqual(vec3, Vector2d(1.0, 2.0))

        # Subtraction with zeros
        # Scalar right
        self.assertEqual(vec1 - 0, vec1)

        # Vector left and right
        self.assertAlmostEqual(Vector2d.ZERO - vec1, -vec1)
        self.assertAlmostEqual(vec1 - Vector2d.ZERO, vec1)

        # Subtraction assignment
        vec4 = Vector2d(vec1)
        vec4 -= 0
        self.assertEqual(vec4, vec1)
        vec4 -= Vector2d.ZERO
        self.assertAlmostEqual(vec4, vec1)

        # Subtract non-trivial scalar values left and right
        self.assertEqual(vec1 - 2.5, -Vector2d(2.4, 2.3))

        vec4 = vec1
        vec4 -= 2.5
        self.assertEqual(vec4, -Vector2d(2.4, 2.3))

    def test_multiply(self):
        v = Vector2d(0.1, -4.2)

        vec2 = v * 2.0
        self.assertEqual(vec2, Vector2d(0.2, -8.4))

        vec2 *= 4.0
        self.assertEqual(vec2, Vector2d(0.8, -33.6))

        # Multiply by zero
        # Scalar right
        self.assertEqual(v * 0, Vector2d.ZERO)

        # Element-wise vector multiplication
        self.assertEqual(v * Vector2d.ZERO, Vector2d.ZERO)

        # Multiply by one
        # Scalar right
        self.assertEqual(v * 1, v)

        # Element-wise vector multiplication
        self.assertEqual(v * Vector2d.ONE, v)

        # Multiply by non-trivial scalar value
        scalar = 2.5
        expect = Vector2d(0.25, -10.5)
        self.assertEqual(v * scalar, expect)

        # Multiply by itself element-wise
        v.set(0.1, 0.5)
        self.assertAlmostEqual(v * v, Vector2d(0.01, 0.25))

    def test_lenght(self):
        # Zero vector
        self.assertAlmostEqual(Vector2d.ZERO.length(), 0.0)
        self.assertAlmostEqual(Vector2d.ZERO.squared_length(), 0.0)

        # One vector
        self.assertAlmostEqual(Vector2d.ONE.length(),
                               math.sqrt(2), delta=1e-10)
        self.assertAlmostEqual(Vector2d.ONE.squared_length(), 2.0)

        # Arbitrary vector
        v = Vector2d(0.1, -4.2)
        self.assertAlmostEqual(v.length(), 4.20119030752, delta=1e-10)
        self.assertAlmostEqual(v.squared_length(), 17.65)

        # Integer vector
        v = Vector2d(3, 4)
        self.assertAlmostEqual(v.length(), 5)
        self.assertAlmostEqual(v.squared_length(), 25)

    def test_nan(self):
        nanVec = Vector2d.NAN
        self.assertFalse(nanVec.is_finite())
        self.assertTrue(math.isnan(nanVec.x()))
        self.assertTrue(math.isnan(nanVec.y()))

        nanVec.correct()
        self.assertEqual(Vector2d.ZERO, nanVec)
        self.assertTrue(nanVec.is_finite())

        nanVecF = Vector2f.NAN
        self.assertFalse(nanVecF.is_finite())
        self.assertTrue(math.isnan(nanVecF.x()))
        self.assertTrue(math.isnan(nanVecF.y()))

        nanVecF.correct()
        self.assertEqual(Vector2f.ZERO, nanVecF)
        self.assertTrue(nanVecF.is_finite())

if __name__ == '__main__':
    unittest.main()
