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

import math
import unittest
from ignition.math import Matrix3d
from ignition.math import Quaterniond
from ignition.math import Vector3d


class TestMatrix3(unittest.TestCase):

    def test_matrix3d(self):
        matrix = Matrix3d()
        self.assertAlmostEqual(matrix, Matrix3d(0, 0, 0, 0, 0, 0, 0, 0, 0))

        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)
        self.assertAlmostEqual(matrix, Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9))

        matrix1 = Matrix3d(matrix)
        self.assertAlmostEqual(matrix1, Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9))

        matrix = Matrix3d()
        matrix.axes(Vector3d(1, 1, 1), Vector3d(2, 2, 2),
                    Vector3d(3, 3, 3))
        self.assertAlmostEqual(matrix, Matrix3d(1, 2, 3, 1, 2, 3, 1, 2, 3))

        matrix.axis(Vector3d(1, 1, 1), math.pi)
        self.assertAlmostEqual(matrix, Matrix3d(1, 2, 2, 2, 1, 2, 2, 2, 1))

        matrix.col(0, Vector3d(3, 4, 5))
        self.assertAlmostEqual(matrix, Matrix3d(3, 2, 2, 4, 1, 2, 5, 2, 1))

        matrix.col(3, Vector3d(1, 1, 1))
        self.assertAlmostEqual(matrix, Matrix3d(3, 2, 1, 4, 1, 1, 5, 2, 1))

    def test_sub(self):
        matZero = Matrix3d.ZERO
        matIdent = Matrix3d.IDENTITY

        mat = matIdent - matZero
        self.assertAlmostEqual(mat, matIdent)

        matA = Matrix3d(1, 2, 3,
                        4, 5, 6,
                        7, 8, 9)
        matB = Matrix3d(10, 20, 30,
                        40, 50, 60,
                        70, 80, 90)

        mat = matB - matA
        self.assertAlmostEqual(mat, Matrix3d(9, 18, 27,
                                             36, 45, 54,
                                             63, 72, 81))

    def test_add(self):

        matZero = Matrix3d.ZERO
        matIdent = Matrix3d.IDENTITY

        mat = matIdent + matZero
        self.assertAlmostEqual(mat, matIdent)

        matA = Matrix3d(1, 2, 3,
                        4, 5, 6,
                        7, 8, 9)
        matB = Matrix3d(10, 20, 30,
                        40, 50, 60,
                        70, 80, 90)

        mat = matB + matA
        self.assertAlmostEqual(mat, Matrix3d(11, 22, 33,
                                             44, 55, 66,
                                             77, 88, 99))

    def test_mul(self):
        matZero = Matrix3d.ZERO
        matIdent = Matrix3d.IDENTITY

        mat = matIdent * matZero
        self.assertAlmostEqual(mat, matZero)

        matA = Matrix3d(1, 2, 3,
                        4, 5, 6,
                        7, 8, 9)
        matB = Matrix3d(11, 21, 31,
                        41, 51, 61,
                        71, 81, 91)

        mat = matA * matB
        self.assertAlmostEqual(mat, Matrix3d(306, 366, 426,
                                             675, 825, 975,
                                             1044, 1284, 1524))

        mat = matB * matA
        self.assertAlmostEqual(mat, Matrix3d(312, 375, 438,
                                             672, 825, 978,
                                             1032, 1275, 1518))

        mat = mat * 2.0
        self.assertAlmostEqual(mat, Matrix3d(624, 750, 876,
                                             1344, 1650, 1956,
                                             2064, 2550, 3036))

    def test_stream_out(self):
        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)

        self.assertEqual(str(matrix), "1 2 3 4 5 6 7 8 9")

    def test_vector3_mul(self):
        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)

        # Scalar
        self.assertAlmostEqual(Matrix3d.ZERO, matrix * 0)

        # Vector3.ZERO
        self.assertAlmostEqual(Vector3d.ZERO, matrix * Vector3d.ZERO)

        # Matrix3.ZERO
        self.assertAlmostEqual(Matrix3d.ZERO, matrix * Matrix3d.ZERO)
        self.assertAlmostEqual(Matrix3d.ZERO, Matrix3d.ZERO * matrix)

        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)

        # scalar 1.0
        self.assertAlmostEqual(matrix, matrix * 1.0)

        # Vector3.Unit
        # right multiply
        self.assertAlmostEqual(Vector3d(matrix(0, 0), matrix(1, 0),
                                        matrix(2, 0)),
                               matrix * Vector3d.UNIT_X)
        self.assertAlmostEqual(Vector3d(matrix(0, 1), matrix(1, 1),
                                        matrix(2, 1)),
                               matrix * Vector3d.UNIT_Y)
        self.assertAlmostEqual(Vector3d(matrix(0, 2), matrix(1, 2),
                                        matrix(2, 2)),
                               matrix * Vector3d.UNIT_Z)

        # Matrix3.IDENTITY
        self.assertAlmostEqual(matrix, matrix * Matrix3d.IDENTITY)
        self.assertAlmostEqual(matrix, Matrix3d.IDENTITY * matrix)

        # Multiply arbitrary matrix by itself
        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)
        matrix2 = Matrix3d(30,  36,  42,
                           66,  81,  96,
                           102, 126, 150)

        self.assertAlmostEqual(matrix * matrix, matrix2)

    def test_not_equal(self):
        matrix1 = Matrix3d()
        matrix2 = Matrix3d()

        self.assertTrue(matrix1 == matrix2)
        self.assertFalse(matrix1 != matrix2)

        matrix1 = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)
        matrix2 = Matrix3d(matrix1)

        self.assertFalse(matrix1 != matrix1)

        matrix2 = Matrix3d(1.00001, 2, 3, 4, 5, 6, 7, 8, 9)
        self.assertTrue(matrix1 != matrix2)

        matrix2 = Matrix3d(1.000001, 2, 3, 4, 5, 6, 7, 8, 9)
        self.assertFalse(matrix1 != matrix2)

    def test_equal_tolerance(self):
        self.assertFalse(Matrix3d.ZERO.equal(Matrix3d.IDENTITY, 1e-6))
        self.assertFalse(Matrix3d.ZERO.equal(Matrix3d.IDENTITY, 1e-3))
        self.assertFalse(Matrix3d.ZERO.equal(Matrix3d.IDENTITY, 1e-1))
        self.assertTrue(Matrix3d.ZERO.equal(Matrix3d.IDENTITY, 1))
        self.assertTrue(Matrix3d.ZERO.equal(Matrix3d.IDENTITY, 1.1))

    def test_inverse(self):
        self.assertAlmostEqual(Matrix3d.IDENTITY, Matrix3d.IDENTITY.inverse())

        # Matrix multiplied by its inverse results in the identity matrix
        matrix1 = Matrix3d(-2, 4, 0, 0.1, 9, 55, -7, 1, 26)
        matrix2 = matrix1.inverse()
        self.assertAlmostEqual(matrix1 * matrix2, Matrix3d.IDENTITY)
        self.assertAlmostEqual(matrix2 * matrix1, Matrix3d.IDENTITY)

        # Inverse of inverse results in the same matrix
        self.assertAlmostEqual((matrix1.inverse()).inverse(), matrix1)

        # Invert multiplication by scalar
        scalar = 2.5
        self.assertAlmostEqual((matrix1 * scalar).inverse(),
                               matrix1.inverse() * (1.0/scalar))

    def test_determinant(self):
        # |ZERO matrix| = 0.0
        self.assertAlmostEqual(0.0, Matrix3d.ZERO.determinant())

        # |IDENTITY matrix| = 1.0
        self.assertAlmostEqual(1.0, Matrix3d.IDENTITY.determinant())

        # Determinant of arbitrary matrix
        m = Matrix3d(-2, 4, 0, 0.1, 9, 55, -7, 1, 26)
        self.assertAlmostEqual(-1908.4, m.determinant())

    def test_transpose(self):
        # transpose of zero matrix is itself
        self.assertAlmostEqual(Matrix3d.ZERO, Matrix3d.ZERO.transposed())

        # transpose of identity matrix is itself
        self.assertAlmostEqual(Matrix3d.IDENTITY,
                               Matrix3d.IDENTITY.transposed())

        # Matrix and expected transpose
        m = Matrix3d(-2, 4, 0,
                     0.1, 9, 55,
                     -7, 1, 26)
        mT = Matrix3d(-2, 0.1, -7,
                      4,   9, 1,
                      0,  55, 26)
        self.assertNotEqual(m, mT)
        self.assertAlmostEqual(m.transposed(), mT)
        self.assertAlmostEqual(m.determinant(), m.transposed().determinant())

        mT.transpose()
        self.assertAlmostEqual(m, mT)

    def test_from_2axes(self):
        v1 = Vector3d(1.0, 0.0, 0.0)
        v2 = Vector3d(0.0, 1.0, 0.0)

        m1 = Matrix3d()
        m1.from_2_axes(v1, v2)

        m2 = Matrix3d()
        m2.from_2_axes(v2, v1)

        m1Correct = Matrix3d(0, -1, 0,
                             1, 0, 0,
                             0, 0, 1)
        m2Correct = Matrix3d(m1Correct)
        m2Correct.transpose()

        self.assertNotEqual(m1, m2)
        self.assertAlmostEqual(m1Correct, m1)
        self.assertAlmostEqual(m2Correct, m2)
        self.assertAlmostEqual(Matrix3d.IDENTITY, m1 * m2)
        self.assertAlmostEqual(v2, m1 * v1)
        self.assertAlmostEqual(v1, m2 * v2)

        # rotation about 45 degrees
        v1.set(1.0, 0.0, 0.0)
        v2.set(1.0, 1.0, 0.0)
        m2.from_2_axes(v1, v2)
        # m1 is 90 degrees rotation
        self.assertAlmostEqual(m1, m2*m2)

        # with non-unit vectors
        v1.set(0.5, 0.5, 0)
        v2.set(-0.5, 0.5, 0)

        m1.from_2_axes(v1, v2)
        m2.from_2_axes(v2, v1)

        self.assertNotEqual(m1, m2)
        self.assertAlmostEqual(m1Correct, m1)
        self.assertAlmostEqual(m2Correct, m2)
        self.assertAlmostEqual(Matrix3d.IDENTITY, m1 * m2)
        self.assertAlmostEqual(v2, m1 * v1)
        self.assertAlmostEqual(v1, m2 * v2)

        # For zero-length vectors, a unit matrix is returned
        v1.set(0, 0, 0)
        v2.set(-0.5, 0.5, 0)
        m1.from_2_axes(v1, v2)
        self.assertAlmostEqual(Matrix3d.IDENTITY, m1)

        # For zero-length vectors, a unit matrix is returned
        v1.set(-0.5, 0.5, 0)
        v2.set(0, 0, 0)
        m1.from_2_axes(v1, v2)
        self.assertAlmostEqual(Matrix3d.IDENTITY, m1)

        # Parallel vectors
        v1.set(1, 0, 0)
        v2.set(2, 0, 0)
        m1.from_2_axes(v1, v2)
        self.assertAlmostEqual(Matrix3d.IDENTITY, m1)

        # Opposite vectors
        v1.set(1, 0, 0)
        v2.set(-2, 0, 0)
        m1.from_2_axes(v1, v2)
        self.assertAlmostEqual(Matrix3d.ZERO - Matrix3d.IDENTITY, m1)

    def test_to_quaternion(self):
        q = Quaterniond(math.pi/2.0, math.pi/2.0, 0)
        matFromQuat = Matrix3d(q)
        quatFromMat = Quaterniond(matFromQuat)
        self.assertTrue(q == quatFromMat)

        # test the cases where matrix trace is negative
        # (requires special handling)
        q = Quaterniond(0, 0, 0, 1)
        mat = Matrix3d(-1,  0, 0,
                       0, -1, 0,
                       0,  0, 1)
        q2 = Quaterniond(mat)
        self.assertTrue(q == q2)

        q = Quaterniond(0, 0, 1, 0)
        mat = Matrix3d(-1,  0,  0,
                       0,  1,  0,
                       0,  0, -1)
        q2 = Quaterniond(mat)
        self.assertTrue(q == q2)

        q = Quaterniond(0, 1, 0, 0)
        mat = Matrix3d(1,  0,  0,
                       0, -1,  0,
                       0,  0, -1)
        q2 = Quaterniond(mat)
        self.assertTrue(q == q2)


if __name__ == '__main__':
    unittest.main()
