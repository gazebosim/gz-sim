# Copyright (C) 2021 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http:#www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import unittest
import math

from ignition.math import sort3
from ignition.math import MassMatrix3d
from ignition.math import Material
from ignition.math import Vector3d
from ignition.math import Matrix3d
from ignition.math import Quaterniond

IGN_PI = 3.14159265358979323846
IGN_PI_2 = 1.57079632679489661923
IGN_PI_4 = 0.78539816339744830962
IGN_SQRT2 = 1.41421356237309504880


def equal(a, b, error):
    return abs(a - b) <= error


class TestMassMatrix(unittest.TestCase):

    def test_constructors(self):
        m = MassMatrix3d()

        self.assertEqual(0.0, m.mass())
        self.assertEqual(0.0, m.ixx())
        self.assertEqual(0.0, m.iyy())
        self.assertEqual(0.0, m.izz())
        self.assertEqual(0.0, m.ixy())
        self.assertEqual(0.0, m.ixz())
        self.assertEqual(0.0, m.iyz())
        self.assertEqual(Vector3d.ZERO, m.diagonal_moments())
        self.assertEqual(Vector3d.ZERO, m.off_diagonal_moments())
        self.assertEqual(Matrix3d.ZERO, m.moi())
        self.assertFalse(m.is_positive())
        self.assertTrue(m.is_near_positive())
        self.assertTrue(m.is_valid())

        m2 = MassMatrix3d(0.0, Vector3d.ZERO, Vector3d.ZERO)
        self.assertEqual(m2, MassMatrix3d())
        self.assertEqual(m2, MassMatrix3d(m2))
        self.assertFalse(m2.is_positive())
        self.assertTrue(m2.is_near_positive())
        self.assertTrue(m2.is_valid())

        mass = 5.0
        Ixxyyzz = Vector3d(2.0, 3.0, 4.0)
        Ixyxzyz = Vector3d(0.2, 0.3, 0.4)
        moi = Matrix3d(2.0, 0.2, 0.3,
                       0.2, 3.0, 0.4,
                       0.3, 0.4, 4.0)
        m3 = MassMatrix3d(mass, Ixxyyzz, Ixyxzyz)

        self.assertNotEqual(m3, MassMatrix3d())
        self.assertEqual(m3, MassMatrix3d(m3))

        # Test accessors
        self.assertEqual(mass, m3.mass())
        self.assertEqual(Ixxyyzz.x(), m3.ixx())
        self.assertEqual(Ixxyyzz.y(), m3.iyy())
        self.assertEqual(Ixxyyzz.z(), m3.izz())
        self.assertEqual(Ixyxzyz.x(), m3.ixy())
        self.assertEqual(Ixyxzyz.y(), m3.ixz())
        self.assertEqual(Ixyxzyz.z(), m3.iyz())
        self.assertEqual(Ixxyyzz, m3.diagonal_moments())
        self.assertEqual(Ixyxzyz, m3.off_diagonal_moments())
        self.assertEqual(moi, m3.moi())
        self.assertTrue(m3.is_positive())
        self.assertTrue(m3.is_near_positive())
        self.assertTrue(m3.is_valid())

        # Test assignment operator
        m4 = MassMatrix3d()
        self.assertNotEqual(m3, m4)
        m4 = m3
        self.assertEqual(m3, m4)

    def test_setters(self):
        mass = 5.0
        Ixxyyzz = Vector3d(2.0, 3.0, 4.0)
        Ixyxzyz = Vector3d(0.2, 0.3, 0.4)
        moi = Matrix3d(2.0, 0.2, 0.3,
                       0.2, 3.0, 0.4,
                       0.3, 0.4, 4.0)

        m = MassMatrix3d()
        self.assertFalse(m.is_positive())
        self.assertTrue(m.is_near_positive())
        self.assertTrue(m.is_valid())

        # Valid when mass is set
        self.assertTrue(m.set_mass(mass))
        self.assertFalse(m.set_ixx(Ixxyyzz.x()))
        self.assertFalse(m.set_iyy(Ixxyyzz.y()))

        # Valid once enough properties are set
        self.assertTrue(m.set_izz(Ixxyyzz.z()))
        self.assertTrue(m.set_ixy(Ixyxzyz.x()))
        self.assertTrue(m.set_ixz(Ixyxzyz.y()))
        self.assertTrue(m.set_iyz(Ixyxzyz.z()))

        # Verify values
        self.assertEqual(m.mass(), mass)
        self.assertEqual(m.ixx(), Ixxyyzz.x())
        self.assertEqual(m.iyy(), Ixxyyzz.y())
        self.assertEqual(m.izz(), Ixxyyzz.z())
        self.assertEqual(m.ixy(), Ixyxzyz.x())
        self.assertEqual(m.ixz(), Ixyxzyz.y())
        self.assertEqual(m.iyz(), Ixyxzyz.z())
        self.assertEqual(m.diagonal_moments(), Ixxyyzz)
        self.assertEqual(m.off_diagonal_moments(), Ixyxzyz)
        self.assertEqual(m.moi(), moi)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        # Invalid again if an invalid inertia is set
        self.assertFalse(m.set_mass(-1))

        m = MassMatrix3d()
        self.assertFalse(m.is_positive())
        self.assertTrue(m.is_near_positive())
        self.assertTrue(m.is_valid())

        # Valid when mass is set
        self.assertTrue(m.set_mass(mass))

        # Valid once enough properties are set
        self.assertTrue(m.set_diagonal_moments(Ixxyyzz))
        self.assertTrue(m.set_off_diagonal_moments(Ixyxzyz))

        # Verify values
        self.assertEqual(m.mass(), mass)
        self.assertEqual(m.ixx(), Ixxyyzz.x())
        self.assertEqual(m.iyy(), Ixxyyzz.y())
        self.assertEqual(m.izz(), Ixxyyzz.z())
        self.assertEqual(m.ixy(), Ixyxzyz.x())
        self.assertEqual(m.ixz(), Ixyxzyz.y())
        self.assertEqual(m.iyz(), Ixyxzyz.z())
        self.assertEqual(m.diagonal_moments(), Ixxyyzz)
        self.assertEqual(m.off_diagonal_moments(), Ixyxzyz)
        self.assertEqual(m.moi(), moi)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        # Invalid if an invalid inertia is set
        self.assertFalse(m.set_ixx(-1))

        # Test Matrix3 setter for moment of inertia
        m = MassMatrix3d()
        self.assertFalse(m.is_positive())
        self.assertTrue(m.is_near_positive())
        self.assertTrue(m.is_valid())

        # Valid when mass is set
        self.assertTrue(m.set_mass(mass))

        # Valid once enough properties are set
        self.assertTrue(m.set_moi(moi))

        # Verify values
        self.assertEqual(m.mass(), mass)
        self.assertEqual(m.ixx(), Ixxyyzz.x())
        self.assertEqual(m.iyy(), Ixxyyzz.y())
        self.assertEqual(m.izz(), Ixxyyzz.z())
        self.assertEqual(m.ixy(), Ixyxzyz.x())
        self.assertEqual(m.ixz(), Ixyxzyz.y())
        self.assertEqual(m.iyz(), Ixyxzyz.z())
        self.assertEqual(m.diagonal_moments(), Ixxyyzz)
        self.assertEqual(m.off_diagonal_moments(), Ixyxzyz)
        self.assertEqual(m.moi(), moi)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        # Invalid if an excessive off-diagonal inertia is set
        self.assertFalse(m.set_ixy(1e3))

        # Test atomic InertiaMatrix setter
        m = MassMatrix3d()
        self.assertFalse(m.is_positive())
        self.assertTrue(m.is_near_positive())
        self.assertTrue(m.is_valid())

        # Initially invalid
        self.assertTrue(m.set_mass(mass))

        # Valid once enough properties are set
        self.assertTrue(m.set_inertia_matrix(2, 3, 4, 0.2, 0.3, 0.4))

        # Verify values
        self.assertEqual(m.mass(), mass)
        self.assertEqual(m.ixx(), Ixxyyzz.x())
        self.assertEqual(m.iyy(), Ixxyyzz.y())
        self.assertEqual(m.izz(), Ixxyyzz.z())
        self.assertEqual(m.ixy(), Ixyxzyz.x())
        self.assertEqual(m.ixz(), Ixyxzyz.y())
        self.assertEqual(m.iyz(), Ixyxzyz.z())
        self.assertEqual(m.diagonal_moments(), Ixxyyzz)
        self.assertEqual(m.off_diagonal_moments(), Ixyxzyz)
        self.assertEqual(m.moi(), moi)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

    def test_principal_moments(self):
        m = MassMatrix3d(1.0, Vector3d.ONE, Vector3d.ZERO)
        self.assertEqual(m.principal_moments(), Vector3d.ONE)

        # Minor perturbations of product moments
        # shouldn't affect principal_moments, given the tolerance
        # of the Vector3 equality operator
        self.assertTrue(m.set_ixy(1e-10))
        self.assertTrue(m.set_ixz(2e-10))
        self.assertTrue(m.set_iyz(3e-10))
        self.assertEqual(m.principal_moments(), Vector3d.ONE)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        # Non-equal eigen-moments
        Ixxyyzz = Vector3d(2.0, 3.0, 4.0)
        m = MassMatrix3d(1.0, Ixxyyzz, Vector3d.ZERO)
        self.assertTrue(m.set_diagonal_moments(Ixxyyzz))
        self.assertEqual(m.principal_moments(), Ixxyyzz)

        # Minor perturbation of product moments
        self.assertTrue(m.set_ixy(1e-10))
        self.assertTrue(m.set_ixz(2e-10))
        self.assertTrue(m.set_iyz(3e-10))
        self.assertEqual(m.principal_moments(), Ixxyyzz)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        # Non-trivial off-diagonal product moments
        # Symmetric positive definite matrix from
        # Strang's Intro to Linear Algebra textbook
        # This isn't actually a valid inertia matrix though,
        # since it doesn't satisfy the triangle inequality
        # 2-math.sqrt(2) + 2 ~= 2.59
        # 2+math.sqrt(2) ~= 3.41
        Ixxyyzz = Vector3d(2.0, 2.0, 2.0)
        Ixyxzyz = Vector3d(-1.0, 0, -1.0)
        m = MassMatrix3d(1.0, Ixxyyzz, Ixyxzyz)
        Ieigen = Vector3d(2-IGN_SQRT2, 2, 2+IGN_SQRT2)
        self.assertEqual(m.principal_moments(), Ieigen)
        self.assertTrue(m.is_positive())
        self.assertFalse(m.is_valid())

        # Non-trivial off-diagonal product moments
        # variant of previous example that is valid inertia matrix
        Ixxyyzz = Vector3d(4.0, 4.0, 4.0)
        Ixyxzyz = Vector3d(-1.0, 0, -1.0)
        m = MassMatrix3d(1.0, Ixxyyzz, Ixyxzyz)
        Ieigen = Vector3d(4-IGN_SQRT2, 4, 4+IGN_SQRT2)
        self.assertEqual(m.principal_moments(), Ieigen)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        # Degenerate matrix with eigenvalue of 0
        # not positive definite
        Ixxyyzz = Vector3d(1.0, 1.0, 1.0)
        Ixyxzyz = Vector3d(1.0, 0, 0)
        m = MassMatrix3d(1.0, Ixxyyzz, Ixyxzyz)
        Ieigen = Vector3d(0, 1, 2)
        self.assertEqual(m.principal_moments(), Ieigen)
        self.assertTrue(m.is_positive())
        self.assertFalse(m.is_valid())

        # Matrix with large condition number
        # barely positive definite
        # invalid inertia matrix since it doesn't satisfy triangle inequality
        # 5e-6 + 1.0 < 2+5e-6
        Ixxyyzz = Vector3d(1.0, 1.00001, 1.0)
        Ixyxzyz = Vector3d(1.0, 0, 0)
        m = MassMatrix3d(1.0, Ixxyyzz, Ixyxzyz)
        Ieigen = Vector3d(5e-6, 1.0, 2 + 5e-6)
        self.assertEqual(m.principal_moments(), Ieigen)
        self.assertTrue(m.is_positive())
        self.assertFalse(m.is_valid())

        # Another matrix with large condition number
        # invalid inertia matrix since it doesn't satisfy triangle inequality
        # 0.98 + 1e8-1e3 < 1e8+1e3
        # 0.98 < 2e3
        Ixxyyzz = Vector3d(1e8, 1e8, 1)
        Ixyxzyz = Vector3d(1e3, 1e3, 1e3)
        m = MassMatrix3d(1.0, Ixxyyzz, Ixyxzyz)
        Ieigen = Vector3d(0.98, 1e8-1e3, 1e8+1e3)
        # the accuracy is approximately 2e-2
        self.assertTrue(m.principal_moments().equal(Ieigen, 2.5e-2))
        self.assertFalse(m.principal_moments().equal(Ieigen, 1.5e-2))
        # the default tolerance for == is 1e-6
        # so this should resolve as not equal
        self.assertNotEqual(m.principal_moments(), Ieigen)
        self.assertTrue(m.is_positive())
        self.assertFalse(m.is_valid())

    def test_principal_axes_offset_identity(self):
        # Identity inertia matrix, expect unit quaternion
        m = MassMatrix3d(1.0, Vector3d.ONE, Vector3d.ZERO)
        self.assertEqual(m.principal_axes_offset(), Quaterniond())

        # Scale the diagonal terms
        self.assertTrue(m.set_diagonal_moments(Vector3d.ONE * 3.5))
        self.assertTrue(m.set_off_diagonal_moments(Vector3d.ZERO))
        self.assertTrue(m.is_valid())
        self.assertEqual(m.principal_axes_offset(), Quaterniond.IDENTITY)

    # \brief Helper function for verifying principal moments
    # and axes offset by reconstructing the moment of inertia matrix
    # from the eigenvectors and diagonalized matrix.
    # \param[in] _m mass matrix to verify
    # \param[in] _tolerance relative tolerance to use
    def verify_principal_moments_and_axes(self, _m, _tolerance=1e-6):
        q = _m.principal_axes_offset(_tolerance)
        R = Matrix3d(q)
        self.assertFalse(equal(q.w(), 0.0, 1e-6) and equal(q.x(), 0.0, 1e-6) and
                         equal(q.y(), 0.0, 1e-6) and equal(q.z(), 0.0, 1e-6))
        moments = _m.principal_moments(_tolerance)
        L = Matrix3d(moments.x(), 0, 0,
                     0, moments.y(), 0,
                     0, 0, moments.z())
        self.assertEqual(_m.moi(), R * L * R.transposed())

    # \brief Helper function for testing diagonal inertia matrices.
    # Expect the following:
    # * that principal moments match the diagonal values,
    # * that mass matrix is valid,
    # * that principal axes have no offset (identity quaternion)
    # * that reconstructed moment of inertia matrix matches the original
    # \param[in] _moments Diagonal/principal moments of inertia.
    def verify_diagonal_moments_and_axes(self, _moments):
        m = MassMatrix3d(1.0, Vector3d.ZERO, Vector3d.ZERO)
        self.assertTrue(m.set_diagonal_moments(_moments))
        self.assertEqual(m.principal_moments(), m.diagonal_moments())
        self.assertTrue(m.is_valid())
        # Expect unit quaternion
        self.assertEqual(m.principal_axes_offset(), Quaterniond.IDENTITY)
        self.verify_principal_moments_and_axes(m)

        # Try with negative tolerance, expect sorted principal moments
        sortedMoments = Vector3d()
        m0 = _moments.x()
        m1 = _moments.y()
        m2 = _moments.z()
        m0, m1, m2 = sort3(m0, m1, m2)
        sortedMoments.set(m0, m1, m2)
        tolerance = -1e-6
        self.assertEqual(m.principal_moments(tolerance), sortedMoments)
        self.verify_principal_moments_and_axes(m, tolerance)

    def test_principal_axes_offset_diagonal(self):
        # all repeated moments [3, 3, 3]
        self.verify_diagonal_moments_and_axes(Vector3d(3.0, 3.0, 3.0))
        # repeated moments [2, 3, 3]
        self.verify_diagonal_moments_and_axes(Vector3d(2.0, 3.0, 3.0))
        self.verify_diagonal_moments_and_axes(Vector3d(3.0, 2.0, 3.0))
        self.verify_diagonal_moments_and_axes(Vector3d(3.0, 3.0, 2.0))
        # repeated moments [2, 2, 3]
        self.verify_diagonal_moments_and_axes(Vector3d(3.0, 2.0, 2.0))
        self.verify_diagonal_moments_and_axes(Vector3d(2.0, 3.0, 2.0))
        self.verify_diagonal_moments_and_axes(Vector3d(2.0, 2.0, 3.0))
        # non-repeated moments
        self.verify_diagonal_moments_and_axes(Vector3d(2.0, 3.0, 4.0))
        self.verify_diagonal_moments_and_axes(Vector3d(4.0, 2.0, 3.0))
        self.verify_diagonal_moments_and_axes(Vector3d(3.0, 4.0, 2.0))
        self.verify_diagonal_moments_and_axes(Vector3d(2.0, 4.0, 3.0))
        self.verify_diagonal_moments_and_axes(Vector3d(3.0, 2.0, 4.0))
        self.verify_diagonal_moments_and_axes(Vector3d(4.0, 3.0, 2.0))

    # Helper function for testing non-diagonal inertia matrices.
    # Expect the following:
    # * that principal moments match the supplied values,
    # * that mass matrix is valid,
    # * that principal axes have an offset (non-identity quaternion)
    # * that reconstructed moment of inertia matrix matches the original
    # \param[in] _principalMoments Expected principal moments of inertia
    # \param[in] _ixxyyzz Diagonal moments of inertia.
    # \param[in] _ixyxzyz Off-diagonal moments of inertia.
    # \param[in] _tolerance Absolute tolerance for eigenvalue expectation.
    def verify_non_diagonal_moments_and_axes(self,
                                             _principalMoments,
                                             _ixxyyzz,
                                             _ixyxzyz,
                                             _tolerance=1e-6):
        m = MassMatrix3d(1.0, _ixxyyzz, _ixyxzyz)
        # self.assertEqual with default tolerance of 1e-6
        # this outputs more useful error messages
        self.assertEqual(m.principal_moments(_tolerance), _principalMoments)
        # also check equality with custom tolerance for small moments
        self.assertTrue(
            m.principal_moments(_tolerance).equal(_principalMoments, _tolerance))
        self.assertTrue(m.is_valid())
        # Expect non-unit quaternion
        self.assertNotEqual(m.principal_axes_offset(_tolerance), Quaterniond())
        self.verify_principal_moments_and_axes(m, _tolerance)

        # Try also with negated tolerance
        self.assertTrue(
            m.principal_moments(-_tolerance).equal(_principalMoments, _tolerance))
        self.verify_principal_moments_and_axes(m, -_tolerance)

    def test_principal_axes_offset_repeat(self):
        # Principal moments: [3, 3, 5]
        # Non-zero ixy
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 3, 5),
            Vector3d(4, 4, 3),
            Vector3d(-1, 0, 0))
        # Non-zero ixz
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 3, 5),
            Vector3d(4, 3, 4),
            Vector3d(0, -1, 0))
        # Non-zero iyz
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 3, 5),
            Vector3d(3, 4, 4),
            Vector3d(0, 0, -1))

        # Principal moments: [3, 5, 5]
        # Non-zero ixy
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 5, 5),
            Vector3d(4, 4, 5),
            Vector3d(-1, 0, 0))
        # Non-zero ixz
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 5, 5),
            Vector3d(4, 5, 4),
            Vector3d(0, -1, 0))
        # Non-zero iyz
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 5, 5),
            Vector3d(5, 4, 4),
            Vector3d(0, 0, -1))

        # Principal moments: [4, 5, 5]
        # Rotated by [45, 45, 0] degrees
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 5, 5),
            Vector3d(4.5, 4.75, 4.75),
            Vector3d(-IGN_SQRT2, IGN_SQRT2, 1) * 0.25)
        # Rotated by [-45, 45, 0] degrees
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 5, 5),
            Vector3d(4.5, 4.75, 4.75),
            Vector3d(IGN_SQRT2, IGN_SQRT2, -1) * 0.25)
        # Rotated by [45, -45, 0] degrees
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 5, 5),
            Vector3d(4.5, 4.75, 4.75),
            Vector3d(IGN_SQRT2, -IGN_SQRT2, 1) * 0.25)
        # Rotated by [-45, -45, 0] degrees
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 5, 5),
            Vector3d(4.5, 4.75, 4.75),
            Vector3d(-IGN_SQRT2, -IGN_SQRT2, -1) * 0.25)

        # Principal moments: [4, 4, 5]
        # Rotated by [45, 45, 45] degrees
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 5),
            Vector3d(4.5, 4.25, 4.25),
            Vector3d(-IGN_SQRT2, IGN_SQRT2, -1) * 0.25)
        # different rotation
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 5),
            Vector3d(4.5, 4.25, 4.25),
            Vector3d(IGN_SQRT2, IGN_SQRT2, 1) * 0.25)
        # different rotation
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 5),
            Vector3d(4.5, 4.25, 4.25),
            Vector3d(-IGN_SQRT2, -IGN_SQRT2, 1) * 0.25)
        # different rotation
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 5),
            Vector3d(4.5, 4.25, 4.25),
            Vector3d(IGN_SQRT2, -IGN_SQRT2, -1) * 0.25)

        # Principal moments [4e-9, 4e-9, 5e-9]
        # Rotated by [45, 45, 45] degrees
        # use tolerance of 1e-15
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 5) * 1e-9,
            Vector3d(4.5, 4.25, 4.25) * 1e-9,
            Vector3d(-IGN_SQRT2, IGN_SQRT2, -1) * 0.25e-9, 1e-15)
        # different rotation
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 5) * 1e-9,
            Vector3d(4.5, 4.25, 4.25) * 1e-9,
            Vector3d(IGN_SQRT2, IGN_SQRT2, 1) * 0.25e-9)
        # different rotation
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 5) * 1e-9,
            Vector3d(4.5, 4.25, 4.25) * 1e-9,
            Vector3d(-IGN_SQRT2, -IGN_SQRT2, 1) * 0.25e-9)
        # different rotation
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 5) * 1e-9,
            Vector3d(4.5, 4.25, 4.25) * 1e-9,
            Vector3d(IGN_SQRT2, -IGN_SQRT2, -1) * 0.25e-9, 1e-15)

        # Principal moments [4, 4, 6]
        # rotate by 30, 60, 0 degrees
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 6),
            Vector3d(5.5, 4.125, 4.375),
            Vector3d(-math.sqrt(3), 3.0, -math.sqrt(3)/2) * 0.25)

        # different rotation
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4, 4, 6),
            Vector3d(4.125, 5.5, 4.375),
            Vector3d(-math.sqrt(3), -math.sqrt(3)/2, 3.0) * 0.25)

    def test_principal_axes_offset_no_repeat(self):
        # Non-diagonal inertia matrix with f1 = 0
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(3.0, 5.0, 5.0),
            Vector3d(0, 0, 1))
        # Non-diagonal inertia matrix with f1 = 0
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(3.0, 5.0, 5.0),
            Vector3d(0, 0, -1))

        # Non-diagonal inertia matrix with f2 = 0
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(5.0, 4.0, 4.0),
            Vector3d(-1, 1, 0))
        # Non-diagonal inertia matrix with f2 = 0
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(5.0, 4.0, 4.0),
            Vector3d(1, -1, 0))
        # Non-diagonal inertia matrix with f2 = 0
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(5.0, 4.0, 4.0),
            Vector3d(-1, -1, 0))
        # Non-diagonal inertia matrix with f2 = 0
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(5.0, 4.0, 4.0),
            Vector3d(1, 1, 0))

        # Similar non-diagonal inertia matrix with f2 != 0
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(4.0, 4.0, 5.0),
            Vector3d(0, 1, 1))
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(4.0, 4.0, 5.0),
            Vector3d(0, -1, 1))
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(4.0, 4.0, 5.0),
            Vector3d(0, 1, -1))
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(3, 4, 6),
            Vector3d(4.0, 4.0, 5.0),
            Vector3d(0, -1, -1))

        # Test case for v = 0
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(2.5, 3.5, 4.0),
            Vector3d(4.0, 3.0, 3.0),
            Vector3d(0.0, 0, -0.5))

        # Tri-diagonal matrix with identical diagonal terms
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4-IGN_SQRT2, 4, 4+IGN_SQRT2),
            Vector3d(4.0, 4.0, 4.0),
            Vector3d(-1.0, 0, -1.0))
        # small magnitude, use tolerance of 1e-15
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(4-IGN_SQRT2, 4, 4+IGN_SQRT2) * 1e-9,
            Vector3d(4.0, 4.0, 4.0) * 1e-9,
            Vector3d(-1.0, 0, -1.0) * 1e-9, 1e-15)

        # Tri-diagonal matrix with unique diagonal terms
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(5-math.sqrt(3), 5, 5+math.sqrt(3)),
            Vector3d(4.0, 5.0, 6.0),
            Vector3d(-1.0, 0, -1.0))
        # small magnitude, use tolerance of 1e-15
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(5-math.sqrt(3), 5, 5+math.sqrt(3)) * 1e-9,
            Vector3d(4.0, 5.0, 6.0) * 1e-9,
            Vector3d(-1.0, 0, -1.0) * 1e-9, 1e-15)

        # Nonzero values for all off-axis terms
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(10, 12, 14),
            Vector3d(13, 11.75, 11.25),
            Vector3d(-0.5*math.sqrt(3), 1.5, 0.25*math.sqrt(3)))

        # Nonzero values for all off-axis terms
        self.verify_non_diagonal_moments_and_axes(
            Vector3d(6.6116, 8.2393186767, 13.983881323),
            Vector3d(11.6116, 8.6116, 8.6116),
            Vector3d(2, 2, 2))

    def test_equivalent_box(self):
        # Default mass matrix with non-negative inertia
        m = MassMatrix3d()
        size = Vector3d()
        rot = Quaterniond()

        # size is all zeros, so set_from_box should fail
        self.assertFalse(m.set_from_box(0.0, size, rot))
        self.assertFalse(m.set_from_box(size, rot))

        # even if mass is valid, it should not be set if size is invalid
        self.assertFalse(m.set_from_box(1.0, size, rot))
        self.assertEqual(m.mass(), 0.0)

        # equivalent box should not be findable
        self.assertFalse(m.equivalent_box(size, rot))

        # Moment of inertia matrix that doesn't satisfy triangle inequality
        ixxyyzz = Vector3d(2.0, 2.0, 2.0)
        ixyxzyz = Vector3d(-1.0, 0, -1.0)
        m = MassMatrix3d(1.0, ixxyyzz, ixyxzyz)
        size = Vector3d()
        rot = Quaterniond()
        self.assertFalse(m.equivalent_box(size, rot))

        # Identity inertia matrix
        # expect cube with side length math.sqrt(6)
        mass = 1.0
        m = MassMatrix3d(mass, Vector3d.ONE, Vector3d.ZERO)
        size = Vector3d()
        sizeTrue = Vector3d(Vector3d.ONE * math.sqrt(6))
        rot = Quaterniond()
        rotTrue = Quaterniond(Quaterniond.IDENTITY)
        self.assertTrue(m.equivalent_box(size, rot))
        self.assertEqual(size, sizeTrue)
        self.assertEqual(rot, rotTrue)

        # create new MassMatrix3d
        # it initially has zero mass, so set_from_box(size, rot) will fail
        m2 = MassMatrix3d()
        self.assertFalse(m2.set_from_box(sizeTrue, rotTrue))
        self.assertTrue(m2.set_from_box(mass, sizeTrue, rotTrue))
        self.assertEqual(m, m2)

        density = mass / (sizeTrue.x() * sizeTrue.y() * sizeTrue.z())
        mat = Material(density)
        self.assertEqual(density, mat.density())
        m3 = MassMatrix3d()
        self.assertTrue(m3.set_from_box(mat, sizeTrue, rotTrue))
        self.assertEqual(m2, m3)

        # unit box with mass 1.0
        mass = 1.0
        size = Vector3d(1, 1, 1)
        ixx = mass/12 * (math.pow(size.y(), 2) + math.pow(size.z(), 2))
        iyy = mass/12 * (math.pow(size.z(), 2) + math.pow(size.x(), 2))
        izz = mass/12 * (math.pow(size.x(), 2) + math.pow(size.y(), 2))
        ixxyyzz = Vector3d(ixx, iyy, izz)
        m = MassMatrix3d(mass, ixxyyzz, Vector3d.ZERO)
        size2 = Vector3d()
        rot = Quaterniond()
        self.assertTrue(m.equivalent_box(size2, rot))
        self.assertEqual(size, size2)
        self.assertEqual(rot, Quaterniond.IDENTITY)

        m2 = MassMatrix3d()
        self.assertTrue(m2.set_from_box(mass, size2, rot))
        self.assertEqual(m, m2)

        # box 1x4x9
        mass = 12.0
        ixxyyzz = Vector3d(97, 82, 17)
        m = MassMatrix3d(mass, ixxyyzz, Vector3d.ZERO)
        size = Vector3d()
        rot = Quaterniond()
        self.assertTrue(m.equivalent_box(size, rot))
        self.assertEqual(size, Vector3d(1, 4, 9))
        self.assertEqual(rot, Quaterniond.IDENTITY)

        m2 = MassMatrix3d()
        self.assertTrue(m2.set_from_box(mass, size, rot))
        self.assertEqual(m, m2)

        # box 1x4x9 rotated by 90 degrees around Z
        mass = 12.0
        ixxyyzz = Vector3d(82, 17, 97)
        m = MassMatrix3d(mass, ixxyyzz, Vector3d.ZERO)
        size = Vector3d()
        rot = Quaterniond()
        self.assertTrue(m.equivalent_box(size, rot, -1e-6))
        self.assertEqual(size, Vector3d(9, 4, 1))
        self.assertEqual(rot, Quaterniond(0, 0, IGN_PI/2))

        m2 = MassMatrix3d()
        self.assertTrue(m2.set_from_box(mass, size, rot))
        self.assertEqual(m, m2)

        # box 1x4x9 rotated by 45 degrees around Z
        mass = 12.0
        ixxyyzz = Vector3d(49.5, 49.5, 97)
        ixyxzyz = Vector3d(-32.5, 0.0, 0.0)
        m = MassMatrix3d(mass, ixxyyzz, ixyxzyz)
        size = Vector3d()
        rot = Quaterniond()
        self.assertTrue(m.equivalent_box(size, rot))
        self.assertEqual(size, Vector3d(9, 4, 1))
        # There are multiple correct rotations due to box symmetry
        self.assertTrue(rot == Quaterniond(0, 0, IGN_PI/4) or
                        rot == Quaterniond(IGN_PI, 0, IGN_PI/4))

        m2 = MassMatrix3d()
        self.assertTrue(m2.set_from_box(mass, size, rot))
        self.assertEqual(m, m2)

        # long slender box
        mass = 12.0
        ixxyyzz = Vector3d(1, 1, 2e-6)
        m = MassMatrix3d(mass, ixxyyzz, Vector3d.ZERO)
        size = Vector3d()
        rot = Quaterniond()
        self.assertTrue(m.equivalent_box(size, rot))
        self.assertEqual(size, Vector3d(1e-3, 1e-3, 1))
        self.assertEqual(rot, Quaterniond.IDENTITY)

        m2 = MassMatrix3d()
        self.assertTrue(m2.set_from_box(mass, size, rot))
        self.assertEqual(m, m2)

    def test_set_from_cylinderZ(self):
        q0 = Quaterniond.IDENTITY

        # Default mass matrix with non-positive inertia
        m = MassMatrix3d()

        # input is all zeros, so set_from_cylinder_z should fail
        self.assertFalse(m.set_from_cylinder_z(0, 0, 0, q0))
        self.assertFalse(m.set_from_cylinder_z(0, 0, q0))

        # even if some parameters are valid, none should be set if they
        # are not all valid
        self.assertFalse(m.set_from_cylinder_z(1, 0, 0, q0))
        self.assertFalse(m.set_from_cylinder_z(1, 1, 0, q0))
        self.assertFalse(m.set_from_cylinder_z(1, 0, 1, q0))
        self.assertEqual(m.mass(), 0.0)

        # unit cylinder with mass 1.0
        mass = 1.0
        length = 1.0
        radius = 0.5
        m = MassMatrix3d()
        self.assertTrue(m.set_from_cylinder_z(mass, length, radius, q0))

        ixx = mass / 12.0 * (3*math.pow(radius, 2) + math.pow(length, 2))
        iyy = ixx
        izz = mass / 2.0 * math.pow(radius, 2)
        ixxyyzz = Vector3d(ixx, iyy, izz)
        self.assertEqual(m.diagonal_moments(), ixxyyzz)
        self.assertEqual(m.off_diagonal_moments(), Vector3d.ZERO)

        density = mass / (IGN_PI * radius * radius * length)
        mat = Material(density)
        self.assertEqual(density, mat.density())
        m1 = MassMatrix3d()
        # self.assertFalse(m1.set_from_cylinder_z(Material(0), length, radius))
        self.assertTrue(m1.set_from_cylinder_z(mat, length, radius))
        self.assertEqual(m, m1)

        # double the length and radius
        self.assertTrue(m.set_from_cylinder_z(mass, 2*length, 2*radius, q0))
        self.assertEqual(m.diagonal_moments(), ixxyyzz * 4)

    def test_set_from_sphere(self):
        m = MassMatrix3d()

        # input is all zeros, so set_from_sphere should fail
        self.assertFalse(m.set_from_sphere(0.0, 0.0))
        self.assertFalse(m.set_from_sphere(0.0))

        # even if mass is valid, it should not be set if radius is invalid
        self.assertFalse(m.set_from_sphere(1.0, 0.0))
        self.assertEqual(m.mass(), 0.0)

        mass = 1.0
        radius = 0.5
        m = MassMatrix3d()
        self.assertTrue(m.set_from_sphere(mass, radius))

        ixx = 0.4 * mass * math.pow(radius, 2)
        iyy = ixx
        izz = ixx
        ixxyyzz = Vector3d(ixx, iyy, izz)
        self.assertEqual(m.diagonal_moments(), ixxyyzz)
        self.assertEqual(m.off_diagonal_moments(), Vector3d.ZERO)

        density = mass / ((4.0/3.0) * IGN_PI * math.pow(radius, 3))
        mat = Material(density)
        self.assertEqual(density, mat.density())
        m1 = MassMatrix3d()
        self.assertFalse(m1.set_from_sphere(mat, 0))
        self.assertFalse(m1.set_from_sphere(Material(0), 0))
        self.assertTrue(m1.set_from_sphere(mat, radius))
        self.assertEqual(m, m1)

        # double the radius
        self.assertTrue(m.set_from_sphere(mass, 2*radius))
        self.assertEqual(m.diagonal_moments(), ixxyyzz * 4)

    def test_valid_moments_tolerance(self):
        moments = Vector3d()
        self.assertTrue(MassMatrix3d.valid_moments(moments, 0))
        self.assertTrue(MassMatrix3d.valid_moments(moments))

        massMatrix = MassMatrix3d()
        # Default inertia matrix is all zeros.
        # Since the tolerance is relative to the max inertia,
        # it has no effect when max inertia is zero.
        # is_valid and is_near_positive will be true
        self.assertTrue(massMatrix.is_valid())
        self.assertTrue(massMatrix.is_valid(10))
        self.assertTrue(massMatrix.is_valid(0))
        self.assertTrue(massMatrix.is_valid(-1))
        self.assertTrue(massMatrix.is_near_positive())
        self.assertTrue(massMatrix.is_near_positive(10))
        self.assertTrue(massMatrix.is_near_positive(0))
        self.assertTrue(massMatrix.is_near_positive(-1))
        # and is_positive will be false
        self.assertFalse(massMatrix.is_positive())
        self.assertFalse(massMatrix.is_positive(10))
        self.assertFalse(massMatrix.is_positive(0))
        self.assertFalse(massMatrix.is_positive(-1))

        # setting ixx = iyy > 0 with izz = 0
        # satisfies the triangle inequality
        massMatrix.set_ixx(0.1)
        massMatrix.set_iyy(0.1)
        # is_valid, is_near_positive, and is_positive will have same
        # behavior if tolerance >= 0
        self.assertTrue(massMatrix.is_valid())
        self.assertTrue(massMatrix.is_valid(10))
        self.assertTrue(massMatrix.is_valid(0))
        self.assertTrue(massMatrix.is_near_positive())
        self.assertTrue(massMatrix.is_near_positive(10))
        self.assertTrue(massMatrix.is_near_positive(0))
        self.assertFalse(massMatrix.is_positive())
        self.assertFalse(massMatrix.is_positive(10))
        self.assertFalse(massMatrix.is_positive(0))
        # but they are all false if the tolerance is negative
        self.assertFalse(massMatrix.is_valid(-1))
        self.assertFalse(massMatrix.is_near_positive(-1))
        self.assertFalse(massMatrix.is_positive(-1))


if __name__ == '__main__':
    unittest.main()
