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

import copy
import math
import unittest

from ignition.math import Inertiald, Quaterniond, Pose3d, Matrix3d, MassMatrix3d, Vector3d


class TestInertial(unittest.TestCase):

    def CompareModuloPi(self, _q1, _q2, _tol=1e-6):
        rotErrorEuler = (_q1.inverse() * _q2).euler()
        self.assertAlmostEqual(math.sin(rotErrorEuler.x()), 0.0, delta=_tol)
        self.assertAlmostEqual(math.sin(rotErrorEuler.y()), 0.0, delta=_tol)
        self.assertAlmostEqual(math.sin(rotErrorEuler.z()), 0.0, delta=_tol)

    def test_constructor(self):
        inertial = Inertiald()
        self.assertEqual(inertial.pose(), Pose3d.ZERO)
        self.assertEqual(inertial.mass_matrix(), MassMatrix3d())
        self.assertEqual(inertial.moi(), Matrix3d.ZERO)

    def test_constructor_default_values(self):
        inertial = Inertiald(MassMatrix3d(), Pose3d.ZERO)
        self.assertEqual(inertial, Inertiald())
        self.assertEqual(inertial, Inertiald(inertial))

    def test_constructor_non_default_values(self):
        mass = 5.0
        Ixxyyzz = Vector3d(2.0, 3.0, 4.0)
        Ixyxzyz = Vector3d(0.2, 0.3, 0.4)
        m = MassMatrix3d(mass, Ixxyyzz, Ixyxzyz)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())
        pose = Pose3d(1, 2, 3, math.pi/6, 0, 0)
        inertial = Inertiald(m, pose)

        # Should not match simple constructor
        self.assertNotEqual(inertial, Inertiald())

        # Should match with copy constructor
        self.assertEqual(inertial, Inertiald(inertial))

        # Test accessors
        self.assertEqual(inertial.mass_matrix(), m)
        self.assertEqual(inertial.pose(), pose)
        self.assertTrue(inertial.mass_matrix().is_positive())
        self.assertTrue(inertial.mass_matrix().is_valid())

        # Test assignment operator
        inertial2 = Inertiald()
        self.assertNotEqual(inertial, inertial2)
        inertial2 = inertial
        self.assertEqual(inertial, inertial2)

    def test_set_mass_matrix(self):
        inertial = Inertiald()
        m = MassMatrix3d()

        # This will be true because the default mass of zero is considered valid
        self.assertTrue(inertial.set_mass_matrix(m, 0))
        # Set the mass to a negative value, and set_mass_matrix should complain.
        m.set_mass(-1)
        self.assertFalse(inertial.set_mass_matrix(m, 0))

    def test_setters(self):
        mass = 5.0
        Ixxyyzz = Vector3d(2.0, 3.0, 4.0)
        Ixyxzyz = Vector3d(0.2, 0.3, 0.4)
        m = MassMatrix3d(mass, Ixxyyzz, Ixyxzyz)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())
        pose = Pose3d(1, 2, 3, math.pi/6, 0, 0)
        inertial = Inertiald()

        # Initially valid
        self.assertTrue(inertial.set_pose(pose))

        # Valid once valid mass matrix is set
        self.assertTrue(inertial.set_mass_matrix(m))

        # Verify values
        self.assertEqual(inertial.mass_matrix(), m)
        self.assertEqual(inertial.pose(), pose)

        # Invalid again if an invalid inertia is set
        mInvalid = MassMatrix3d(-1, Ixxyyzz, Ixyxzyz)
        self.assertFalse(inertial.set_mass_matrix(mInvalid))

    def test_moi_diagonal(self):
        mass = 12.0
        Ixxyyzz = Vector3d(2.0, 3.0, 4.0)
        Ixyxzyz = Vector3d(0, 0, 0)
        m = MassMatrix3d(mass, Ixxyyzz, Ixyxzyz)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        # no rotation, expect MOI's to match
        pose = Pose3d(0, 0, 0, 0, 0, 0)
        inertial = Inertiald(m, pose)
        self.assertEqual(inertial.moi(), m.moi())

        # 90 deg rotation about X axis, expect different MOI
        pose = Pose3d(0, 0, 0, math.pi/2, 0, 0)
        expectedMOI = Matrix3d(2, 0, 0, 0, 4, 0, 0, 0, 3)
        inertial = Inertiald(m, pose)
        self.assertNotEqual(inertial.moi(), m.moi())
        self.assertEqual(inertial.moi(), expectedMOI)

        # 90 deg rotation about Y axis, expect different MOI
        pose = Pose3d(0, 0, 0, 0, math.pi/2, 0)
        expectedMOI = Matrix3d(4, 0, 0, 0, 3, 0, 0, 0, 2)
        inertial = Inertiald(m, pose)
        self.assertNotEqual(inertial.moi(), m.moi())
        self.assertEqual(inertial.moi(), expectedMOI)

        # 90 deg rotation about Z axis, expect different MOI
        pose = Pose3d(0, 0, 0, 0, 0, math.pi/2)
        expectedMOI = Matrix3d(3, 0, 0, 0, 2, 0, 0, 0, 4)
        inertial = Inertiald(m, pose)
        self.assertNotEqual(inertial.moi(), m.moi())
        self.assertEqual(inertial.moi(), expectedMOI)

        # 45 deg rotation about Z axis, expect different MOI
        pose = Pose3d(0, 0, 0, 0, 0, math.pi/4)
        expectedMOI = Matrix3d(2.5, -0.5, 0, -0.5, 2.5, 0, 0, 0, 4)
        inertial = Inertiald(m, pose)
        self.assertNotEqual(inertial.moi(), m.moi())
        self.assertEqual(inertial.moi(), expectedMOI)

        # check with a second MassMatrix3 instance
        # that has the same base frame MOI but no pose rotation
        m2 = MassMatrix3d()
        self.assertTrue(m2.set_mass(mass))
        self.assertTrue(m2.set_moi(expectedMOI))
        self.assertEqual(inertial.moi(), m2.moi())
        # There are multiple correct rotations due to symmetry
        self.CompareModuloPi(m2.principal_axes_offset(), pose.rot())

    def SetRotation(self, _mass, _ixxyyzz, _ixyxzyz, _unique=True):
        m = MassMatrix3d(_mass, _ixxyyzz, _ixyxzyz)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        pose = Pose3d(Vector3d.ZERO, Quaterniond.IDENTITY)
        inertialRef = Inertiald(m, pose)
        moi = inertialRef.moi()

        rotations = [
            Quaterniond.IDENTITY,
            Quaterniond(math.pi, 0, 0),
            Quaterniond(0, math.pi, 0),
            Quaterniond(0, 0, math.pi),
            Quaterniond(math.pi/2, 0, 0),
            Quaterniond(0, math.pi/2, 0),
            Quaterniond(0, 0, math.pi/2),
            Quaterniond(math.pi/4, 0, 0),
            Quaterniond(0, math.pi/4, 0),
            Quaterniond(0, 0, math.pi/4),
            Quaterniond(math.pi/6, 0, 0),
            Quaterniond(0, math.pi/6, 0),
            Quaterniond(0, 0, math.pi/6),
            Quaterniond(0.1, 0.2, 0.3),
            Quaterniond(-0.1, 0.2, -0.3),
            Quaterniond(0.4, 0.2, 0.5),
            Quaterniond(-0.1, 0.7, -0.7)]
        for rot in rotations:
            inertial = Inertiald(m, pose)
            tol = -1e-6
            self.assertTrue(inertial.set_mass_matrix_rotation(rot, tol))
            self.assertEqual(moi, inertial.moi())
            if (_unique):
                self.CompareModuloPi(rot, inertial.mass_matrix().principal_axes_offset(tol))

            self.assertTrue(inertial.set_inertial_rotation(rot))
            self.assertEqual(rot, inertial.pose().rot())
            self.assertEqual(moi, inertial.moi())

            inertial = Inertiald(m, pose)

            self.assertTrue(inertial.set_inertial_rotation(rot))
            self.assertEqual(rot, inertial.pose().rot())
            self.assertEqual(moi, inertial.moi())

            tol = -1e-6
            self.assertTrue(inertial.set_mass_matrix_rotation(rot, tol))
            self.assertEqual(moi, inertial.moi())
            if (_unique):
                self.CompareModuloPi(rot, inertial.mass_matrix().principal_axes_offset(tol))

    def test_set_rotation_unique_diagonal(self):
        self.SetRotation(12, Vector3d(2, 3, 4), Vector3d.ZERO)
        self.SetRotation(12, Vector3d(3, 2, 4), Vector3d.ZERO)
        self.SetRotation(12, Vector3d(2, 4, 3), Vector3d.ZERO)
        self.SetRotation(12, Vector3d(3, 4, 2), Vector3d.ZERO)
        self.SetRotation(12, Vector3d(4, 2, 3), Vector3d.ZERO)
        self.SetRotation(12, Vector3d(4, 3, 2), Vector3d.ZERO)

    def test_set_rotation_unique_non_diagonal(self):
        self.SetRotation(12, Vector3d(2, 3, 4), Vector3d(0.3, 0.2, 0.1))

    def test_set_rotation_non_unique_diagonal(self):
        self.SetRotation(12, Vector3d(2, 2, 2), Vector3d.ZERO, False)
        self.SetRotation(12, Vector3d(2, 2, 3), Vector3d.ZERO, False)
        self.SetRotation(12, Vector3d(2, 3, 2), Vector3d.ZERO, False)
        self.SetRotation(12, Vector3d(3, 2, 2), Vector3d.ZERO, False)
        self.SetRotation(12, Vector3d(2, 3, 3), Vector3d.ZERO, False)
        self.SetRotation(12, Vector3d(3, 2, 3), Vector3d.ZERO, False)
        self.SetRotation(12, Vector3d(3, 3, 2), Vector3d.ZERO, False)

    def test_set_rotation_non_unique_non_diagonal(self):
        self.SetRotation(12, Vector3d(4, 4, 3), Vector3d(-1, 0, 0), False)
        self.SetRotation(12, Vector3d(4, 3, 4), Vector3d(0, -1, 0), False)
        self.SetRotation(12, Vector3d(3, 4, 4), Vector3d(0, 0, -1), False)
        self.SetRotation(12, Vector3d(4, 4, 5), Vector3d(-1, 0, 0), False)
        self.SetRotation(12, Vector3d(5, 4, 4), Vector3d(0, 0, -1), False)
        self.SetRotation(12, Vector3d(5.5, 4.125, 4.375),
                         Vector3d(-math.sqrt(3), 3.0, -math.sqrt(3)/2)*0.25, False)
        self.SetRotation(12, Vector3d(4.125, 5.5, 4.375),
                         Vector3d(-math.sqrt(3), -math.sqrt(3)/2, 3.0)*0.25, False)

    # test for diagonalizing MassMatrix
    # verify MOI is conserved
    # and that off-diagonal terms are zero
    def Diagonalize(self, _mass, _ixxyyzz, _ixyxzyz):
        m = MassMatrix3d(_mass, _ixxyyzz, _ixyxzyz)
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        pose = Pose3d(Vector3d.ZERO, Quaterniond.IDENTITY)
        inertial = Inertiald(m, pose)
        moi = inertial.moi()

        self.assertTrue(inertial.set_mass_matrix_rotation(Quaterniond.IDENTITY))
        self.assertEqual(moi, inertial.moi())
        self.assertEqual(inertial.mass_matrix().off_diagonal_moments(), Vector3d.ZERO)

        # try again with negative tolerance
        self.assertTrue(inertial.set_mass_matrix_rotation(Quaterniond.IDENTITY, -1e-6))
        self.assertEqual(moi, inertial.moi())
        self.assertEqual(inertial.mass_matrix().off_diagonal_moments(), Vector3d.ZERO)

    def test_diagonalize(self):
        self.Diagonalize(12, Vector3d(2, 3, 4), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(3, 2, 4), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(2, 4, 3), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(3, 4, 2), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(4, 2, 3), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(4, 3, 2), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(2, 3, 4), Vector3d(0.3, 0.2, 0.1))
        self.Diagonalize(12, Vector3d(2, 2, 2), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(2, 2, 3), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(2, 3, 2), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(3, 2, 2), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(2, 3, 3), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(3, 2, 3), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(3, 3, 2), Vector3d.ZERO)
        self.Diagonalize(12, Vector3d(4, 4, 3), Vector3d(-1, 0, 0))
        self.Diagonalize(12, Vector3d(4, 3, 4), Vector3d(0, -1, 0))
        self.Diagonalize(12, Vector3d(3, 4, 4), Vector3d(0, 0, -1))
        self.Diagonalize(12, Vector3d(4, 4, 5), Vector3d(-1, 0, 0))
        self.Diagonalize(12, Vector3d(5, 4, 4), Vector3d(0, 0, -1))
        self.Diagonalize(12, Vector3d(5.5, 4.125, 4.375),
                         Vector3d(-math.sqrt(3), 3.0, -math.sqrt(3)/2)*0.25)
        self.Diagonalize(12, Vector3d(4.125, 5.5, 4.375),
                         Vector3d(-math.sqrt(3), -math.sqrt(3)/2, 3.0)*0.25)

    def test_addition(self):
        # Add two half-cubes together
        mass = 12.0
        size = Vector3d(1, 1, 1)
        cubeMM3 = MassMatrix3d()
        self.assertTrue(cubeMM3.set_from_box(mass, size))
        cube = Inertiald(cubeMM3, Pose3d.ZERO)
        half = MassMatrix3d()
        self.assertTrue(half.set_from_box(0.5*mass, Vector3d(0.5, 1, 1)))
        left = Inertiald(half, Pose3d(-0.25, 0, 0, 0, 0, 0))
        right = Inertiald(half, Pose3d(0.25, 0, 0, 0, 0, 0))
        self.assertEqual(cube, left + right)
        self.assertEqual(cube, right + left)

        # test += operator
        tmp = left
        tmp += right
        self.assertTrue(cube == tmp)

        left = Inertiald(half, Pose3d(-0.25, 0, 0, 0, 0, 0))
        right = Inertiald(half, Pose3d(0.25, 0, 0, 0, 0, 0))
        tmp = copy.copy(right)
        tmp += left
        self.assertTrue(cube == tmp)

        # Test equivalent_box
        left = Inertiald(half, Pose3d(-0.25, 0, 0, 0, 0, 0))
        right = Inertiald(half, Pose3d(0.25, 0, 0, 0, 0, 0))
        size2 = Vector3d()
        rot2 = Quaterniond()
        self.assertTrue((left + right).mass_matrix().equivalent_box(size2, rot2))
        self.assertEqual(size, size2)
        self.assertEqual(rot2, Quaterniond.IDENTITY)

        size2 = Vector3d()
        rot2 = Quaterniond()
        self.assertTrue((right + left).mass_matrix().equivalent_box(size2, rot2))
        self.assertEqual(size, size2)
        self.assertEqual(rot2, Quaterniond.IDENTITY)

        # Add two rotated half-cubes together
        mass = 12.0
        size = Vector3d(1, 1, 1)
        cubeMM3 = MassMatrix3d()
        self.assertTrue(cubeMM3.set_from_box(mass, size))
        cube = Inertiald(cubeMM3, Pose3d(0, 0, 0, math.pi/4, 0, 0))

        half = MassMatrix3d()
        self.assertTrue(half.set_from_box(0.5*mass, Vector3d(0.5, 1, 1)))
        left = Inertiald(half, Pose3d(-0.25, 0, 0, math.pi/4, 0, 0))
        right = Inertiald(half, Pose3d(0.25, 0, 0, math.pi/4, 0, 0))

        # objects won't match exactly
        # since inertia matrices will all be in base frame
        # but mass, center of mass, and base-frame MOI should match
        self.assertNotEqual(cube, left + right)
        self.assertNotEqual(cube, right + left)
        self.assertEqual(cubeMM3.mass(), (left + right).mass_matrix().mass())
        self.assertEqual(cubeMM3.mass(), (right + left).mass_matrix().mass())
        self.assertEqual(cube.pose().pos(), (left + right).pose().pos())
        self.assertEqual(cube.pose().pos(), (right + left).pose().pos())
        self.assertEqual(cube.moi(), (left + right).moi())
        self.assertEqual(cube.moi(), (right + left).moi())

        mass = 12.0
        size = Vector3d(1, 1, 1)
        cubeMM3 = MassMatrix3d()
        self.assertTrue(cubeMM3.set_from_box(mass, size))
        addedCube = Inertiald(
          Inertiald(cubeMM3, Pose3d(-0.5, -0.5, -0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(-0.5,  0.5, -0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(0.5,  -0.5, -0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(0.5,   0.5, -0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(-0.5, -0.5, 0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(-0.5,  0.5, 0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(0.5,  -0.5, 0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(0.5,   0.5, 0.5, 0, 0, 0)))

        trueCubeMM3 = MassMatrix3d()
        self.assertTrue(trueCubeMM3.set_from_box(mass * 8, size * 2))
        self.assertEqual(addedCube, Inertiald(trueCubeMM3, Pose3d.ZERO))

        # Add eight rotated cubes together into larger cube
        mass = 12.0
        size = Vector3d(1, 1, 1)
        cubeMM3 = MassMatrix3d()
        self.assertTrue(cubeMM3.set_from_box(mass, size))
        addedCube = Inertiald(
          Inertiald(cubeMM3, Pose3d(-0.5, -0.5, -0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(-0.5,  0.5, -0.5, math.pi/2, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(0.5,  -0.5, -0.5, 0, math.pi/2, 0)) +
          Inertiald(cubeMM3, Pose3d(0.5,   0.5, -0.5, 0, 0, math.pi/2)) +
          Inertiald(cubeMM3, Pose3d(-0.5, -0.5, 0.5, math.pi, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(-0.5,  0.5, 0.5, 0, math.pi, 0)) +
          Inertiald(cubeMM3, Pose3d(0.5,  -0.5, 0.5, 0, 0, math.pi)) +
          Inertiald(cubeMM3, Pose3d(0.5,   0.5, 0.5, 0, 0, 0)))

        trueCubeMM3 = MassMatrix3d()
        self.assertTrue(trueCubeMM3.set_from_box(mass * 8, size * 2))
        self.assertEqual(addedCube, Inertiald(trueCubeMM3, Pose3d.ZERO))

        # Add two cubes with diagonal corners touching at one point
        #           ┌---------┐
        #           |         |
        #           |         |
        #           |         |
        #           |         |
        # ┌---------+---------┘
        # |         |
        # |         |
        # |         |
        # |         |
        # └---------┘

        # properties of each cube to be added
        # side length: 1
        # mass: 6
        # diagonal moment of inertia values: 1
        # off-diagonal moment of inertia values: 0
        mass = 6.0
        size = Vector3d(1, 1, 1)
        cubeMM3 = MassMatrix3d()
        self.assertTrue(cubeMM3.set_from_box(mass, size))
        self.assertEqual(
            Vector3d.ONE,
            cubeMM3.diagonal_moments())
        self.assertEqual(
            Vector3d.ZERO,
            cubeMM3.off_diagonal_moments())

        diagonalCubes = Inertiald(
          Inertiald(cubeMM3, Pose3d(-0.5, -0.5, -0.5, 0, 0, 0)) +
          Inertiald(cubeMM3, Pose3d(0.5,  0.5, 0.5, 0, 0, 0)))

        # lumped mass = 6 + 6 = 12
        # lumped center of mass at (0, 0, 0)
        # lumped Moment of inertia:
        #   for each cube
        #   [ 1  0  0 ]       [ 0.5^2 + 0.5^2  -0.5*0.5            -0.5*0.5 ]
        #   [ 0  1  0 ] + 6 * [ -0.5*0.5       0.5^2 + 0.5^2       -0.5*0.5 ]
        #   [ 0  0  1 ]       [ -0.5*0.5       -0.5*0.5       0.5^2 + 0.5^2 ]
        #
        #   [ 1  0  0 ]       [  0.5   -0.25  -0.25 ]
        #   [ 0  1  0 ] + 6 * [ -0.25   0.5   -0.25 ]
        #   [ 0  0  1 ]       [ -0.25  -0.25   0.5  ]
        #
        #   [ 1  0  0 ]   [  3.0  -1.5  -1.5 ]
        #   [ 0  1  0 ] + [ -1.5   3.0  -1.5 ]
        #   [ 0  0  1 ]   [ -1.5  -1.5   3.0 ]
        #
        #   [  4.0  -1.5  -1.5 ]
        #   [ -1.5   4.0  -1.5 ]
        #   [ -1.5  -1.5   4.0 ]
        #
        # then it to account for both cubes
        self.assertEqual(Pose3d.ZERO, diagonalCubes.pose())
        self.assertEqual(mass * 2.0, diagonalCubes.mass_matrix().mass())
        self.assertEqual(
            Vector3d(8, 8, 8),
            diagonalCubes.mass_matrix().diagonal_moments())
        self.assertEqual(
            Vector3d(-3, -3, -3),
            diagonalCubes.mass_matrix().off_diagonal_moments())

    def test_addition_invalid(self):
        # inertias all zero
        m0 = MassMatrix3d(0.0, Vector3d.ZERO, Vector3d.ZERO)
        self.assertFalse(m0.is_positive())
        self.assertTrue(m0.is_near_positive())
        self.assertTrue(m0.is_valid())

        # both inertials with zero mass
        left = Inertiald(m0, Pose3d(-1, 0, 0, 0, 0, 0))
        right = Inertiald(m0, Pose3d(1, 0, 0, 0, 0, 0))

        # expect sum to equal left argument
        self.assertEqual(left, left + right)
        self.assertEqual(right, right + left)

        tmp = left
        tmp += right
        self.assertEqual(tmp, left)

        tmp = right
        tmp += left
        self.assertEqual(tmp, right)

        # one inertial with zero inertias should not affect the sum
        m = MassMatrix3d(12.0, Vector3d(2, 3, 4), Vector3d(0.1, 0.2, 0.3))
        self.assertTrue(m.is_positive())
        self.assertTrue(m.is_valid())

        i = Inertiald(m, Pose3d(-1, 0, 0, 0, 0, 0))
        i0 = Inertiald(m0, Pose3d(1, 0, 0, 0, 0, 0))

        # expect i0 to not affect the sum
        self.assertEqual(i, i + i0)
        self.assertEqual(i, i0 + i)

        tmp = i
        tmp += i0
        self.assertEqual(tmp, i)

        tmp = i0
        tmp += i
        self.assertEqual(tmp, i)

        self.assertTrue((i + i0).mass_matrix().is_positive())
        self.assertTrue((i0 + i).mass_matrix().is_positive())
        self.assertTrue((i + i0).mass_matrix().is_valid())
        self.assertTrue((i0 + i).mass_matrix().is_valid())


if __name__ == '__main__':
    unittest.main()
