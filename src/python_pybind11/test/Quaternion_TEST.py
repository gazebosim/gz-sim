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
from ignition.math import Matrix3d
from ignition.math import Matrix4d
from ignition.math import Quaterniond
from ignition.math import Quaternionf
from ignition.math import Quaternioni
from ignition.math import Vector3d


class TestQuaternion(unittest.TestCase):
    def test_construction(self):
        q = Quaterniond(0, 0, 0, 1)

        q2 = Quaterniond(q)
        self.assertEqual(q2, q)

        q3 = q
        self.assertEqual(q3, q)

        q4 = Quaterniond(q)
        self.assertEqual(q4, q2)
        q = q4
        self.assertEqual(q, q2)

        q5 = q2
        self.assertEqual(q5, q3)
        q2 = q5
        self.assertEqual(q2, q3)

        q6 = Quaterniond()
        self.assertNotEqual(q6, q3)

    def test_unit(self):
        q = Quaterniond()
        self.assertAlmostEqual(q.w(), 1.0)
        self.assertAlmostEqual(q.x(), 0.0)
        self.assertAlmostEqual(q.y(), 0.0)
        self.assertAlmostEqual(q.z(), 0.0)

    def test_construct_values(self):
        q = Quaterniond(1.0, 2.0, 3.0, 4.0)
        self.assertAlmostEqual(q.w(), 1.0)
        self.assertAlmostEqual(q.x(), 2.0)
        self.assertAlmostEqual(q.y(), 3.0)
        self.assertAlmostEqual(q.z(), 4.0)

    def test_construct_zero(self):
        q = Quaterniond(0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(q.w(), 0.0)
        self.assertAlmostEqual(q.x(), 0.0)
        self.assertAlmostEqual(q.y(), 0.0)
        self.assertAlmostEqual(q.z(), 0.0)

        qI = q.inverse()
        self.assertAlmostEqual(qI.w(), 1.0)
        self.assertAlmostEqual(qI.x(), 0.0)
        self.assertAlmostEqual(qI.y(), 0.0)
        self.assertAlmostEqual(qI.z(), 0.0)

    def test_construct_euler(self):
        q = Quaterniond(0, 1, 2)
        self.assertAlmostEqual(q, Quaterniond(Vector3d(0, 1, 2)))

    def test_construct_axis_angle(self):
        q1 = Quaterniond(Vector3d(0, 0, 1), math.pi)

        self.assertAlmostEqual(q1.x(), 0.0)
        self.assertAlmostEqual(q1.y(), 0.0)
        self.assertAlmostEqual(q1.z(), 1.0)
        self.assertAlmostEqual(q1.w(), 0.0)

        q = Quaterniond(q1)
        self.assertTrue(q == q1)

    def test_equal(self):
        # double
        q = Quaterniond(1, 2, 3, 4)
        q2 = Quaterniond(1.01, 2.015, 3.002, 4.007)
        self.assertTrue(q.equal(q2, 0.02))
        self.assertFalse(q.equal(q2, 0.01))

        # floats
        q3 = Quaternionf(1, 2, 3, 4)
        q4 = Quaternionf(1.05, 2.1, 3.03, 4.04)
        self.assertTrue(q3.equal(q4, 0.2))
        self.assertFalse(q3.equal(q4, 0.04))

        # ints
        q5 = Quaternioni(3, 5, -1, 9)
        q6 = Quaternioni(3, 6, 1, 12)
        self.assertTrue(q5.equal(q6, 3))
        self.assertFalse(q5.equal(q6, 2))

    def test_identity(self):
        q = Quaterniond.IDENTITY
        self.assertAlmostEqual(q.w(), 1.0)
        self.assertAlmostEqual(q.x(), 0.0)
        self.assertAlmostEqual(q.y(), 0.0)
        self.assertAlmostEqual(q.z(), 0.0)

    def test_mathlog(self):
        q = Quaterniond(math.pi*0.1, math.pi*0.5, math.pi)
        self.assertAlmostEqual(q.log(),
                               Quaterniond(0, -1.02593, 0.162491, 1.02593))

        q1 = Quaterniond(q)
        q1.w(2.0)
        self.assertAlmostEqual(q1.log(),
                               Quaterniond(0, -0.698401, 0.110616, 0.698401))

    def test_math_exp(self):
        q = Quaterniond(math.pi*0.1, math.pi*0.5, math.pi)
        self.assertAlmostEqual(q.exp(), Quaterniond(0.545456, -0.588972,
                                                    0.093284, 0.588972))

        q1 = Quaterniond(q)
        q1.x(0.000000001)
        q1.y(0.0)
        q1.z(0.0)
        q1.w(0.0)
        self.assertAlmostEqual(q1.exp(), Quaterniond(1, 0, 0, 0))

    def test_math_invert(self):
        q = Quaterniond(math.pi*0.1, math.pi*0.5, math.pi)

        q.invert()
        self.assertAlmostEqual(q, Quaterniond(0.110616, 0.698401,
                                              -0.110616, -0.698401))

    def test_math_axis(self):
        q = Quaterniond(math.pi*0.1, math.pi*0.5, math.pi)

        q.axis(0, 1, 0, math.pi)
        self.assertAlmostEqual(q, Quaterniond(6.12303e-17, 0, 1, 0))

        q.axis(Vector3d(1, 0, 0), math.pi)
        self.assertAlmostEqual(q, Quaterniond(0, 1, 0, 0))

    def test_math_set(self):
        q = Quaterniond(math.pi*0.1, math.pi*0.5, math.pi)

        q.set(1, 2, 3, 4)
        self.assertAlmostEqual(q.w(), 1.0)
        self.assertAlmostEqual(q.x(), 2.0)
        self.assertAlmostEqual(q.y(), 3.0)
        self.assertAlmostEqual(q.z(), 4.0)

    def test_math_normalized(self):
        q = Quaterniond(1, 2, 3, 4)

        q2 = q.normalized()
        self.assertAlmostEqual(q2, Quaterniond(0.182574, 0.365148,
                                               0.547723, 0.730297))

    def test_normalize(self):
        q = Quaterniond(1, 2, 3, 4)

        q.normalize()
        self.assertAlmostEqual(q, Quaterniond(0.182574, 0.365148,
                                              0.547723, 0.730297))

    def test_math(self):
        q = Quaterniond(math.pi*0.1, math.pi*0.5, math.pi)
        self.assertTrue(q == Quaterniond(0.110616, -0.698401,
                                         0.110616, 0.698401))

        q.set(1, 2, 3, 4)

        q.normalize()

        self.assertAlmostEqual(q.roll(), 1.4289, delta=1e-3)
        self.assertAlmostEqual(q.pitch(), -0.339837, delta=1e-3)
        self.assertAlmostEqual(q.yaw(), 2.35619, delta=1e-3)

        axis, angle = q.to_axis()
        self.assertAlmostEqual(
            axis, Vector3d(0.371391, 0.557086, 0.742781), delta=1e-3)
        self.assertAlmostEqual(angle, 2.77438, delta=1e-3)

        q.scale(0.1)
        self.assertTrue(q == Quaterniond(0.990394, 0.051354,
                                         0.0770309, 0.102708))

        q = q + Quaterniond(0, 1, 2)
        self.assertTrue(q == Quaterniond(1.46455, -0.352069,
                                         0.336066, 0.841168))

        q += q
        self.assertTrue(q == Quaterniond(2.92911, -0.704137,
                                         0.672131, 1.68234))

        q -= Quaterniond(.4, .2, .1)
        self.assertTrue(q == Quaterniond(1.95416, -0.896677, 0.56453, 1.65341))

        q = q - Quaterniond(0, 1, 2)
        self.assertTrue(q == Quaterniond(1.48, -0.493254,
                                         0.305496, 0.914947))

        q *= Quaterniond(.4, .1, .01)
        self.assertTrue(q == Quaterniond(1.53584, -0.236801,
                                         0.551841, 0.802979))

        q = q * 5.0
        self.assertTrue(q == Quaterniond(7.67918, -1.184, 2.7592, 4.0149))

        self.assertTrue(q.rotate_vector_reverse(Vector3d(1, 2, 3)) ==
                        Vector3d(-0.104115, 0.4975, 3.70697))

        self.assertAlmostEqual(q.dot(Quaterniond(.4, .2, .1)), 7.67183,
                               delta=1e-3)

        self.assertTrue(Quaterniond.squad(1.1, Quaterniond(.1, 0, .2),
                        Quaterniond(0, .3, .4), Quaterniond(.5, .2, 1),
                        Quaterniond(0, 0, 2), True) ==
                        Quaterniond(0.346807, -0.0511734,
                                    -0.0494723, 0.935232))

        self.assertTrue(Quaterniond.euler_to_quaternion(
                        Vector3d(.1, .2, .3)) ==
                        Quaterniond(0.983347, 0.0342708,
                                    0.106021, 0.143572))

        q.round(2)
        self.assertAlmostEqual(-1.18, q.x())
        self.assertAlmostEqual(2.76, q.y())
        self.assertAlmostEqual(4.01, q.z())
        self.assertAlmostEqual(7.68, q.w())

        q.x(0.0)
        q.y(0.0)
        q.z(0.0)
        q.w(0.0)
        q.normalize()
        self.assertTrue(q == Quaterniond())

        q.axis(0, 0, 0, 0)
        self.assertTrue(q == Quaterniond())

        self.assertTrue(Quaterniond.euler_to_quaternion(0.1, 0.2, 0.3) ==
                        Quaterniond(0.983347, 0.0342708, 0.106021, 0.143572))

        # to_axis() method
        q.x(0.0)
        q.y(0.0)
        q.z(0.0)
        q.w(0.0)
        axis, angle = q.to_axis()
        self.assertAlmostEqual(axis, Vector3d(1., 0., 0.), delta=1e-3)
        self.assertAlmostEqual(angle, 0., delta=1e-3)

        # simple 180 rotation about yaw,
        # should result in x and y flipping signs
        q = Quaterniond(0, 0, math.pi)
        v = Vector3d(1, 2, 3)
        r1 = q.rotate_vector(v)
        r2 = q.rotate_vector_reverse(v)
        self.assertTrue(r1 == Vector3d(-1, -2, 3))
        self.assertTrue(r2 == Vector3d(-1, -2, 3))

        # simple  90 rotation about yaw, should map x to y, y to -x
        # simple -90 rotation about yaw, should map x to -y, y to x
        q = Quaterniond(0, 0, 0.5 * math.pi)
        v = Vector3d(1, 2, 3)
        r1 = q.rotate_vector(v)
        r2 = q.rotate_vector_reverse(v)
        self.assertTrue(r1 == Vector3d(-2, 1, 3))
        self.assertTrue(r2 == Vector3d(2, -1, 3))
        self.assertTrue(q.inverse().x_axis() == Vector3d(0, -1, 0))
        self.assertTrue(q.inverse().y_axis() == Vector3d(1, 0, 0))
        self.assertTrue(q.inverse().z_axis() == Vector3d(0, 0, 1))

        # Test RPY fixed-body-frame convention:
        # Rotate each unit vector in roll and pitch
        q = Quaterniond(math.pi/2.0, math.pi/2.0, 0)
        v1 = Vector3d(1, 0, 0)
        r1 = q.rotate_vector(v1)
        # 90 degrees about x does nothing,
        # 90 degrees about y sends point down to -z
        self.assertAlmostEqual(r1, Vector3d(0, 0, -1))

        v2 = Vector3d(0, 1, 0)
        r2 = q.rotate_vector(v2)
        # 90 degrees about x sends point to +z
        # 90 degrees about y sends point to +x
        self.assertAlmostEqual(r2, Vector3d(1, 0, 0))

        v3 = Vector3d(0, 0, 1)
        r3 = q.rotate_vector(v3)
        # 90 degrees about x sends point to -y
        # 90 degrees about y does nothing
        self.assertAlmostEqual(r3, Vector3d(0, -1, 0))

        # now try a harder case (axis[1,2,3], rotation[0.3*pi])
        # verified with octave
        q.axis(Vector3d(1, 2, 3), 0.3*math.pi)
        self.assertTrue(q.inverse().x_axis() ==
                        Vector3d(0.617229, -0.589769, 0.520770))
        self.assertTrue(q.inverse().y_axis() ==
                        Vector3d(0.707544, 0.705561, -0.039555))
        self.assertTrue(q.inverse().z_axis() ==
                        Vector3d(-0.344106, 0.392882, 0.852780))

        # rotate about the axis of rotation should not change axis
        v = Vector3d(1, 2, 3)
        r1 = q.rotate_vector(v)
        r2 = q.rotate_vector_reverse(v)
        self.assertTrue(r1 == Vector3d(1, 2, 3))
        self.assertTrue(r2 == Vector3d(1, 2, 3))

        # rotate unit vectors
        v = Vector3d(0, 0, 1)
        r1 = q.rotate_vector(v)
        r2 = q.rotate_vector_reverse(v)
        self.assertTrue(r1 == Vector3d(0.520770, -0.039555, 0.852780))
        self.assertTrue(r2 == Vector3d(-0.34411, 0.39288, 0.85278))
        v = Vector3d(0, 1, 0)
        r1 = q.rotate_vector(v)
        r2 = q.rotate_vector_reverse(v)
        self.assertTrue(r1 == Vector3d(-0.58977, 0.70556, 0.39288))
        self.assertTrue(r2 == Vector3d(0.707544, 0.705561, -0.039555))
        v = Vector3d(1, 0, 0)
        r1 = q.rotate_vector(v)
        r2 = q.rotate_vector_reverse(v)
        self.assertTrue(r1 == Vector3d(0.61723, 0.70754, -0.34411))
        self.assertTrue(r2 == Vector3d(0.61723, -0.58977, 0.52077))

        self.assertTrue(-q == Quaterniond(-0.891007, -0.121334,
                                          -0.242668, -0.364002))

        self.assertTrue(Matrix3d(q) == Matrix3d(
                    0.617229, -0.589769, 0.52077,
                    0.707544, 0.705561, -0.0395554,
                    -0.344106, 0.392882, 0.85278))

        matFromQ = Matrix3d(q)
        self.assertTrue(matFromQ == Matrix3d(
                    0.617229, -0.589769, 0.52077,
                    0.707544, 0.705561, -0.0395554,
                    -0.344106, 0.392882, 0.85278))

        self.assertTrue(Matrix4d(q) == Matrix4d(
                    0.617229, -0.589769, 0.52077, 0,
                    0.707544, 0.705561, -0.0395554, 0,
                    -0.344106, 0.392882, 0.85278, 0,
                    0, 0, 0, 1))

    def test_stream_out(self):
        q = Quaterniond(0.1, 1.2, 2.3)
        self.assertEqual(str(q), "0.1 1.2 2.3")

    def test_slerp(self):
        q1 = Quaterniond(0.1, 1.2, 2.3)
        q2 = Quaterniond(1.2, 2.3, -3.4)

        q3 = Quaterniond.slerp(1.0, q1, q2, True)
        self.assertAlmostEqual(q3, Quaterniond(0.554528, -0.717339,
                                               0.32579, 0.267925))

    def test_from_2_axes(self):
        v1 = Vector3d(1.0, 0.0, 0.0)
        v2 = Vector3d(0.0, 1.0, 0.0)

        q1 = Quaterniond()
        q1.from_2_axes(v1, v2)

        q2 = Quaterniond()
        q2.from_2_axes(v2, v1)

        q2Correct = Quaterniond(math.sqrt(2)/2, 0, 0, -math.sqrt(2)/2)
        q1Correct = Quaterniond(math.sqrt(2)/2, 0, 0, math.sqrt(2)/2)

        self.assertNotEqual(q1, q2)
        self.assertAlmostEqual(q1Correct, q1)
        self.assertAlmostEqual(q2Correct, q2)
        self.assertAlmostEqual(Quaterniond.IDENTITY, q1 * q2)
        self.assertAlmostEqual(v2, q1 * v1)
        self.assertAlmostEqual(v1, q2 * v2)

        # still the same rotation, but with non-unit vectors
        v1.set(0.5, 0.5, 0)
        v2.set(-0.5, 0.5, 0)

        q1.from_2_axes(v1, v2)
        q2.from_2_axes(v2, v1)

        self.assertNotEqual(q1, q2)
        self.assertAlmostEqual(q1Correct, q1)
        self.assertAlmostEqual(q2Correct, q2)
        self.assertAlmostEqual(Quaterniond.IDENTITY, q1 * q2)
        self.assertAlmostEqual(v2, q1 * v1)
        self.assertAlmostEqual(v1, q2 * v2)

        # Test various settings of opposite vectors (which need special care)
        tolerance = 1e-4

        v1.set(1, 0, 0)
        v2.set(-1, 0, 0)
        q1.from_2_axes(v1, v2)
        q2 = q1 * q1
        self.assertTrue(abs(q2.w()-1.0) <= tolerance or
                        abs(q2.w()-(-1.0)) <= tolerance)
        self.assertAlmostEqual(q2.x(), 0.0)
        self.assertAlmostEqual(q2.y(), 0.0)
        self.assertAlmostEqual(q2.z(), 0.0)

        v1.set(0, 1, 0)
        v2.set(0, -1, 0)
        q1.from_2_axes(v1, v2)
        q2 = q1 * q1
        self.assertTrue(abs(q2.w()-1.0) <= tolerance or
                        abs(q2.w()-(-1.0)) <= tolerance)
        self.assertAlmostEqual(q2.x(), 0.0)
        self.assertAlmostEqual(q2.y(), 0.0)
        self.assertAlmostEqual(q2.z(), 0.0)

        v1.set(0, 0, 1)
        v2.set(0, 0, -1)
        q1.from_2_axes(v1, v2)
        q2 = q1 * q1
        self.assertTrue(abs(q2.w()-1.0) <= tolerance or
                        abs(q2.w()-(-1.0)) <= tolerance)
        self.assertAlmostEqual(q2.x(), 0.0)
        self.assertAlmostEqual(q2.y(), 0.0)
        self.assertAlmostEqual(q2.z(), 0.0)

        v1.set(0, 1, 1)
        v2.set(0, -1, -1)
        q1.from_2_axes(v1, v2)
        q2 = q1 * q1
        self.assertTrue(abs(q2.w()-1.0) <= tolerance or
                        abs(q2.w()-(-1.0)) <= tolerance)
        self.assertAlmostEqual(q2.x(), 0.0)
        self.assertAlmostEqual(q2.y(), 0.0)
        self.assertAlmostEqual(q2.z(), 0.0)

    def test_integrate(self):
        # integrate by zero, expect no change
        q = Quaterniond(0.5, 0.5, 0.5, 0.5)
        self.assertAlmostEqual(q, q.integrate(Vector3d.ZERO, 1.0))
        self.assertAlmostEqual(q, q.integrate(Vector3d.UNIT_X, 0.0))
        self.assertAlmostEqual(q, q.integrate(Vector3d.UNIT_Y, 0.0))
        self.assertAlmostEqual(q, q.integrate(Vector3d.UNIT_Z, 0.0))

        # integrate along single axes,
        # expect linear change in roll, pitch, yaw
        q = Quaterniond(1, 0, 0, 0)
        qRoll = q.integrate(Vector3d.UNIT_X, 1.0)
        qPitch = q.integrate(Vector3d.UNIT_Y, 1.0)
        qYaw = q.integrate(Vector3d.UNIT_Z, 1.0)
        self.assertAlmostEqual(qRoll.euler(), Vector3d.UNIT_X)
        self.assertAlmostEqual(qPitch.euler(), Vector3d.UNIT_Y)
        self.assertAlmostEqual(qYaw.euler(), Vector3d.UNIT_Z)

        # integrate sequentially along single axes in order XYZ,
        # expect rotations to match euler Angles
        q = Quaterniond(1, 0, 0, 0)
        angle = 0.5
        qX = q.integrate(Vector3d.UNIT_X, angle)
        qXY = qX.integrate(Vector3d.UNIT_Y, angle)
        self.assertAlmostEqual(qXY.euler(), Vector3d(1, 1, 0)*angle)

        q = Quaterniond(1, 0, 0, 0)
        angle = 0.5
        qX = q.integrate(Vector3d.UNIT_X, angle)
        qXZ = qX.integrate(Vector3d.UNIT_Z, angle)
        self.assertAlmostEqual(qXZ.euler(), Vector3d(1, 0, 1)*angle)

        q = Quaterniond(1, 0, 0, 0)
        angle = 0.5
        qY = q.integrate(Vector3d.UNIT_Y, angle)
        qYZ = qY.integrate(Vector3d.UNIT_Z, angle)
        self.assertAlmostEqual(qYZ.euler(), Vector3d(0, 1, 1)*angle)

        q = Quaterniond(1, 0, 0, 0)
        angle = 0.5
        qX = q.integrate(Vector3d.UNIT_X, angle)
        qXY = qX.integrate(Vector3d.UNIT_Y, angle)
        qXYZ = qXY.integrate(Vector3d.UNIT_Z, angle)
        self.assertAlmostEqual(qXYZ.euler(), Vector3d.ONE*angle)

        # integrate sequentially along single axes in order ZYX,
        # expect rotations to not match euler Angles
        q = Quaterniond(1, 0, 0, 0)
        angle = 0.5
        qZ = q.integrate(Vector3d.UNIT_Z, angle)
        qZY = qZ.integrate(Vector3d.UNIT_Y, angle)
        self.assertNotEqual(qZY.euler(), Vector3d(0, 1, 1)*angle)

        q = Quaterniond(1, 0, 0, 0)
        angle = 0.5
        qZ = q.integrate(Vector3d.UNIT_Z, angle)
        qZX = qZ.integrate(Vector3d.UNIT_X, angle)
        self.assertNotEqual(qZX.euler(), Vector3d(1, 0, 1)*angle)

        q = Quaterniond(1, 0, 0, 0)
        angle = 0.5
        qZ = q.integrate(Vector3d.UNIT_Z, angle)
        qZY = qZ.integrate(Vector3d.UNIT_Y, angle)
        qZYX = qZY.integrate(Vector3d.UNIT_X, angle)
        self.assertNotEqual(qZYX.euler(), Vector3d(1, 1, 1)*angle)

        q = Quaterniond(1, 0, 0, 0)
        angle = 0.5
        qY = q.integrate(Vector3d.UNIT_Y, angle)
        qYX = qY.integrate(Vector3d.UNIT_X, angle)
        self.assertNotEqual(qYX.euler(), Vector3d(1, 1, 0)*angle)

        # integrate a full rotation about different axes,
        # expect no change.
        q = Quaterniond(0.5, 0.5, 0.5, 0.5)
        fourPi = 4 * math.pi
        qX = q.integrate(Vector3d.UNIT_X, fourPi)
        qY = q.integrate(Vector3d.UNIT_Y, fourPi)
        qZ = q.integrate(Vector3d.UNIT_Z, fourPi)
        self.assertAlmostEqual(q, qX)
        self.assertAlmostEqual(q, qY)
        self.assertAlmostEqual(q, qZ)


if __name__ == '__main__':
    unittest.main()
