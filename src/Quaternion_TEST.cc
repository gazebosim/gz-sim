/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include <cmath>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Quaternion.hh"
#include "ignition/math/Matrix3.hh"
#include "ignition/math/Matrix4.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(QuaternionTest, Unit)
{
  math::Quaterniond q;
  EXPECT_TRUE(math::equal(q.W(), 1.0));
  EXPECT_TRUE(math::equal(q.X(), 0.0));
  EXPECT_TRUE(math::equal(q.Y(), 0.0));
  EXPECT_TRUE(math::equal(q.Z(), 0.0));
}

/////////////////////////////////////////////////
TEST(QuaternionTest, ConstructWithValues)
{
  math::Quaterniond q(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(q.W(), 1.0));
  EXPECT_TRUE(math::equal(q.X(), 2.0));
  EXPECT_TRUE(math::equal(q.Y(), 3.0));
  EXPECT_TRUE(math::equal(q.Z(), 4.0));
}

/////////////////////////////////////////////////
TEST(QuaternionTest, ConstructZero)
{
  math::Quaterniond q(0, 0, 0, 0);
  EXPECT_TRUE(math::equal(q.W(), 0.0));
  EXPECT_TRUE(math::equal(q.X(), 0.0));
  EXPECT_TRUE(math::equal(q.Y(), 0.0));
  EXPECT_TRUE(math::equal(q.Z(), 0.0));

  math::Quaterniond qI = q.Inverse();
  EXPECT_TRUE(math::equal(qI.W(), 1.0));
  EXPECT_TRUE(math::equal(qI.X(), 0.0));
  EXPECT_TRUE(math::equal(qI.Y(), 0.0));
  EXPECT_TRUE(math::equal(qI.Z(), 0.0));
}

/////////////////////////////////////////////////
TEST(QuaternionTest, ConstructEuler)
{
  math::Quaterniond q(0, 1, 2);
  EXPECT_TRUE(q == math::Quaterniond(math::Vector3d(0, 1, 2)));

  // Make sure that singularities are being handled properly.
  // There are an infinite number of equivalent Euler angle
  // representations when pitch = PI/2, so rather than comparing Euler
  // angles, we will compare quaternions.
  for (double pitch : { -IGN_PI_2, IGN_PI_2 })
  {
    for (double roll = 0; roll < 2 * IGN_PI + 0.1; roll += IGN_PI_4)
    {
      for (double yaw = 0; yaw < 2 * IGN_PI + 0.1; yaw += IGN_PI_4)
      {
        math::Quaterniond q_orig(roll, pitch, yaw);
        math::Quaterniond q_derived(q_orig.Euler());
        EXPECT_TRUE(q_orig == q_derived || q_orig == -q_derived);
      }
    }
  }
}

/////////////////////////////////////////////////
TEST(QuaternionTest, ConstructAxisAngle)
{
  math::Quaterniond q1(math::Vector3d(0, 0, 1), IGN_PI);
  EXPECT_TRUE(math::equal(q1.X(), 0.0));
  EXPECT_TRUE(math::equal(q1.Y(), 0.0));
  EXPECT_TRUE(math::equal(q1.Z(), 1.0));
  EXPECT_TRUE(math::equal(q1.W(), 0.0));

  math::Quaterniond q(q1);
  EXPECT_TRUE(q == q1);
}

/////////////////////////////////////////////////
TEST(QuaternionTest, Identity)
{
  math::Quaterniond q = math::Quaterniond::Identity;
  EXPECT_TRUE(math::equal(q.W(), 1.0));
  EXPECT_TRUE(math::equal(q.X(), 0.0));
  EXPECT_TRUE(math::equal(q.Y(), 0.0));
  EXPECT_TRUE(math::equal(q.Z(), 0.0));
}

//////////////////////////////////////////////////
TEST(QuaternionTest, Integrate)
{
  // Integrate by zero, expect no change
  {
    const math::Quaterniond q(0.5, 0.5, 0.5, 0.5);
    EXPECT_EQ(q, q.Integrate(math::Vector3d::Zero, 1.0));
    EXPECT_EQ(q, q.Integrate(math::Vector3d::UnitX, 0.0));
    EXPECT_EQ(q, q.Integrate(math::Vector3d::UnitY, 0.0));
    EXPECT_EQ(q, q.Integrate(math::Vector3d::UnitZ, 0.0));
  }

  // Integrate along single axes,
  // expect linear change in roll, pitch, yaw
  {
    const math::Quaterniond q(1, 0, 0, 0);
    math::Quaterniond qRoll  = q.Integrate(math::Vector3d::UnitX, 1.0);
    math::Quaterniond qPitch = q.Integrate(math::Vector3d::UnitY, 1.0);
    math::Quaterniond qYaw   = q.Integrate(math::Vector3d::UnitZ, 1.0);
    EXPECT_EQ(qRoll.Euler(),  math::Vector3d::UnitX);
    EXPECT_EQ(qPitch.Euler(), math::Vector3d::UnitY);
    EXPECT_EQ(qYaw.Euler(),   math::Vector3d::UnitZ);
  }

  // Integrate sequentially along single axes in order XYZ,
  // expect rotations to match Euler Angles
  {
    const math::Quaterniond q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaterniond qX   = q.Integrate(math::Vector3d::UnitX, angle);
    math::Quaterniond qXY  = qX.Integrate(math::Vector3d::UnitY, angle);
    EXPECT_EQ(qXY.Euler(), angle*math::Vector3d(1, 1, 0));
  }
  {
    const math::Quaterniond q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaterniond qX   = q.Integrate(math::Vector3d::UnitX, angle);
    math::Quaterniond qXZ  = qX.Integrate(math::Vector3d::UnitZ, angle);
    EXPECT_EQ(qXZ.Euler(), angle*math::Vector3d(1, 0, 1));
  }
  {
    const math::Quaterniond q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaterniond qY   = q.Integrate(math::Vector3d::UnitY, angle);
    math::Quaterniond qYZ  = qY.Integrate(math::Vector3d::UnitZ, angle);
    EXPECT_EQ(qYZ.Euler(), angle*math::Vector3d(0, 1, 1));
  }
  {
    const math::Quaterniond q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaterniond qX   = q.Integrate(math::Vector3d::UnitX, angle);
    math::Quaterniond qXY  = qX.Integrate(math::Vector3d::UnitY, angle);
    math::Quaterniond qXYZ = qXY.Integrate(math::Vector3d::UnitZ, angle);
    EXPECT_EQ(qXYZ.Euler(), angle*math::Vector3d::One);
  }

  // Integrate sequentially along single axes in order ZYX,
  // expect rotations to not match Euler Angles
  {
    const math::Quaterniond q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaterniond qZ   = q.Integrate(math::Vector3d::UnitZ, angle);
    math::Quaterniond qZY  = qZ.Integrate(math::Vector3d::UnitY, angle);
    EXPECT_NE(qZY.Euler(), angle*math::Vector3d(0, 1, 1));
  }
  {
    const math::Quaterniond q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaterniond qZ   = q.Integrate(math::Vector3d::UnitZ, angle);
    math::Quaterniond qZX  = qZ.Integrate(math::Vector3d::UnitX, angle);
    EXPECT_NE(qZX.Euler(), angle*math::Vector3d(1, 0, 1));
  }
  {
    const math::Quaterniond q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaterniond qZ   = q.Integrate(math::Vector3d::UnitZ, angle);
    math::Quaterniond qZY  = qZ.Integrate(math::Vector3d::UnitY, angle);
    math::Quaterniond qZYX = qZY.Integrate(math::Vector3d::UnitX, angle);
    EXPECT_NE(qZYX.Euler(), angle*math::Vector3d(1, 1, 1));
  }
  {
    const math::Quaterniond q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaterniond qY   = q.Integrate(math::Vector3d::UnitY, angle);
    math::Quaterniond qYX  = qY.Integrate(math::Vector3d::UnitX, angle);
    EXPECT_NE(qYX.Euler(), angle*math::Vector3d(1, 1, 0));
  }

  // Integrate a full rotation about different axes,
  // expect no change.
  {
    const math::Quaterniond q(0.5, 0.5, 0.5, 0.5);
    const double fourPi = 4 * IGN_PI;
    math::Quaterniond qX = q.Integrate(math::Vector3d::UnitX, fourPi);
    math::Quaterniond qY = q.Integrate(math::Vector3d::UnitY, fourPi);
    math::Quaterniond qZ = q.Integrate(math::Vector3d::UnitZ, fourPi);
    EXPECT_EQ(q, qX);
    EXPECT_EQ(q, qY);
    EXPECT_EQ(q, qZ);
  }
}

/////////////////////////////////////////////////
TEST(QuaternionTest, Math)
{
  math::Quaterniond q(IGN_PI*0.1, IGN_PI*0.5, IGN_PI);
  EXPECT_TRUE(q == math::Quaterniond(0.110616, -0.698401, 0.110616, 0.698401));

  EXPECT_TRUE(q.Log() ==
      math::Quaterniond(0, -1.02593, 0.162491, 1.02593));

  EXPECT_TRUE(q.Exp() ==
      math::Quaterniond(0.545456, -0.588972, 0.093284, 0.588972));

  math::Quaterniond q1 = q;
  q1.W(2.0);
  EXPECT_TRUE(q1.Log() ==
      math::Quaterniond(0, -0.698401, 0.110616, 0.698401));

  q1.X(0.000000001);
  q1.Y(0.0);
  q1.Z(0.0);
  q1.W(0.0);
  EXPECT_TRUE(q1.Exp() == math::Quaterniond(1, 0, 0, 0));

  q.Invert();
  EXPECT_TRUE(q == math::Quaterniond(0.110616, 0.698401, -0.110616, -0.698401));

  q.Axis(0, 1, 0, IGN_PI);
  EXPECT_TRUE(q == math::Quaterniond(6.12303e-17, 0, 1, 0));

  q.Axis(math::Vector3d(1, 0, 0), IGN_PI);
  EXPECT_TRUE(q == math::Quaterniond(0, 1, 0, 0));

  q.Set(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(q.W(), 1.0));
  EXPECT_TRUE(math::equal(q.X(), 2.0));
  EXPECT_TRUE(math::equal(q.Y(), 3.0));
  EXPECT_TRUE(math::equal(q.Z(), 4.0));

  q.Normalize();
  EXPECT_TRUE(q == math::Quaterniond(0.182574, 0.365148, 0.547723, 0.730297));


  EXPECT_TRUE(math::equal(q.Roll(), 1.4289, 1e-3));
  EXPECT_TRUE(math::equal(q.Pitch(), -0.339837, 1e-3));
  EXPECT_TRUE(math::equal(q.Yaw(), 2.35619, 1e-3));

  math::Vector3d axis;
  double angle;
  q.ToAxis(axis, angle);
  EXPECT_TRUE(axis == math::Vector3d(0.371391, 0.557086, 0.742781));
  EXPECT_TRUE(math::equal(angle, 2.77438, 1e-3));

  q.Scale(0.1);
  EXPECT_TRUE(q == math::Quaterniond(0.990394, 0.051354, 0.0770309, 0.102708));

  q = q + math::Quaterniond(0, 1, 2);
  EXPECT_TRUE(q == math::Quaterniond(1.46455, -0.352069, 0.336066, 0.841168));

  q += q;
  EXPECT_TRUE(q == math::Quaterniond(2.92911, -0.704137, 0.672131, 1.68234));

  q -= math::Quaterniond(.4, .2, .1);
  EXPECT_TRUE(q == math::Quaterniond(1.95416, -0.896677, 0.56453, 1.65341));

  q = q - math::Quaterniond(0, 1, 2);
  EXPECT_TRUE(q == math::Quaterniond(1.48, -0.493254, 0.305496, 0.914947));

  q *= math::Quaterniond(.4, .1, .01);
  EXPECT_TRUE(q == math::Quaterniond(1.53584, -0.236801, 0.551841, 0.802979));

  q = q * 5.0;
  EXPECT_TRUE(q == math::Quaterniond(7.67918, -1.184, 2.7592, 4.0149));

  EXPECT_TRUE(q.RotateVectorReverse(math::Vector3d(1, 2, 3)) ==
      math::Vector3d(-0.104115, 0.4975, 3.70697));

  EXPECT_TRUE(math::equal(q.Dot(math::Quaterniond(.4, .2, .1)), 7.67183, 1e-3));

  EXPECT_TRUE(math::Quaterniond::Squad(1.1, math::Quaterniond(.1, 0, .2),
        math::Quaterniond(0, .3, .4), math::Quaterniond(.5, .2, 1),
        math::Quaterniond(0, 0, 2), true) ==
      math::Quaterniond(0.346807, -0.0511734, -0.0494723, 0.935232));

  EXPECT_TRUE(math::Quaterniond::EulerToQuaternion(
        math::Vector3d(.1, .2, .3)) ==
      math::Quaterniond(0.983347, 0.0342708, 0.106021, 0.143572));

  q.Round(2);
  EXPECT_TRUE(math::equal(-1.18, q.X()));
  EXPECT_TRUE(math::equal(2.76, q.Y()));
  EXPECT_TRUE(math::equal(4.01, q.Z()));
  EXPECT_TRUE(math::equal(7.68, q.W()));

  q.X(0.0);
  q.Y(0.0);
  q.Z(0.0);
  q.W(0.0);
  q.Normalize();
  EXPECT_TRUE(q == math::Quaterniond());

  q.Axis(0, 0, 0, 0);
  EXPECT_TRUE(q == math::Quaterniond());

  EXPECT_TRUE(math::Quaterniond::EulerToQuaternion(0.1, 0.2, 0.3) ==
      math::Quaterniond(0.983347, 0.0342708, 0.106021, 0.143572));

  q.X(0.0);
  q.Y(0.0);
  q.Z(0.0);
  q.W(0.0);
  q.ToAxis(axis, angle);
  EXPECT_TRUE(axis == math::Vector3d(1, 0, 0));
  EXPECT_TRUE(math::equal(angle, 0.0, 1e-3));
  {
    // simple 180 rotation about yaw, should result in x and y flipping signs
    q = math::Quaterniond(0, 0, IGN_PI);
    math::Vector3d v = math::Vector3d(1, 2, 3);
    math::Vector3d r1 = q.RotateVector(v);
    math::Vector3d r2 = q.RotateVectorReverse(v);
    std::cout << "[" << q.W() << ", " << q.X() << ", "
      << q.Y() << ", " << q.Z() << "]\n";
    std::cout << " forward turns [" << v << "] to [" << r1 << "]\n";
    std::cout << " reverse turns [" << v << "] to [" << r2 << "]\n";
    EXPECT_TRUE(r1 == math::Vector3d(-1, -2, 3));
    EXPECT_TRUE(r2 == math::Vector3d(-1, -2, 3));
  }

  {
    // simple  90 rotation about yaw, should map x to y, y to -x
    // simple -90 rotation about yaw, should map x to -y, y to x
    q = math::Quaterniond(0, 0, 0.5*IGN_PI);
    math::Vector3d v = math::Vector3d(1, 2, 3);
    math::Vector3d r1 = q.RotateVector(v);
    math::Vector3d r2 = q.RotateVectorReverse(v);
    std::cout << "[" << q.W() << ", " << q.X() << ", "
      << q.Y() << ", " << q.Z() << "]\n";
    std::cout << " forward turns [" << v << "] to [" << r1 << "]\n";
    std::cout << " reverse turns [" << v << "] to [" << r2 << "]\n";
    std::cout << " x axis [" << q.XAxis() << "]\n";
    std::cout << " y axis [" << q.YAxis() << "]\n";
    std::cout << " z axis [" << q.ZAxis() << "]\n";
    EXPECT_TRUE(r1 == math::Vector3d(-2, 1, 3));
    EXPECT_TRUE(r2 == math::Vector3d(2, -1, 3));
    EXPECT_TRUE(q.Inverse().XAxis() == math::Vector3d(0, -1, 0));
    EXPECT_TRUE(q.Inverse().YAxis() == math::Vector3d(1, 0, 0));
    EXPECT_TRUE(q.Inverse().ZAxis() == math::Vector3d(0, 0, 1));
  }

  // Test RPY fixed-body-frame convention:
  // Rotate each unit vector in roll and pitch
  {
    q = math::Quaterniond(IGN_PI/2.0, IGN_PI/2.0, 0);
    math::Vector3d v1(1, 0, 0);
    math::Vector3d r1 = q.RotateVector(v1);
    // 90 degrees about X does nothing,
    // 90 degrees about Y sends point down to -Z
    EXPECT_EQ(r1, math::Vector3d(0, 0, -1));

    math::Vector3d v2(0, 1, 0);
    math::Vector3d r2 = q.RotateVector(v2);
    // 90 degrees about X sends point to +Z
    // 90 degrees about Y sends point to +X
    EXPECT_EQ(r2, math::Vector3d(1, 0, 0));

    math::Vector3d v3(0, 0, 1);
    math::Vector3d r3 = q.RotateVector(v3);
    // 90 degrees about X sends point to -Y
    // 90 degrees about Y does nothing
    EXPECT_EQ(r3, math::Vector3d(0, -1, 0));
  }

  {
    // now try a harder case (axis[1,2,3], rotation[0.3*pi])
    // verified with octave
    q.Axis(math::Vector3d(1, 2, 3), 0.3*IGN_PI);
    std::cout << "[" << q.W() << ", " << q.X() << ", "
      << q.Y() << ", " << q.Z() << "]\n";
    std::cout << " x [" << q.Inverse().XAxis() << "]\n";
    std::cout << " y [" << q.Inverse().YAxis() << "]\n";
    std::cout << " z [" << q.Inverse().ZAxis() << "]\n";
    EXPECT_TRUE(q.Inverse().XAxis() ==
                math::Vector3d(0.617229, -0.589769, 0.520770));
    EXPECT_TRUE(q.Inverse().YAxis() ==
                math::Vector3d(0.707544, 0.705561, -0.039555));
    EXPECT_TRUE(q.Inverse().ZAxis() ==
                math::Vector3d(-0.344106, 0.392882, 0.852780));

    // rotate about the axis of rotation should not change axis
    math::Vector3d v = math::Vector3d(1, 2, 3);
    math::Vector3d r1 = q.RotateVector(v);
    math::Vector3d r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3d(1, 2, 3));
    EXPECT_TRUE(r2 == math::Vector3d(1, 2, 3));

    // rotate unit vectors
    v = math::Vector3d(0, 0, 1);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3d(0.520770, -0.039555, 0.852780));
    EXPECT_TRUE(r2 == math::Vector3d(-0.34411, 0.39288, 0.85278));
    v = math::Vector3d(0, 1, 0);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3d(-0.58977, 0.70556, 0.39288));
    EXPECT_TRUE(r2 == math::Vector3d(0.707544, 0.705561, -0.039555));
    v = math::Vector3d(1, 0, 0);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3d(0.61723, 0.70754, -0.34411));
    EXPECT_TRUE(r2 == math::Vector3d(0.61723, -0.58977, 0.52077));

    EXPECT_TRUE(-q == math::Quaterniond(-0.891007, -0.121334,
                                       -0.242668, -0.364002));

    EXPECT_TRUE(math::Matrix3d(q) == math::Matrix3d(
                0.617229, -0.589769, 0.52077,
                0.707544, 0.705561, -0.0395554,
                -0.344106, 0.392882, 0.85278));

    math::Matrix3d matFromQ;
    matFromQ = q;
    EXPECT_TRUE(matFromQ == math::Matrix3d(
                0.617229, -0.589769, 0.52077,
                0.707544, 0.705561, -0.0395554,
                -0.344106, 0.392882, 0.85278));

    EXPECT_TRUE(math::Matrix4d(q) == math::Matrix4d(
                0.617229, -0.589769, 0.52077, 0,
                0.707544, 0.705561, -0.0395554, 0,
                -0.344106, 0.392882, 0.85278, 0,
                0, 0, 0, 1));

    math::Matrix3d matFromQuat(q);

    math::Quaterniond quatFromMat(matFromQuat);
    math::Quaterniond quatFromMat2; quatFromMat2.Matrix(matFromQuat);

    EXPECT_TRUE(q == quatFromMat);
    EXPECT_TRUE(q == quatFromMat2);

    // test the cases where matrix trace is negative (requires special handling)
    q = math::Quaterniond(0, 0, 0, 1);
    EXPECT_TRUE(q == math::Quaterniond(math::Matrix3d(
                -1,  0, 0,
                 0, -1, 0,
                 0,  0, 1)));

    q = math::Quaterniond(0, 0, 1, 0);
    EXPECT_TRUE(q == math::Quaterniond(math::Matrix3d(
                -1,  0,  0,
                 0,  1,  0,
                 0,  0, -1)));

    q = math::Quaterniond(0, 1, 0, 0);
    EXPECT_TRUE(q == math::Quaterniond(math::Matrix3d(
                1,  0,  0,
                0, -1,  0,
                0,  0, -1)));
  }
}

/////////////////////////////////////////////////
TEST(QuaternionTest, MultiplicationOrder)
{
  math::Quaterniond q1(0, 0, 1.0);
  math::Quaterniond q2(0.1, 0, 0);
  math::Quaterniond q3(0, 0, -1.0);
  math::Vector3d v(1, 2, 3);
  math::Matrix3d m1(q1), m2(q2), m3(q3);
  EXPECT_EQ(q1*v, m1*v);
  EXPECT_EQ(q2*v, m2*v);
  EXPECT_EQ(q3*v, m3*v);
  EXPECT_EQ(q1*q2*v, m1*m2*v);
  EXPECT_EQ(q2*q3*v, m2*m3*v);
  EXPECT_EQ(q1*q2*q3*v, m1*m2*m3*v);
  EXPECT_EQ(math::Matrix3d(q1*q2*q3), m1*m2*m3);
  EXPECT_EQ(math::Matrix3d(q3*q2*q1), m3*m2*m1);
  // ensure that it's not commutative
  EXPECT_NE(math::Matrix3d(q1*q2*q3), m3*m2*m1);
}

/////////////////////////////////////////////////
TEST(QuaternionTest, OperatorStreamOut)
{
  math::Quaterniond q(0.1, 1.2, 2.3);
  std::ostringstream stream;
  stream << q;
  EXPECT_EQ(stream.str(), "0.1 1.2 2.3");
}

/////////////////////////////////////////////////
TEST(QuaternionTest, Slerp)
{
  math::Quaterniond q1(0.1, 1.2, 2.3);
  math::Quaterniond q2(1.2, 2.3, -3.4);

  math::Quaterniond q3 = math::Quaterniond::Slerp(1.0, q1, q2, true);
  EXPECT_EQ(q3, math::Quaterniond(0.554528, -0.717339, 0.32579, 0.267925));
}

/////////////////////////////////////////////////
TEST(QuaterniondTest, From2Axes)
{
  math::Vector3d v1(1.0, 0.0, 0.0);
  math::Vector3d v2(0.0, 1.0, 0.0);

  math::Quaterniond q1;
  q1.From2Axes(v1, v2);

  math::Quaterniond q2;
  q2.From2Axes(v2, v1);

  math::Quaterniond q1Correct(sqrt(2)/2, 0, 0, sqrt(2)/2);
  math::Quaterniond q2Correct(sqrt(2)/2, 0, 0, -sqrt(2)/2);

  EXPECT_NE(q1, q2);
  EXPECT_EQ(q1Correct, q1);
  EXPECT_EQ(q2Correct, q2);
  EXPECT_EQ(math::Quaterniond::Identity, q1 * q2);
  EXPECT_EQ(v2, q1 * v1);
  EXPECT_EQ(v1, q2 * v2);

  // still the same rotation, but with non-unit vectors
  v1.Set(0.5, 0.5, 0);
  v2.Set(-0.5, 0.5, 0);

  q1.From2Axes(v1, v2);
  q2.From2Axes(v2, v1);

  EXPECT_NE(q1, q2);
  EXPECT_EQ(q1Correct, q1);
  EXPECT_EQ(q2Correct, q2);
  EXPECT_EQ(math::Quaterniond::Identity, q1 * q2);
  EXPECT_EQ(v2, q1 * v1);
  EXPECT_EQ(v1, q2 * v2);

  // Test various settings of opposite vectors (which need special care)

  v1.Set(1, 0, 0);
  v2.Set(-1, 0, 0);
  q1.From2Axes(v1, v2);
  q2 = q1 * q1;
  EXPECT_TRUE(math::equal(q2.W(), 1.0) || math::equal(q2.W(), -1.0));
  EXPECT_TRUE(math::equal(q2.X(), 0.0));
  EXPECT_TRUE(math::equal(q2.Y(), 0.0));
  EXPECT_TRUE(math::equal(q2.Z(), 0.0));

  v1.Set(0, 1, 0);
  v2.Set(0, -1, 0);
  q1.From2Axes(v1, v2);
  q2 = q1 * q1;
  EXPECT_TRUE(math::equal(q2.W(), 1.0) || math::equal(q2.W(), -1.0));
  EXPECT_TRUE(math::equal(q2.X(), 0.0));
  EXPECT_TRUE(math::equal(q2.Y(), 0.0));
  EXPECT_TRUE(math::equal(q2.Z(), 0.0));

  v1.Set(0, 0, 1);
  v2.Set(0, 0, -1);
  q1.From2Axes(v1, v2);
  q2 = q1 * q1;
  EXPECT_TRUE(math::equal(q2.W(), 1.0) || math::equal(q2.W(), -1.0));
  EXPECT_TRUE(math::equal(q2.X(), 0.0));
  EXPECT_TRUE(math::equal(q2.Y(), 0.0));
  EXPECT_TRUE(math::equal(q2.Z(), 0.0));

  v1.Set(0, 1, 1);
  v2.Set(0, -1, -1);
  q1.From2Axes(v1, v2);
  q2 = q1 * q1;
  EXPECT_TRUE(math::equal(q2.W(), 1.0) || math::equal(q2.W(), -1.0));
  EXPECT_TRUE(math::equal(q2.X(), 0.0));
  EXPECT_TRUE(math::equal(q2.Y(), 0.0));
  EXPECT_TRUE(math::equal(q2.Z(), 0.0));
}

