/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#define _USE_MATH_DEFINES
#include <cmath>

#include <gtest/gtest.h>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Quaternion.hh"
#include "ignition/math/Matrix3.hh"
#include "ignition/math/Matrix4.hh"

using namespace ignition;

TEST(QuaternionTest, Unit)
{
  math::Quaterniond q;
  EXPECT_TRUE(math::equal(q.w(), 1.0));
  EXPECT_TRUE(math::equal(q.x(), 0.0));
  EXPECT_TRUE(math::equal(q.y(), 0.0));
  EXPECT_TRUE(math::equal(q.z(), 0.0));
}

TEST(QuaternionTest, ConstructWithValues)
{
  math::Quaterniond q(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(q.w(), 1.0));
  EXPECT_TRUE(math::equal(q.x(), 2.0));
  EXPECT_TRUE(math::equal(q.y(), 3.0));
  EXPECT_TRUE(math::equal(q.z(), 4.0));
}

TEST(QuaternionTest, ConstructZero)
{
  math::Quaterniond q(0, 0, 0, 0);
  EXPECT_TRUE(math::equal(q.w(), 0.0));
  EXPECT_TRUE(math::equal(q.x(), 0.0));
  EXPECT_TRUE(math::equal(q.y(), 0.0));
  EXPECT_TRUE(math::equal(q.z(), 0.0));

  math::Quaterniond qI = q.GetInverse();
  EXPECT_TRUE(math::equal(qI.w(), 1.0));
  EXPECT_TRUE(math::equal(qI.x(), 0.0));
  EXPECT_TRUE(math::equal(qI.y(), 0.0));
  EXPECT_TRUE(math::equal(qI.z(), 0.0));
}

TEST(QuaternionTest, ConstructEuler)
{
  math::Quaterniond q(0, 1, 2);
  EXPECT_TRUE(q == math::Quaterniond(math::Vector3d(0, 1, 2)));
}

TEST(QuaternionTest, ConstructAxisAngle)
{
  math::Quaterniond q1(math::Vector3d(0, 0, 1), M_PI);
  EXPECT_TRUE(math::equal(q1.x(), 0.0));
  EXPECT_TRUE(math::equal(q1.y(), 0.0));
  EXPECT_TRUE(math::equal(q1.z(), 1.0));
  EXPECT_TRUE(math::equal(q1.w(), 0.0));

  math::Quaterniond q(q1);
  EXPECT_TRUE(q == q1);
}


TEST(QuaternionTest, Identity)
{
  math::Quaterniond q;
  q.SetToIdentity();
  EXPECT_TRUE(math::equal(q.w(), 1.0));
  EXPECT_TRUE(math::equal(q.x(), 0.0));
  EXPECT_TRUE(math::equal(q.y(), 0.0));
  EXPECT_TRUE(math::equal(q.z(), 0.0));
}

TEST(QuaternionTest, Math)
{
  math::Quaterniond q(M_PI*0.1, M_PI*0.5, M_PI);
  EXPECT_TRUE(q == math::Quaterniond(0.110616, -0.698401, 0.110616, 0.698401));

  EXPECT_TRUE(q.GetLog() ==
      math::Quaterniond(0, -1.02593, 0.162491, 1.02593));

  EXPECT_TRUE(q.GetExp() ==
      math::Quaterniond(0.545456, -0.588972, 0.093284, 0.588972));

  math::Quaterniond q1 = q;
  q1.w(2.0);
  EXPECT_TRUE(q1.GetLog() ==
      math::Quaterniond(0, -0.698401, 0.110616, 0.698401));

  q1.x(0.000000001);
  q1.y(0.0);
  q1.z(0.0);
  q1.w(0.0);
  EXPECT_TRUE(q1.GetExp() == math::Quaterniond(1, 0, 0, 0));

  q.Invert();
  EXPECT_TRUE(q == math::Quaterniond(0.110616, 0.698401, -0.110616, -0.698401));

  q.SetFromAxis(0, 1, 0, M_PI);
  EXPECT_TRUE(q == math::Quaterniond(6.12303e-17, 0, 1, 0));

  q.SetFromAxis(math::Vector3d(1, 0, 0), M_PI);
  EXPECT_TRUE(q == math::Quaterniond(0, 1, 0, 0));

  q.Set(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(q.w(), 1.0));
  EXPECT_TRUE(math::equal(q.x(), 2.0));
  EXPECT_TRUE(math::equal(q.y(), 3.0));
  EXPECT_TRUE(math::equal(q.z(), 4.0));

  q.Normalize();
  EXPECT_TRUE(q == math::Quaterniond(0.182574, 0.365148, 0.547723, 0.730297));


  EXPECT_TRUE(math::equal(q.GetRoll(), 1.4289, 1e-3));
  EXPECT_TRUE(math::equal(q.GetPitch(), -0.339837, 1e-3));
  EXPECT_TRUE(math::equal(q.GetYaw(), 2.35619, 1e-3));

  math::Vector3d axis;
  double angle;
  q.GetAsAxis(axis, angle);
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
  EXPECT_TRUE(math::equal(-1.18, q.x()));
  EXPECT_TRUE(math::equal(2.76, q.y()));
  EXPECT_TRUE(math::equal(4.01, q.z()));
  EXPECT_TRUE(math::equal(7.68, q.w()));

  q.x(0.0);
  q.y(0.0);
  q.z(0.0);
  q.w(0.0);
  q.Normalize();
  EXPECT_TRUE(q == math::Quaterniond());

  q.SetFromAxis(0, 0, 0, 0);
  EXPECT_TRUE(q == math::Quaterniond());

  EXPECT_TRUE(math::Quaterniond::EulerToQuaternion(0.1, 0.2, 0.3) ==
      math::Quaterniond(0.983347, 0.0342708, 0.106021, 0.143572));

  q.x(0.0);
  q.y(0.0);
  q.z(0.0);
  q.w(0.0);
  q.GetAsAxis(axis, angle);
  EXPECT_TRUE(axis == math::Vector3d(1, 0, 0));
  EXPECT_TRUE(math::equal(angle, 0.0, 1e-3));
  {
    // simple 180 rotation about yaw, should result in x and y flipping signs
    q = math::Quaterniond(0, 0, M_PI);
    math::Vector3d v = math::Vector3d(1, 2, 3);
    math::Vector3d r1 = q.RotateVector(v);
    math::Vector3d r2 = q.RotateVectorReverse(v);
    std::cout << "[" << q.w() << ", " << q.x() << ", "
      << q.y() << ", " << q.z() << "]\n";
    std::cout << " forward turns [" << v << "] to [" << r1 << "]\n";
    std::cout << " reverse turns [" << v << "] to [" << r2 << "]\n";
    EXPECT_TRUE(r1 == math::Vector3d(-1, -2, 3));
    EXPECT_TRUE(r2 == math::Vector3d(-1, -2, 3));
  }

  {
    // simple  90 rotation about yaw, should map x to y, y to -x
    // simple -90 rotation about yaw, should map x to -y, y to x
    q = math::Quaterniond(0, 0, 0.5*M_PI);
    math::Vector3d v = math::Vector3d(1, 2, 3);
    math::Vector3d r1 = q.RotateVector(v);
    math::Vector3d r2 = q.RotateVectorReverse(v);
    std::cout << "[" << q.w() << ", " << q.x() << ", "
      << q.y() << ", " << q.z() << "]\n";
    std::cout << " forward turns [" << v << "] to [" << r1 << "]\n";
    std::cout << " reverse turns [" << v << "] to [" << r2 << "]\n";
    std::cout << " x axis [" << q.GetXAxis() << "]\n";
    std::cout << " y axis [" << q.GetYAxis() << "]\n";
    std::cout << " z axis [" << q.GetZAxis() << "]\n";
    EXPECT_TRUE(r1 == math::Vector3d(-2, 1, 3));
    EXPECT_TRUE(r2 == math::Vector3d(2, -1, 3));
    EXPECT_TRUE(q.GetInverse().GetXAxis() == math::Vector3d(0, -1, 0));
    EXPECT_TRUE(q.GetInverse().GetYAxis() == math::Vector3d(1, 0, 0));
    EXPECT_TRUE(q.GetInverse().GetZAxis() == math::Vector3d(0, 0, 1));
  }

  {
    // now try a harder case (axis[1,2,3], rotation[0.3*pi])
    // verified with octave
    q.SetFromAxis(math::Vector3d(1, 2, 3), 0.3*M_PI);
    std::cout << "[" << q.w() << ", " << q.x() << ", "
      << q.y() << ", " << q.z() << "]\n";
    std::cout << " x [" << q.GetInverse().GetXAxis() << "]\n";
    std::cout << " y [" << q.GetInverse().GetYAxis() << "]\n";
    std::cout << " z [" << q.GetInverse().GetZAxis() << "]\n";
    EXPECT_TRUE(q.GetInverse().GetXAxis() ==
                math::Vector3d(0.617229, -0.589769, 0.520770));
    EXPECT_TRUE(q.GetInverse().GetYAxis() ==
                math::Vector3d(0.707544, 0.705561, -0.039555));
    EXPECT_TRUE(q.GetInverse().GetZAxis() ==
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

    EXPECT_TRUE(math::Matrix4d(q) == math::Matrix4d(
                0.617229, -0.589769, 0.52077, 0,
                0.707544, 0.705561, -0.0395554, 0,
                -0.344106, 0.392882, 0.85278, 0,
                0, 0, 0, 1));
  }
}
