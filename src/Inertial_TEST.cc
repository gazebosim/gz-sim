/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "ignition/math/Inertial.hh"

using namespace ignition;

/////////////////////////////////////////////////
/// \brief Compare quaternions, but allow rotations of PI about any axis.
void CompareModuloPi(const math::Quaterniond &_q1,
                     const math::Quaterniond &_q2,
                     const double _tol = 1e-6)
{
  const auto rotErrorEuler = (_q1.Inverse() * _q2).Euler();
  EXPECT_NEAR(sin(rotErrorEuler.X()), 0.0, _tol);
  EXPECT_NEAR(sin(rotErrorEuler.Y()), 0.0, _tol);
  EXPECT_NEAR(sin(rotErrorEuler.Z()), 0.0, _tol);
}

/////////////////////////////////////////////////
// Simple constructor, test default values
TEST(Inertiald_Test, Constructor)
{
  math::Inertiald inertial;
  EXPECT_EQ(inertial.Pose(), math::Pose3d::Zero);
  EXPECT_EQ(inertial.MassMatrix(), math::MassMatrix3d());
  EXPECT_EQ(inertial.MOI(), math::Matrix3d::Zero);
}

/////////////////////////////////////////////////
// Constructor with default arguments
// Should match simple constructor and with copy constructor
TEST(Inertiald_Test, ConstructorDefaultValues)
{
  math::Inertiald inertial(math::MassMatrix3d(), math::Pose3d::Zero);
  EXPECT_EQ(inertial, math::Inertiald());
  EXPECT_EQ(inertial, math::Inertiald(inertial));
}

/////////////////////////////////////////////////
// Constructor with non-default arguments
TEST(Inertiald_Test, ConstructorNonDefaultValues)
{
  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());
  const math::Pose3d pose(1, 2, 3, IGN_PI/6, 0, 0);
  math::Inertiald inertial(m, pose);

  // Should not match simple constructor
  EXPECT_NE(inertial, math::Inertiald());

  // Should match with copy constructor
  EXPECT_EQ(inertial, math::Inertiald(inertial));

  // Test accessors
  EXPECT_EQ(inertial.MassMatrix(), m);
  EXPECT_EQ(inertial.Pose(), pose);
  EXPECT_TRUE(inertial.MassMatrix().IsPositive());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());

  // Test assignment operator
  math::Inertiald inertial2;
  EXPECT_NE(inertial, inertial2);
  inertial2 = inertial;
  EXPECT_EQ(inertial, inertial2);
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, CoverageExtra)
{
  // getting full destructor coverage
  math::Inertiald *p = new math::Inertiald;
  EXPECT_TRUE(p != NULL);
  delete p;
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, Setters)
{
  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());
  const math::Pose3d pose(1, 2, 3, IGN_PI/6, 0, 0);
  math::Inertiald inertial;

  // Initially invalid
  EXPECT_FALSE(inertial.SetPose(pose));

  // Valid once valid mass matrix is set
  EXPECT_TRUE(inertial.SetMassMatrix(m));

  // Verify values
  EXPECT_EQ(inertial.MassMatrix(), m);
  EXPECT_EQ(inertial.Pose(), pose);

  // Invalid again if an invalid inertia is set
  math::MassMatrix3d mInvalid(-1, Ixxyyzz, Ixyxzyz);
  EXPECT_FALSE(inertial.SetMassMatrix(mInvalid));
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, MOI_Diagonal)
{
  const double mass = 12.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0, 0, 0);
  const math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  // no rotation, expect MOI's to match
  {
    const math::Pose3d pose(0, 0, 0, 0, 0, 0);
    math::Inertiald inertial(m, pose);
    EXPECT_EQ(inertial.MOI(), m.MOI());
  }

  // 90 deg rotation about X axis, expect different MOI
  {
    const math::Pose3d pose(0, 0, 0, IGN_PI_2, 0, 0);
    const math::Matrix3d expectedMOI(2, 0, 0, 0, 4, 0, 0, 0, 3);
    math::Inertiald inertial(m, pose);
    EXPECT_NE(inertial.MOI(), m.MOI());
    EXPECT_EQ(inertial.MOI(), expectedMOI);
  }

  // 90 deg rotation about Y axis, expect different MOI
  {
    const math::Pose3d pose(0, 0, 0, 0, IGN_PI_2, 0);
    const math::Matrix3d expectedMOI(4, 0, 0, 0, 3, 0, 0, 0, 2);
    math::Inertiald inertial(m, pose);
    EXPECT_NE(inertial.MOI(), m.MOI());
    EXPECT_EQ(inertial.MOI(), expectedMOI);
  }

  // 90 deg rotation about Z axis, expect different MOI
  {
    const math::Pose3d pose(0, 0, 0, 0, 0, IGN_PI_2);
    const math::Matrix3d expectedMOI(3, 0, 0, 0, 2, 0, 0, 0, 4);
    math::Inertiald inertial(m, pose);
    EXPECT_NE(inertial.MOI(), m.MOI());
    EXPECT_EQ(inertial.MOI(), expectedMOI);
  }

  // 45 deg rotation about Z axis, expect different MOI
  {
    const math::Pose3d pose(0, 0, 0, 0, 0, IGN_PI_4);
    const math::Matrix3d expectedMOI(2.5, -0.5, 0, -0.5, 2.5, 0, 0, 0, 4);
    math::Inertiald inertial(m, pose);
    EXPECT_NE(inertial.MOI(), m.MOI());
    EXPECT_EQ(inertial.MOI(), expectedMOI);

    // double check with a second MassMatrix3 instance
    // that has the same base frame MOI but no pose rotation
    math::MassMatrix3d m2;
    EXPECT_FALSE(m2.Mass(mass));
    EXPECT_TRUE(m2.MOI(expectedMOI));
    EXPECT_EQ(inertial.MOI(), m2.MOI());
    // There are multiple correct rotations due to symmetry
    CompareModuloPi(m2.PrincipalAxesOffset(), pose.Rot());
  }
}

/////////////////////////////////////////////////
// Base frame MOI should be invariant
void SetRotation(const double _mass,
    const math::Vector3d &_ixxyyzz,
    const math::Vector3d &_ixyxzyz,
    const bool _unique = true)
{
  const math::MassMatrix3d m(_mass, _ixxyyzz, _ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  math::Pose3d pose(math::Vector3d::Zero, math::Quaterniond::Identity);
  const math::Inertiald inertialRef(m, pose);
  const auto moi = inertialRef.MOI();

  const std::vector<math::Quaterniond> rotations = {
    math::Quaterniond::Identity,
    math::Quaterniond(IGN_PI, 0, 0),
    math::Quaterniond(0, IGN_PI, 0),
    math::Quaterniond(0, 0, IGN_PI),
    math::Quaterniond(IGN_PI_2, 0, 0),
    math::Quaterniond(0, IGN_PI_2, 0),
    math::Quaterniond(0, 0, IGN_PI_2),
    math::Quaterniond(IGN_PI_4, 0, 0),
    math::Quaterniond(0, IGN_PI_4, 0),
    math::Quaterniond(0, 0, IGN_PI_4),
    math::Quaterniond(IGN_PI/6, 0, 0),
    math::Quaterniond(0, IGN_PI/6, 0),
    math::Quaterniond(0, 0, IGN_PI/6),
    math::Quaterniond(0.1, 0.2, 0.3),
    math::Quaterniond(-0.1, 0.2, -0.3),
    math::Quaterniond(0.4, 0.2, 0.5),
    math::Quaterniond(-0.1, 0.7, -0.7)};
  for (const auto rot : rotations)
  {
    {
      auto inertial = inertialRef;

      const double tol  = -1e-6;
      EXPECT_TRUE(inertial.SetMassMatrixRotation(rot, tol));
      EXPECT_EQ(moi, inertial.MOI());
      if (_unique)
      {
        CompareModuloPi(rot, inertial.MassMatrix().PrincipalAxesOffset(tol));
      }

      EXPECT_TRUE(inertial.SetInertialRotation(rot));
      EXPECT_EQ(rot, inertial.Pose().Rot());
      EXPECT_EQ(moi, inertial.MOI());
    }

    {
      auto inertial = inertialRef;

      EXPECT_TRUE(inertial.SetInertialRotation(rot));
      EXPECT_EQ(rot, inertial.Pose().Rot());
      EXPECT_EQ(moi, inertial.MOI());

      const double tol = -1e-6;
      EXPECT_TRUE(inertial.SetMassMatrixRotation(rot, tol));
      EXPECT_EQ(moi, inertial.MOI());
      if (_unique)
      {
        CompareModuloPi(rot, inertial.MassMatrix().PrincipalAxesOffset(tol));
      }
    }
  }
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetRotationUniqueDiagonal)
{
  SetRotation(12, math::Vector3d(2, 3, 4), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(3, 2, 4), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(2, 4, 3), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(3, 4, 2), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(4, 2, 3), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(4, 3, 2), math::Vector3d::Zero);
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetRotationUniqueNondiagonal)
{
  SetRotation(12, math::Vector3d(2, 3, 4), math::Vector3d(0.3, 0.2, 0.1));
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetRotationNonuniqueDiagonal)
{
  SetRotation(12, math::Vector3d(2, 2, 2), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(2, 2, 3), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(2, 3, 2), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(3, 2, 2), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(2, 3, 3), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(3, 2, 3), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(3, 3, 2), math::Vector3d::Zero, false);
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetRotationNonuniqueNondiagonal)
{
  SetRotation(12, math::Vector3d(4, 4, 3), math::Vector3d(-1, 0, 0), false);
  SetRotation(12, math::Vector3d(4, 3, 4), math::Vector3d(0, -1, 0), false);
  SetRotation(12, math::Vector3d(3, 4, 4), math::Vector3d(0, 0, -1), false);
  SetRotation(12, math::Vector3d(4, 4, 5), math::Vector3d(-1, 0, 0), false);
  SetRotation(12, math::Vector3d(5, 4, 4), math::Vector3d(0, 0, -1), false);
  SetRotation(12, math::Vector3d(5.5, 4.125, 4.375),
             0.25*math::Vector3d(-sqrt(3), 3.0, -sqrt(3)/2), false);
  SetRotation(12, math::Vector3d(4.125, 5.5, 4.375),
                      0.25*math::Vector3d(-sqrt(3), -sqrt(3)/2, 3.0), false);
}

/////////////////////////////////////////////////
// test for diagonalizing MassMatrix
// verify MOI is conserved
// and that off-diagonal terms are zero
void Diagonalize(
    const double _mass,
    const math::Vector3d &_ixxyyzz,
    const math::Vector3d &_ixyxzyz)
{
  const math::MassMatrix3d m(_mass, _ixxyyzz, _ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  math::Pose3d pose(math::Vector3d::Zero, math::Quaterniond::Identity);
  math::Inertiald inertial(m, pose);
  const auto moi = inertial.MOI();

  EXPECT_TRUE(inertial.SetMassMatrixRotation(math::Quaterniond::Identity));
  EXPECT_EQ(moi, inertial.MOI());
  EXPECT_EQ(inertial.MassMatrix().OffDiagonalMoments(), math::Vector3d::Zero);

  // try again with negative tolerance
  EXPECT_TRUE(
    inertial.SetMassMatrixRotation(math::Quaterniond::Identity, -1e-6));
  EXPECT_EQ(moi, inertial.MOI());
  EXPECT_EQ(inertial.MassMatrix().OffDiagonalMoments(), math::Vector3d::Zero);
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, Diagonalize)
{
  Diagonalize(12, math::Vector3d(2, 3, 4), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 2, 4), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 4, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 4, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(4, 2, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(4, 3, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 3, 4), math::Vector3d(0.3, 0.2, 0.1));
  Diagonalize(12, math::Vector3d(2, 2, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 2, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 3, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 2, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 3, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 2, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 3, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(4, 4, 3), math::Vector3d(-1, 0, 0));
  Diagonalize(12, math::Vector3d(4, 3, 4), math::Vector3d(0, -1, 0));
  Diagonalize(12, math::Vector3d(3, 4, 4), math::Vector3d(0, 0, -1));
  Diagonalize(12, math::Vector3d(4, 4, 5), math::Vector3d(-1, 0, 0));
  Diagonalize(12, math::Vector3d(5, 4, 4), math::Vector3d(0, 0, -1));
  Diagonalize(12, math::Vector3d(5.5, 4.125, 4.375),
                               0.25*math::Vector3d(-sqrt(3), 3.0, -sqrt(3)/2));
  Diagonalize(12, math::Vector3d(4.125, 5.5, 4.375),
                      0.25*math::Vector3d(-sqrt(3), -sqrt(3)/2, 3.0));
}
/////////////////////////////////////////////////
TEST(Inertiald_Test, Addition)
{
  // Add two half-cubes together
  {
    const double mass = 12.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    const math::Inertiald cube(cubeMM3, math::Pose3d::Zero);
    math::MassMatrix3d half;
    EXPECT_TRUE(half.SetFromBox(0.5*mass, math::Vector3d(0.5, 1, 1)));
    math::Inertiald left(half, math::Pose3d(-0.25, 0, 0, 0, 0, 0));
    math::Inertiald right(half, math::Pose3d(0.25, 0, 0, 0, 0, 0));
    EXPECT_EQ(cube, left + right);
    EXPECT_EQ(cube, right + left);
    // test += operator
    {
      math::Inertiald tmp = left;
      tmp += right;
      EXPECT_EQ(cube, tmp);
    }
    {
      math::Inertiald tmp = right;
      tmp += left;
      EXPECT_EQ(cube, tmp);
    }
    // Test EquivalentBox
    {
      math::Vector3d size2;
      math::Quaterniond rot2;
      EXPECT_TRUE((left + right).MassMatrix().EquivalentBox(size2, rot2));
      EXPECT_EQ(size, size2);
      EXPECT_EQ(rot2, math::Quaterniond::Identity);
    }
    {
      math::Vector3d size2;
      math::Quaterniond rot2;
      EXPECT_TRUE((right + left).MassMatrix().EquivalentBox(size2, rot2));
      EXPECT_EQ(size, size2);
      EXPECT_EQ(rot2, math::Quaterniond::Identity);
    }
  }

  // Add two rotated half-cubes together
  {
    const double mass = 12.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    const math::Inertiald cube(cubeMM3, math::Pose3d(0, 0, 0, IGN_PI_4, 0, 0));

    math::MassMatrix3d half;
    EXPECT_TRUE(half.SetFromBox(0.5*mass, math::Vector3d(0.5, 1, 1)));
    math::Inertiald left(half, math::Pose3d(-0.25, 0, 0, IGN_PI_4, 0, 0));
    math::Inertiald right(half, math::Pose3d(0.25, 0, 0, IGN_PI_4, 0, 0));

    // objects won't match exactly
    // since inertia matrices will all be in base frame
    // but mass, center of mass, and base-frame MOI should match
    EXPECT_NE(cube, left + right);
    EXPECT_NE(cube, right + left);
    EXPECT_DOUBLE_EQ(cubeMM3.Mass(), (left + right).MassMatrix().Mass());
    EXPECT_DOUBLE_EQ(cubeMM3.Mass(), (right + left).MassMatrix().Mass());
    EXPECT_EQ(cube.Pose().Pos(), (left + right).Pose().Pos());
    EXPECT_EQ(cube.Pose().Pos(), (right + left).Pose().Pos());
    EXPECT_EQ(cube.MOI(), (left + right).MOI());
    EXPECT_EQ(cube.MOI(), (right + left).MOI());
  }

  // Add eight cubes together into larger cube
  {
    const double mass = 12.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    const math::Inertiald addedCube =
      math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5,  0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,  -0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,   0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, 0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5,  0.5, 0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,  -0.5, 0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,   0.5, 0.5, 0, 0, 0));

    math::MassMatrix3d trueCubeMM3;
    EXPECT_TRUE(trueCubeMM3.SetFromBox(8*mass, 2*size));
    EXPECT_EQ(addedCube, math::Inertiald(trueCubeMM3, math::Pose3d::Zero));
  }

  // Add eight rotated cubes together into larger cube
  {
    const double mass = 12.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    const math::Inertiald addedCube =
      math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5,  0.5, -0.5, IGN_PI_2, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,  -0.5, -0.5, 0, IGN_PI_2, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,   0.5, -0.5, 0, 0, IGN_PI_2)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, 0.5, IGN_PI, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5,  0.5, 0.5, 0, IGN_PI, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,  -0.5, 0.5, 0, 0, IGN_PI)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,   0.5, 0.5, 0, 0, 0));

    math::MassMatrix3d trueCubeMM3;
    EXPECT_TRUE(trueCubeMM3.SetFromBox(8*mass, 2*size));
    EXPECT_EQ(addedCube, math::Inertiald(trueCubeMM3, math::Pose3d::Zero));
  }
}

/////////////////////////////////////////////////
// Addition operator has different behavior if mass is non-positive
TEST(Inertiald_Test, AdditionInvalid)
{
  // inertias all zero
  const math::MassMatrix3d m0(0.0, math::Vector3d::Zero, math::Vector3d::Zero);
  EXPECT_FALSE(m0.IsPositive());
  EXPECT_FALSE(m0.IsValid());

  // both inertials with zero mass
  {
    math::Inertiald left(m0, math::Pose3d(-1, 0, 0, 0, 0, 0));
    math::Inertiald right(m0, math::Pose3d(1, 0, 0, 0, 0, 0));

    // expect sum to equal left argument
    EXPECT_EQ(left, left + right);
    EXPECT_EQ(right, right + left);
    {
      math::Inertiald tmp = left;
      tmp += right;
      EXPECT_EQ(tmp, left);
    }
    {
      math::Inertiald tmp = right;
      tmp += left;
      EXPECT_EQ(tmp, right);
    }
  }

  // one inertial with zero inertias should not affect the sum
  {
    math::MassMatrix3d m(12.0,
      math::Vector3d(2, 3, 4),
      math::Vector3d(0.1, 0.2, 0.3));
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    math::Inertiald i(m, math::Pose3d(-1, 0, 0, 0, 0, 0));
    math::Inertiald i0(m0, math::Pose3d(1, 0, 0, 0, 0, 0));

    // expect i0 to not affect the sum
    EXPECT_EQ(i, i + i0);
    EXPECT_EQ(i, i0 + i);
    {
      math::Inertiald tmp = i;
      tmp += i0;
      EXPECT_EQ(tmp, i);
    }
    {
      math::Inertiald tmp = i0;
      tmp += i;
      EXPECT_EQ(tmp, i);
    }

    EXPECT_TRUE((i + i0).MassMatrix().IsPositive());
    EXPECT_TRUE((i0 + i).MassMatrix().IsPositive());
    EXPECT_TRUE((i + i0).MassMatrix().IsValid());
    EXPECT_TRUE((i0 + i).MassMatrix().IsValid());
  }
}
