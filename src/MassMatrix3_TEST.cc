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

#include "ignition/math/Helpers.hh"
#include "ignition/math/MassMatrix3.hh"
#include "ignition/math/Material.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, Constructors)
{
  // Simple constructor, test default values
  {
    math::MassMatrix3d m;
    EXPECT_DOUBLE_EQ(m.Mass(), 0.0);
    EXPECT_DOUBLE_EQ(m.Ixx(), 0.0);
    EXPECT_DOUBLE_EQ(m.Iyy(), 0.0);
    EXPECT_DOUBLE_EQ(m.Izz(), 0.0);
    EXPECT_DOUBLE_EQ(m.Ixy(), 0.0);
    EXPECT_DOUBLE_EQ(m.Ixz(), 0.0);
    EXPECT_DOUBLE_EQ(m.Iyz(), 0.0);
    EXPECT_EQ(m.DiagonalMoments(), math::Vector3d::Zero);
    EXPECT_EQ(m.OffDiagonalMoments(), math::Vector3d::Zero);
    EXPECT_EQ(m.Moi(), math::Matrix3d::Zero);
    EXPECT_FALSE(m.IsPositive());
    EXPECT_TRUE(m.IsNearPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Constructor with default arguments
  // Should match simple constructor and with copy constructor
  {
    math::MassMatrix3d m(0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_EQ(m, math::MassMatrix3d());
    EXPECT_EQ(m, math::MassMatrix3d(m));
    EXPECT_FALSE(m.IsPositive());
    EXPECT_TRUE(m.IsNearPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Constructor with non-default arguments
  {
    const double mass = 5.0;
    const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
    const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
    const math::Matrix3d moi(2.0, 0.2, 0.3,
                             0.2, 3.0, 0.4,
                             0.3, 0.4, 4.0);
    math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);

    // Should not match simple constructor
    EXPECT_NE(m, math::MassMatrix3d());

    // Should match with copy constructor
    EXPECT_EQ(m, math::MassMatrix3d(m));

    // Test accessors
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.Ixx(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.Iyy(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.Izz(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.Ixy(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.Ixz(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.Iyz(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.Moi(), moi);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsNearPositive());
    EXPECT_TRUE(m.IsValid());

    // Test assignment operator
    math::MassMatrix3d m2;
    EXPECT_NE(m, m2);
    m2 = m;
    EXPECT_EQ(m, m2);
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, Setters)
{
  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  const math::Matrix3d moi(2.0, 0.2, 0.3,
                           0.2, 3.0, 0.4,
                           0.3, 0.4, 4.0);

  // Scalar setters with simple constructor
  // MassMatrix3 won't be valid until enough properties are set
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_TRUE(m.IsNearPositive());
    EXPECT_TRUE(m.IsValid());

    // Valid when mass is set
    EXPECT_TRUE(m.SetMass(mass));
    EXPECT_FALSE(m.SetIxx(Ixxyyzz[0]));
    EXPECT_FALSE(m.SetIyy(Ixxyyzz[1]));

    // Valid once enough properties are set
    EXPECT_TRUE(m.SetIzz(Ixxyyzz[2]));
    EXPECT_TRUE(m.SetIxy(Ixyxzyz[0]));
    EXPECT_TRUE(m.SetIxz(Ixyxzyz[1]));
    EXPECT_TRUE(m.SetIyz(Ixyxzyz[2]));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.Ixx(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.Iyy(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.Izz(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.Ixy(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.Ixz(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.Iyz(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.Moi(), moi);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid again if an invalid inertia is set
    EXPECT_FALSE(m.SetMass(-1));
  }

  // Test vector setters for moment of inertia
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_TRUE(m.IsNearPositive());
    EXPECT_TRUE(m.IsValid());

    // Valid when mass is set
    EXPECT_TRUE(m.SetMass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.SetDiagonalMoments(Ixxyyzz));
    EXPECT_TRUE(m.SetOffDiagonalMoments(Ixyxzyz));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.Ixx(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.Iyy(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.Izz(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.Ixy(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.Ixz(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.Iyz(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.Moi(), moi);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid if an invalid inertia is set
    EXPECT_FALSE(m.SetIxx(-1));
  }

  // Test Matrix3 setter for moment of inertia
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_TRUE(m.IsNearPositive());
    EXPECT_TRUE(m.IsValid());

    // Valid when mass is set
    EXPECT_TRUE(m.SetMass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.SetMoi(moi));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.Ixx(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.Iyy(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.Izz(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.Ixy(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.Ixz(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.Iyz(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.Moi(), moi);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid if an excessive off-diagonal inertia is set
    EXPECT_FALSE(m.SetIxy(1e3));
  }

  // // Test atomic InertiaMatrix setter
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_TRUE(m.IsNearPositive());
    EXPECT_TRUE(m.IsValid());

    // Initially invalid
    EXPECT_TRUE(m.SetMass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.SetInertiaMatrix(2, 3, 4, 0.2, 0.3, 0.4));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.Ixx(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.Iyy(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.Izz(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.Ixy(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.Ixz(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.Iyz(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.Moi(), moi);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, CoverageExtra)
{
  // getting full destructor coverage
  math::MassMatrix3d *p = new math::MassMatrix3d;
  EXPECT_TRUE(p != NULL);
  delete p;
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalMoments)
{
  // Diagonal inertia moments (1, 1, 1)
  {
    math::MassMatrix3d m(1.0, math::Vector3d::One, math::Vector3d::Zero);
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d::One);

    // Minor perturbations of product moments
    // shouldn't affect PrincipalMoments, given the tolerance
    // of the Vector3 equality operator
    EXPECT_TRUE(m.SetIxy(1e-10));
    EXPECT_TRUE(m.SetIxz(2e-10));
    EXPECT_TRUE(m.SetIyz(3e-10));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d::One);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Non-equal eigen-moments
  {
    const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
    math::MassMatrix3d m(1.0, Ixxyyzz, math::Vector3d::Zero);
    EXPECT_TRUE(m.SetDiagonalMoments(Ixxyyzz));
    EXPECT_EQ(m.PrincipalMoments(), Ixxyyzz);

    // Minor perturbation of product moments
    EXPECT_TRUE(m.SetIxy(1e-10));
    EXPECT_TRUE(m.SetIxz(2e-10));
    EXPECT_TRUE(m.SetIyz(3e-10));
    EXPECT_EQ(m.PrincipalMoments(), Ixxyyzz);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Non-trivial off-diagonal product moments
  // Symmetric positive definite matrix from
  // Strang's Intro to Linear Algebra textbook
  // This isn't actually a valid inertia matrix though,
  // since it doesn't satisfy the triangle inequality
  // 2-sqrt(2) + 2 ~= 2.59
  // 2+sqrt(2) ~= 3.41
  {
    const math::Vector3d Ixxyyzz(2.0, 2.0, 2.0);
    const math::Vector3d Ixyxzyz(-1.0, 0, -1.0);
    math::MassMatrix3d m(1.0, Ixxyyzz, Ixyxzyz);
    const math::Vector3d Ieigen(2-IGN_SQRT2, 2, 2+IGN_SQRT2);
    EXPECT_EQ(m.PrincipalMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }

  // Non-trivial off-diagonal product moments
  // variant of previous example that is valid inertia matrix
  {
    const math::Vector3d Ixxyyzz(4.0, 4.0, 4.0);
    const math::Vector3d Ixyxzyz(-1.0, 0, -1.0);
    math::MassMatrix3d m(1.0, Ixxyyzz, Ixyxzyz);
    const math::Vector3d Ieigen(4-IGN_SQRT2, 4, 4+IGN_SQRT2);
    EXPECT_EQ(m.PrincipalMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Degenerate matrix with eigenvalue of 0
  // not positive definite
  {
    const math::Vector3d Ixxyyzz(1.0, 1.0, 1.0);
    const math::Vector3d Ixyxzyz(1.0, 0, 0);
    math::MassMatrix3d m(1.0, Ixxyyzz, Ixyxzyz);
    const math::Vector3d Ieigen(0, 1, 2);
    EXPECT_EQ(m.PrincipalMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }

  // Matrix with large condition number
  // barely positive definite
  // invalid inertia matrix since it doesn't satisfy triangle inequality
  // 5e-6 + 1.0 < 2+5e-6
  {
    const math::Vector3d Ixxyyzz(1.0, 1.00001, 1.0);
    const math::Vector3d Ixyxzyz(1.0, 0, 0);
    math::MassMatrix3d m(1.0, Ixxyyzz, Ixyxzyz);
    const math::Vector3d Ieigen(5e-6, 1.0, 2 + 5e-6);
    EXPECT_EQ(m.PrincipalMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }

  // Another matrix with large condition number
  // invalid inertia matrix since it doesn't satisfy triangle inequality
  // 0.98 + 1e8-1e3 < 1e8+1e3
  // 0.98 < 2e3
  {
    const math::Vector3d Ixxyyzz(1e8, 1e8, 1);
    const math::Vector3d Ixyxzyz(1e3, 1e3, 1e3);
    math::MassMatrix3d m(1.0, Ixxyyzz, Ixyxzyz);
    const math::Vector3d Ieigen(0.98, 1e8-1e3, 1e8+1e3);
    // the accuracy is approximately 2e-2
    EXPECT_TRUE(m.PrincipalMoments().Equal(Ieigen, 2.5e-2));
    EXPECT_FALSE(m.PrincipalMoments().Equal(Ieigen, 1.5e-2));
    // the default tolerance for == is 1e-6
    // so this should resolve as not equal
    EXPECT_NE(m.PrincipalMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalAxesOffsetIdentity)
{
  // Identity inertia matrix, expect unit quaternion
  math::MassMatrix3d m(1.0, math::Vector3d::One, math::Vector3d::Zero);
  EXPECT_EQ(m.PrincipalAxesOffset(), math::Quaterniond());

  // Scale the diagonal terms
  EXPECT_TRUE(m.SetDiagonalMoments(3.5 * math::Vector3d::One));
  EXPECT_TRUE(m.SetOffDiagonalMoments(math::Vector3d::Zero));
  EXPECT_TRUE(m.IsValid());
  EXPECT_EQ(m.PrincipalAxesOffset(), math::Quaterniond::Identity);
}

/////////////////////////////////////////////////
/// \brief Helper function for verifying principal moments
/// and axes offset by reconstructing the moment of inertia matrix
/// from the eigenvectors and diagonalized matrix.
/// \param[in] _m mass matrix to verify
/// \param[in] _tolerance relative tolerance to use
void VerifyPrincipalMomentsAndAxes(const math::MassMatrix3d &_m,
                                   const double _tolerance = 1e-6)
{
  auto q = _m.PrincipalAxesOffset(_tolerance);
  auto R = math::Matrix3d(q);
  EXPECT_FALSE(math::equal(q.W(), 0.0, 1e-6) && math::equal(q.X(), 0.0, 1e-6) &&
               math::equal(q.Y(), 0.0, 1e-6) && math::equal(q.Z(), 0.0, 1e-6));
  auto moments = _m.PrincipalMoments(_tolerance);
  math::Matrix3d L(moments[0], 0, 0,
                   0, moments[1], 0,
                   0, 0, moments[2]);
  EXPECT_EQ(_m.Moi(), R * L * R.Transposed());
}

/////////////////////////////////////////////////
/// \brief Helper function for testing diagonal inertia matrices.
/// Expect the following:
/// * that principal moments match the diagonal values,
/// * that mass matrix is valid,
/// * that principal axes have no offset (identity quaternion)
/// * that reconstructed moment of inertia matrix matches the original
/// \param[in] _moments Diagonal/principal moments of inertia.
void VerifyDiagonalMomentsAndAxes(const math::Vector3d &_moments)
{
  math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
  EXPECT_TRUE(m.SetDiagonalMoments(_moments));
  EXPECT_EQ(m.PrincipalMoments(), m.DiagonalMoments());
  EXPECT_TRUE(m.IsValid());
  // Expect unit quaternion
  EXPECT_EQ(m.PrincipalAxesOffset(), math::Quaterniond::Identity);
  VerifyPrincipalMomentsAndAxes(m);

  // Try with negative tolerance, expect sorted principal moments
  math::Vector3d sortedMoments;
  {
    double m0 = _moments[0];
    double m1 = _moments[1];
    double m2 = _moments[2];
    math::sort3(m0, m1, m2);
    sortedMoments.Set(m0, m1, m2);
  }
  const double tolerance = -1e-6;
  EXPECT_EQ(m.PrincipalMoments(tolerance), sortedMoments);
  VerifyPrincipalMomentsAndAxes(m, tolerance);
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalAxesOffsetDiagonal)
{
  // all repeated moments [3, 3, 3]
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 3.0, 3.0));
  // repeated moments [2, 3, 3]
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 3.0, 3.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 2.0, 3.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 3.0, 2.0));
  // repeated moments [2, 2, 3]
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 2.0, 2.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 3.0, 2.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 2.0, 3.0));
  // non-repeated moments
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 3.0, 4.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(4.0, 2.0, 3.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 4.0, 2.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 4.0, 3.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 2.0, 4.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(4.0, 3.0, 2.0));
}

/////////////////////////////////////////////////
/// \brief Helper function for testing non-diagonal inertia matrices.
/// Expect the following:
/// * that principal moments match the supplied values,
/// * that mass matrix is valid,
/// * that principal axes have an offset (non-identity quaternion)
/// * that reconstructed moment of inertia matrix matches the original
/// \param[in] _principalMoments Expected principal moments of inertia
/// \param[in] _ixxyyzz Diagonal moments of inertia.
/// \param[in] _ixyxzyz Off-diagonal moments of inertia.
/// \param[in] _tolerance Absolute tolerance for eigenvalue expectation.
void VerifyNondiagonalMomentsAndAxes(const math::Vector3d &_principalMoments,
                                     const math::Vector3d &_ixxyyzz,
                                     const math::Vector3d &_ixyxzyz,
                                     const double _tolerance = 1e-6)
{
  math::MassMatrix3d m(1.0, _ixxyyzz, _ixyxzyz);
  // EXPECT_EQ with default tolerance of 1e-6
  // this outputs more useful error messages
  EXPECT_EQ(m.PrincipalMoments(_tolerance), _principalMoments);
  // also check equality with custom tolerance for small moments
  EXPECT_TRUE(
    m.PrincipalMoments(_tolerance).Equal(_principalMoments, _tolerance));
  EXPECT_TRUE(m.IsValid());
  // Expect non-unit quaternion
  EXPECT_NE(m.PrincipalAxesOffset(_tolerance), math::Quaterniond());
  VerifyPrincipalMomentsAndAxes(m, _tolerance);

  // Try also with negated tolerance
  EXPECT_TRUE(
    m.PrincipalMoments(-_tolerance).Equal(_principalMoments, _tolerance));
  VerifyPrincipalMomentsAndAxes(m, -_tolerance);
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalAxesOffsetRepeat)
{
  // Principal moments: [3, 3, 5]
  // Non-zero Ixy
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 3, 5),
    math::Vector3d(4, 4, 3), math::Vector3d(-1, 0, 0));
  // Non-zero Ixz
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 3, 5),
    math::Vector3d(4, 3, 4), math::Vector3d(0, -1, 0));
  // Non-zero Iyz
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 3, 5),
    math::Vector3d(3, 4, 4), math::Vector3d(0, 0, -1));

  // Principal moments: [3, 5, 5]
  // Non-zero Ixy
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 5, 5),
    math::Vector3d(4, 4, 5), math::Vector3d(-1, 0, 0));
  // Non-zero Ixz
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 5, 5),
    math::Vector3d(4, 5, 4), math::Vector3d(0, -1, 0));
  // Non-zero Iyz
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 5, 5),
    math::Vector3d(5, 4, 4), math::Vector3d(0, 0, -1));

  // Principal moments: [4, 5, 5]
  // Rotated by [45, 45, 0] degrees
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 5, 5),
    math::Vector3d(4.5, 4.75, 4.75),
    0.25*math::Vector3d(-IGN_SQRT2, IGN_SQRT2, 1));
  // Rotated by [-45, 45, 0] degrees
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 5, 5),
    math::Vector3d(4.5, 4.75, 4.75),
    0.25*math::Vector3d(IGN_SQRT2, IGN_SQRT2, -1));
  // Rotated by [45, -45, 0] degrees
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 5, 5),
    math::Vector3d(4.5, 4.75, 4.75),
    0.25*math::Vector3d(IGN_SQRT2, -IGN_SQRT2, 1));
  // Rotated by [-45, -45, 0] degrees
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 5, 5),
    math::Vector3d(4.5, 4.75, 4.75),
    0.25*math::Vector3d(-IGN_SQRT2, -IGN_SQRT2, -1));

  // Principal moments: [4, 4, 5]
  // Rotated by [45, 45, 45] degrees
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 4, 5),
    math::Vector3d(4.5, 4.25, 4.25),
    0.25*math::Vector3d(-IGN_SQRT2, IGN_SQRT2, -1));
  // different rotation
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 4, 5),
    math::Vector3d(4.5, 4.25, 4.25),
    0.25*math::Vector3d(IGN_SQRT2, IGN_SQRT2, 1));
  // different rotation
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 4, 5),
    math::Vector3d(4.5, 4.25, 4.25),
    0.25*math::Vector3d(-IGN_SQRT2, -IGN_SQRT2, 1));
  // different rotation
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 4, 5),
    math::Vector3d(4.5, 4.25, 4.25),
    0.25*math::Vector3d(IGN_SQRT2, -IGN_SQRT2, -1));

  // Principal moments [4e-9, 4e-9, 5e-9]
  // Rotated by [45, 45, 45] degrees
  // use tolerance of 1e-15
  VerifyNondiagonalMomentsAndAxes(1e-9 * math::Vector3d(4, 4, 5),
    1e-9 * math::Vector3d(4.5, 4.25, 4.25),
    0.25e-9*math::Vector3d(-IGN_SQRT2, IGN_SQRT2, -1), 1e-15);
  // different rotation
  VerifyNondiagonalMomentsAndAxes(1e-9 * math::Vector3d(4, 4, 5),
    1e-9 * math::Vector3d(4.5, 4.25, 4.25),
    0.25e-9*math::Vector3d(IGN_SQRT2, IGN_SQRT2, 1));
  // different rotation
  VerifyNondiagonalMomentsAndAxes(1e-9 * math::Vector3d(4, 4, 5),
    1e-9 * math::Vector3d(4.5, 4.25, 4.25),
    0.25e-9*math::Vector3d(-IGN_SQRT2, -IGN_SQRT2, 1));
  // different rotation
  VerifyNondiagonalMomentsAndAxes(1e-9 * math::Vector3d(4, 4, 5),
    1e-9 * math::Vector3d(4.5, 4.25, 4.25),
    0.25e-9*math::Vector3d(IGN_SQRT2, -IGN_SQRT2, -1), 1e-15);

  // Principal moments [4, 4, 6]
  // rotate by 30, 60, 0 degrees
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 4, 6),
    math::Vector3d(5.5, 4.125, 4.375),
    0.25*math::Vector3d(-sqrt(3), 3.0, -sqrt(3)/2));

  // different rotation
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4, 4, 6),
    math::Vector3d(4.125, 5.5, 4.375),
    0.25*math::Vector3d(-sqrt(3), -sqrt(3)/2, 3.0));
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalAxesOffsetNoRepeat)
{
  // Non-diagonal inertia matrix with f1 = 0
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(3.0, 5.0, 5.0),
    math::Vector3d(0, 0, 1));
  // Non-diagonal inertia matrix with f1 = 0
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(3.0, 5.0, 5.0),
    math::Vector3d(0, 0, -1));

  // Non-diagonal inertia matrix with f2 = 0
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(5.0, 4.0, 4.0),
    math::Vector3d(-1, 1, 0));
  // Non-diagonal inertia matrix with f2 = 0
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(5.0, 4.0, 4.0),
    math::Vector3d(1, -1, 0));
  // Non-diagonal inertia matrix with f2 = 0
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(5.0, 4.0, 4.0),
    math::Vector3d(-1, -1, 0));
  // Non-diagonal inertia matrix with f2 = 0
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(5.0, 4.0, 4.0),
    math::Vector3d(1, 1, 0));

  // Similar non-diagonal inertia matrix with f2 != 0
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(4.0, 4.0, 5.0),
    math::Vector3d(0, 1, 1));
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(4.0, 4.0, 5.0),
    math::Vector3d(0, -1, 1));
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(4.0, 4.0, 5.0),
    math::Vector3d(0, 1, -1));
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 4, 6),
    math::Vector3d(4.0, 4.0, 5.0),
    math::Vector3d(0, -1, -1));

  // Test case for v = 0
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(2.5, 3.5, 4.0),
    math::Vector3d(4.0, 3.0, 3.0),
    math::Vector3d(0.0, 0, -0.5));

  // Tri-diagonal matrix with identical diagonal terms
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(4-IGN_SQRT2, 4, 4+IGN_SQRT2),
    math::Vector3d(4.0, 4.0, 4.0),
    math::Vector3d(-1.0, 0, -1.0));
  // small magnitude, use tolerance of 1e-15
  VerifyNondiagonalMomentsAndAxes(
    1e-9 * math::Vector3d(4-IGN_SQRT2, 4, 4+IGN_SQRT2),
    1e-9 * math::Vector3d(4.0, 4.0, 4.0),
    1e-9 * math::Vector3d(-1.0, 0, -1.0), 1e-15);

  // Tri-diagonal matrix with unique diagonal terms
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(5-sqrt(3), 5, 5+sqrt(3)),
    math::Vector3d(4.0, 5.0, 6.0),
    math::Vector3d(-1.0, 0, -1.0));
  // small magnitude, use tolerance of 1e-15
  VerifyNondiagonalMomentsAndAxes(1e-9*math::Vector3d(5-sqrt(3), 5, 5+sqrt(3)),
    1e-9 * math::Vector3d(4.0, 5.0, 6.0),
    1e-9 * math::Vector3d(-1.0, 0, -1.0), 1e-15);

  // Nonzero values for all off-axis terms
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(10, 12, 14),
    math::Vector3d(13, 11.75, 11.25),
    math::Vector3d(-0.5*sqrt(3), 1.5, 0.25*sqrt(3)));

  // Nonzero values for all off-axis terms
  VerifyNondiagonalMomentsAndAxes(
    math::Vector3d(6.6116, 8.2393186767, 13.983881323),
    math::Vector3d(11.6116, 8.6116, 8.6116),
    math::Vector3d(2, 2, 2));
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, EquivalentBox)
{
  // Default mass matrix with non-negative inertia
  {
    math::MassMatrix3d m;
    math::Vector3d size;
    math::Quaterniond rot;

    // size is all zeros, so SetFromBox should fail
    EXPECT_FALSE(m.SetFromBox(0.0, size, rot));
    EXPECT_FALSE(m.SetFromBox(size, rot));

    // even if mass is valid, it should not be set if size is invalid
    EXPECT_FALSE(m.SetFromBox(1.0, size, rot));
    EXPECT_DOUBLE_EQ(m.Mass(), 0.0);

    // equivalent box should not be findable
    EXPECT_FALSE(m.EquivalentBox(size, rot));
  }

  // Moment of inertia matrix that doesn't satisfy triangle inequality
  {
    const math::Vector3d ixxyyzz(2.0, 2.0, 2.0);
    const math::Vector3d ixyxzyz(-1.0, 0, -1.0);
    math::MassMatrix3d m(1.0, ixxyyzz, ixyxzyz);
    math::Vector3d size;
    math::Quaterniond rot;
    EXPECT_FALSE(m.EquivalentBox(size, rot));
  }

  // Identity inertia matrix
  // expect cube with side length sqrt(6)
  {
    const double mass = 1.0;
    math::MassMatrix3d m(mass, math::Vector3d::One, math::Vector3d::Zero);
    math::Vector3d size;
    math::Vector3d sizeTrue(sqrt(6) * math::Vector3d::One);
    math::Quaterniond rot;
    math::Quaterniond rotTrue(math::Quaterniond::Identity);
    EXPECT_TRUE(m.EquivalentBox(size, rot));
    EXPECT_EQ(size, sizeTrue);
    EXPECT_EQ(rot, rotTrue);

    // create new MassMatrix3d
    // it initially has zero mass, so SetFromBox(size, rot) will fail
    math::MassMatrix3d m2;
    EXPECT_FALSE(m2.SetFromBox(sizeTrue, rotTrue));
    EXPECT_TRUE(m2.SetFromBox(mass, sizeTrue, rotTrue));
    EXPECT_EQ(m, m2);

    double density = mass / (sizeTrue.X() * sizeTrue.Y() * sizeTrue.Z());
    math::Material mat(density);
    EXPECT_DOUBLE_EQ(density, mat.Density());
    math::MassMatrix3d m3;
    EXPECT_TRUE(m3.SetFromBox(mat, sizeTrue, rotTrue));
    EXPECT_EQ(m2, m3);
  }

  // unit box with mass 1.0
  {
    const double mass = 1.0;
    const math::Vector3d size(1, 1, 1);
    double ixx = mass/12 * (std::pow(size.Y(), 2) + std::pow(size.Z(), 2));
    double iyy = mass/12 * (std::pow(size.Z(), 2) + std::pow(size.X(), 2));
    double izz = mass/12 * (std::pow(size.X(), 2) + std::pow(size.Y(), 2));
    math::Vector3d ixxyyzz(ixx, iyy, izz);
    math::MassMatrix3d m(mass, ixxyyzz, math::Vector3d::Zero);
    math::Vector3d size2;
    math::Quaterniond rot;
    EXPECT_TRUE(m.EquivalentBox(size2, rot));
    EXPECT_EQ(size, size2);
    EXPECT_EQ(rot, math::Quaterniond::Identity);

    math::MassMatrix3d m2;
    EXPECT_TRUE(m2.SetFromBox(mass, size, rot));
    EXPECT_EQ(m, m2);
  }

  // box 1x4x9
  {
    const double mass = 12.0;
    const math::Vector3d ixxyyzz(97, 82, 17);
    math::MassMatrix3d m(mass, ixxyyzz, math::Vector3d::Zero);
    math::Vector3d size;
    math::Quaterniond rot;
    EXPECT_TRUE(m.EquivalentBox(size, rot));
    EXPECT_EQ(size, math::Vector3d(1, 4, 9));
    EXPECT_EQ(rot, math::Quaterniond::Identity);

    math::MassMatrix3d m2;
    EXPECT_TRUE(m2.SetFromBox(mass, size, rot));
    EXPECT_EQ(m, m2);
  }

  // box 1x4x9 rotated by 90 degrees around Z
  {
    const double mass = 12.0;
    const math::Vector3d ixxyyzz(82, 17, 97);
    math::MassMatrix3d m(mass, ixxyyzz, math::Vector3d::Zero);
    math::Vector3d size;
    math::Quaterniond rot;
    EXPECT_TRUE(m.EquivalentBox(size, rot, -1e-6));
    EXPECT_EQ(size, math::Vector3d(9, 4, 1));
    EXPECT_EQ(rot, math::Quaterniond(0, 0, IGN_PI/2));

    math::MassMatrix3d m2;
    EXPECT_TRUE(m2.SetFromBox(mass, size, rot));
    EXPECT_EQ(m, m2);
  }

  // box 1x4x9 rotated by 45 degrees around Z
  {
    const double mass = 12.0;
    const math::Vector3d ixxyyzz(49.5, 49.5, 97);
    const math::Vector3d ixyxzyz(-32.5, 0.0, 0.0);
    math::MassMatrix3d m(mass, ixxyyzz, ixyxzyz);
    math::Vector3d size;
    math::Quaterniond rot;
    EXPECT_TRUE(m.EquivalentBox(size, rot));
    EXPECT_EQ(size, math::Vector3d(9, 4, 1));
    // There are multiple correct rotations due to box symmetry
    EXPECT_TRUE(rot == math::Quaterniond(0, 0, IGN_PI/4) ||
                rot == math::Quaterniond(IGN_PI, 0, IGN_PI/4));

    math::MassMatrix3d m2;
    EXPECT_TRUE(m2.SetFromBox(mass, size, rot));
    EXPECT_EQ(m, m2);
  }

  // long slender box
  {
    const double mass = 12.0;
    const math::Vector3d ixxyyzz(1, 1, 2e-6);
    math::MassMatrix3d m(mass, ixxyyzz, math::Vector3d::Zero);
    math::Vector3d size;
    math::Quaterniond rot;
    EXPECT_TRUE(m.EquivalentBox(size, rot));
    EXPECT_EQ(size, math::Vector3d(1e-3, 1e-3, 1));
    EXPECT_EQ(rot, math::Quaterniond::Identity);

    math::MassMatrix3d m2;
    EXPECT_TRUE(m2.SetFromBox(mass, size, rot));
    EXPECT_EQ(m, m2);
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, SetFromCylinderZ)
{
  const math::Quaterniond q0 = math::Quaterniond::Identity;

  // Default mass matrix with non-positive inertia
  {
    math::MassMatrix3d m;

    // input is all zeros, so SetFromCylinderZ should fail
    EXPECT_FALSE(m.SetFromCylinderZ(0, 0, 0, q0));
    EXPECT_FALSE(m.SetFromCylinderZ(0, 0, q0));

    // even if some parameters are valid, none should be set if they
    // are not all valid
    EXPECT_FALSE(m.SetFromCylinderZ(1, 0, 0, q0));
    EXPECT_FALSE(m.SetFromCylinderZ(1, 1, 0, q0));
    EXPECT_FALSE(m.SetFromCylinderZ(1, 0, 1, q0));
    EXPECT_DOUBLE_EQ(m.Mass(), 0.0);
  }

  // unit cylinder with mass 1.0
  {
    const double mass = 1.0;
    const double length = 1.0;
    const double radius = 0.5;
    math::MassMatrix3d m;
    EXPECT_TRUE(m.SetFromCylinderZ(mass, length, radius, q0));

    double ixx = mass / 12.0 * (3*std::pow(radius, 2) + std::pow(length, 2));
    double iyy = ixx;
    double izz = mass / 2.0 * std::pow(radius, 2);
    const math::Vector3d ixxyyzz(ixx, iyy, izz);
    EXPECT_EQ(m.DiagonalMoments(), ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), math::Vector3d::Zero);

    double density = mass / (IGN_PI * radius * radius * length);
    math::Material mat(density);
    EXPECT_DOUBLE_EQ(density, mat.Density());
    math::MassMatrix3d m1;
    EXPECT_FALSE(m1.SetFromCylinderZ(math::Material(0), length, radius));
    EXPECT_TRUE(m1.SetFromCylinderZ(mat, length, radius));
    EXPECT_EQ(m, m1);

    // double the length and radius
    EXPECT_TRUE(m.SetFromCylinderZ(mass, 2*length, 2*radius, q0));
    EXPECT_EQ(m.DiagonalMoments(), 4*ixxyyzz);
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, SetFromSphere)
{
  // Default mass matrix with non-positive inertia
  {
    math::MassMatrix3d m;

    // input is all zeros, so SetFromSphere should fail
    EXPECT_FALSE(m.SetFromSphere(0.0, 0.0));
    EXPECT_FALSE(m.SetFromSphere(0.0));

    // even if mass is valid, it should not be set if radius is invalid
    EXPECT_FALSE(m.SetFromSphere(1.0, 0.0));
    EXPECT_DOUBLE_EQ(m.Mass(), 0.0);
  }

  // unit sphere with mass 1.0
  {
    const double mass = 1.0;
    const double radius = 0.5;
    math::MassMatrix3d m;
    EXPECT_TRUE(m.SetFromSphere(mass, radius));

    double ixx = 0.4 * mass * std::pow(radius, 2);
    double iyy = ixx;
    double izz = ixx;
    const math::Vector3d ixxyyzz(ixx, iyy, izz);
    EXPECT_EQ(m.DiagonalMoments(), ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), math::Vector3d::Zero);

    double density = mass / ((4.0/3.0) * IGN_PI * std::pow(radius, 3));
    math::Material mat(density);
    EXPECT_DOUBLE_EQ(density, mat.Density());
    math::MassMatrix3d m1;
    EXPECT_FALSE(m1.SetFromSphere(mat, 0));
    EXPECT_FALSE(m1.SetFromSphere(math::Material(0), 0));
    EXPECT_TRUE(m1.SetFromSphere(mat, radius));
    EXPECT_EQ(m, m1);

    // double the radius
    EXPECT_TRUE(m.SetFromSphere(mass, 2*radius));
    EXPECT_EQ(m.DiagonalMoments(), 4*ixxyyzz);
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, ValidMomentsTolerance)
{
  math::Vector3d moments;
  EXPECT_TRUE(math::MassMatrix3d::ValidMoments(moments, 0));
  EXPECT_TRUE(math::MassMatrix3d::ValidMoments(moments));

  math::MassMatrix3d massMatrix;
  // Default inertia matrix is all zeros.
  // Since the tolerance is relative to the max inertia,
  // it has no effect when max inertia is zero.
  // IsValid and IsNearPositive will be true
  EXPECT_TRUE(massMatrix.IsValid());
  EXPECT_TRUE(massMatrix.IsValid(10));
  EXPECT_TRUE(massMatrix.IsValid(0));
  EXPECT_TRUE(massMatrix.IsValid(-1));
  EXPECT_TRUE(massMatrix.IsNearPositive());
  EXPECT_TRUE(massMatrix.IsNearPositive(10));
  EXPECT_TRUE(massMatrix.IsNearPositive(0));
  EXPECT_TRUE(massMatrix.IsNearPositive(-1));
  // and IsPositive will be false
  EXPECT_FALSE(massMatrix.IsPositive());
  EXPECT_FALSE(massMatrix.IsPositive(10));
  EXPECT_FALSE(massMatrix.IsPositive(0));
  EXPECT_FALSE(massMatrix.IsPositive(-1));

  // setting Ixx = Iyy > 0 with Izz = 0
  // satisfies the triangle inequality
  massMatrix.SetIxx(0.1);
  massMatrix.SetIyy(0.1);
  // IsValid, IsNearPositive, and IsPositive will have same
  // behavior if tolerance >= 0
  EXPECT_TRUE(massMatrix.IsValid());
  EXPECT_TRUE(massMatrix.IsValid(10));
  EXPECT_TRUE(massMatrix.IsValid(0));
  EXPECT_TRUE(massMatrix.IsNearPositive());
  EXPECT_TRUE(massMatrix.IsNearPositive(10));
  EXPECT_TRUE(massMatrix.IsNearPositive(0));
  EXPECT_FALSE(massMatrix.IsPositive());
  EXPECT_FALSE(massMatrix.IsPositive(10));
  EXPECT_FALSE(massMatrix.IsPositive(0));
  // but they are all false if the tolerance is negative
  EXPECT_FALSE(massMatrix.IsValid(-1));
  EXPECT_FALSE(massMatrix.IsNearPositive(-1));
  EXPECT_FALSE(massMatrix.IsPositive(-1));
}
