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

#include "ignition/math/Pose3.hh"
#include "ignition/math/Quaternion.hh"
#include "ignition/math/Matrix4.hh"
#include "ignition/math/Vector3.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Matrix4dTest, Construct)
{
  math::Matrix4d mat;
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      EXPECT_DOUBLE_EQ(mat(i, i), 0.0);
    }
  }

  math::Matrix4d mat2(mat);
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      EXPECT_DOUBLE_EQ(mat2(i, i), 0.0);
    }
  }
  EXPECT_TRUE(mat2 == mat);


  // Set individual values.
  math::Matrix4d mat3(0.0, 1.0, 2.0, 3.0,
                     4.0, 5.0, 6.0, 7.0,
                     8.0, 9.0, 10.0, 11.0,
                     12.0, 13.0, 14.0, 15.0);

  math::Matrix4d mat4;
  mat4 = mat3;
  EXPECT_EQ(mat4, mat3);

  EXPECT_DOUBLE_EQ(mat3(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(mat3(0, 1), 1.0);
  EXPECT_DOUBLE_EQ(mat3(0, 2), 2.0);
  EXPECT_DOUBLE_EQ(mat3(0, 3), 3.0);
  EXPECT_DOUBLE_EQ(mat3(1, 0), 4.0);
  EXPECT_DOUBLE_EQ(mat3(1, 1), 5.0);
  EXPECT_DOUBLE_EQ(mat3(1, 2), 6.0);
  EXPECT_DOUBLE_EQ(mat3(1, 3), 7.0);
  EXPECT_DOUBLE_EQ(mat3(2, 0), 8.0);
  EXPECT_DOUBLE_EQ(mat3(2, 1), 9.0);
  EXPECT_DOUBLE_EQ(mat3(2, 2), 10.0);
  EXPECT_DOUBLE_EQ(mat3(2, 3), 11.0);
  EXPECT_DOUBLE_EQ(mat3(3, 0), 12.0);
  EXPECT_DOUBLE_EQ(mat3(3, 1), 13.0);
  EXPECT_DOUBLE_EQ(mat3(3, 2), 14.0);
  EXPECT_DOUBLE_EQ(mat3(3, 3), 15.0);
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, ConstructFromPose3d)
{
  {
    math::Vector3d trans(1, 2, 3);
    math::Quaterniond qt(0.1, 0.2, 0.3);
    math::Pose3d pose(trans, qt);
    math::Matrix4d mat(pose);

    EXPECT_EQ(pose, mat.Pose());
    EXPECT_EQ(trans, mat.Translation());
    EXPECT_EQ(qt, mat.Rotation());
    EXPECT_EQ(pose.Inverse(), mat.Inverse().Pose());
    // ensure inverses multiply to identity
    EXPECT_EQ(mat.Inverse() * mat, math::Matrix4d::Identity);
    EXPECT_EQ(mat * mat.Inverse(), math::Matrix4d::Identity);
    EXPECT_EQ(pose.Inverse() * pose, math::Pose3d::Zero);
    EXPECT_EQ(pose * pose.Inverse(), math::Pose3d::Zero);
    // repeat test with *=
    {
      math::Matrix4d m = math::Matrix4d::Identity;
      m *= mat;
      EXPECT_EQ(m, mat);
      m *= mat.Inverse();
      EXPECT_EQ(m, math::Matrix4d::Identity);
    }
    {
      math::Pose3d p;
      p *= pose;
      EXPECT_EQ(p, pose);
      p *= pose.Inverse();
      EXPECT_EQ(p, math::Pose3d::Zero);
    }
  }

  // Zero values
  {
    math::Vector3d trans(0, 0, 0);
    math::Quaterniond qt(0, 0, 0);
    math::Pose3d pose(trans, qt);
    math::Matrix4d mat(pose);

    EXPECT_EQ(pose, mat.Pose());
    EXPECT_EQ(trans, mat.Translation());
    EXPECT_EQ(qt, mat.Rotation());
    EXPECT_EQ(pose.Inverse(), mat.Inverse().Pose());
  }

  // Rotate pitch by pi/2 so yaw coincides with roll causing a gimbal lock
  {
    math::Vector3d trans(3, 2, 1);
    math::Quaterniond qt(0, IGN_PI/2, 0);
    math::Pose3d pose(trans, qt);
    math::Matrix4d mat(pose);

    EXPECT_EQ(pose, mat.Pose());
    EXPECT_EQ(trans, mat.Translation());
    EXPECT_EQ(qt, mat.Rotation());
    EXPECT_EQ(pose.Inverse(), mat.Inverse().Pose());
  }

  {
    // setup a ZXZ rotation to ensure non-commutative rotations
    math::Pose3d pose1(1, -2, 3, 0, 0, IGN_PI/4);
    math::Pose3d pose2(0, 1, -1, -IGN_PI/4, 0, 0);
    math::Pose3d pose3(-1, 0, 0, 0, 0, -IGN_PI/4);

    math::Matrix4d m1(pose1);
    math::Matrix4d m2(pose2);
    math::Matrix4d m3(pose3);

    // ensure rotations are not commutative
    EXPECT_NE(m1 * m2 * m3, m3 * m2 * m1);

    // ensure pose multiplication order matches matrix order
    EXPECT_EQ(m1 * m2 * m3, math::Matrix4d(pose1 * pose2 * pose3));
    EXPECT_EQ(m3 * m2 * m1, math::Matrix4d(pose3 * pose2 * pose1));

    // repeat test with *=
    {
      math::Matrix4d m = math::Matrix4d::Identity;
      math::Pose3d p;

      m *= m1;
      p *= pose1;
      EXPECT_EQ(m, m1);
      EXPECT_EQ(p, pose1);
      EXPECT_EQ(m, math::Matrix4d(p));

      m *= m2;
      p *= pose2;
      EXPECT_EQ(m, m1 * m2);
      EXPECT_EQ(p, pose1 * pose2);
      EXPECT_EQ(m, math::Matrix4d(p));

      m *= m3;
      p *= pose3;
      EXPECT_EQ(m, m1 * m2 * m3);
      EXPECT_EQ(p, pose1 * pose2 * pose3);
      EXPECT_EQ(m, math::Matrix4d(p));
    }
  }
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, CoverageExtra)
{
  // getting full destructor coverage
  math::Matrix4d *p = new math::Matrix4d;
  EXPECT_NE(p, nullptr);
  delete p;
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, Scale)
{
  math::Matrix4d mat, mat2;
  mat.Scale(math::Vector3d(1, 2, 3));
  mat2.Scale(1, 2, 3);

  EXPECT_EQ(mat, mat2);

  EXPECT_DOUBLE_EQ(mat(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(mat(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(mat(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(mat(3, 3), 1.0);

  EXPECT_DOUBLE_EQ(mat2(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(mat2(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(mat2(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(mat2(3, 3), 1.0);

  EXPECT_EQ(mat.Scale(), mat2.Scale());
  EXPECT_EQ(mat.Scale(), math::Vector3d(1, 2, 3));

  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      if (i != j)
      {
        EXPECT_DOUBLE_EQ(mat(i, j), 0.0);
        EXPECT_DOUBLE_EQ(mat2(i, j), 0.0);
      }
      else if (i == 3 && j == 3)
      {
        EXPECT_DOUBLE_EQ(mat(i, j), 1.0);
        EXPECT_DOUBLE_EQ(mat2(i, j), 1.0);
      }
    }
  }
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, MultiplyV)
{
  math::Matrix4d mat;
  math::Vector3d vec(-1.2, 2.3, 10.5);

  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      mat(i, j) = i-j;
    }
  }

  EXPECT_EQ(mat * vec, math::Vector3d(-26.3, -13.7, -1.1));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, Multiply4)
{
  math::Matrix4d mat, mat1;

  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      mat(i, j) = i-j;
      mat1(j, i) = i+j;
    }
  }

  math::Matrix4d mat3(
      -14, -20, -26, -32,
      -8, -10, -12, -14,
      -2, 0, 2, 4,
      4, 10, 16, 22);

  math::Matrix4d mat2 = mat * mat1;
  EXPECT_EQ(mat2, mat3);

  math::Matrix4d mat4 = mat;
  mat4 *= mat1;
  EXPECT_EQ(mat2, mat4);
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, Inverse)
{
  math::Matrix4d mat(2, 3, 1, 5,
                    1, 0, 3, 1,
                    0, 2, -3, 2,
                    0, 2, 3, 1);

  math::Matrix4d mat1 = mat.Inverse();
  EXPECT_EQ(mat1, math::Matrix4d(18, -35, -28, 1,
                               9, -18, -14, 1,
                               -2, 4, 3, 0,
                               -12, 24, 19, -1));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, GetAsPose3d)
{
  math::Matrix4d mat(2, 3, 1, 5,
                    1, 0, 3, 1,
                    0, 2, -3, 2,
                    0, 2, 3, 1);
  math::Pose3d pose = mat.Pose();

  EXPECT_EQ(pose,
      math::Pose3d(5, 1, 2, -0.204124, 1.22474, 0.816497, 0.204124));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, Translation)
{
  math::Matrix4d mat, mat2;
  mat.SetTranslation(math::Vector3d(1, 2, 3));
  mat2.SetTranslation(1, 2, 3);

  EXPECT_EQ(mat, mat2);

  EXPECT_DOUBLE_EQ(mat(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(mat(1, 3), 2.0);
  EXPECT_DOUBLE_EQ(mat(2, 3), 3.0);

  EXPECT_DOUBLE_EQ(mat2(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(mat2(1, 3), 2.0);
  EXPECT_DOUBLE_EQ(mat2(2, 3), 3.0);

  EXPECT_EQ(mat.Translation(), mat2.Translation());
  EXPECT_EQ(mat.Translation(), math::Vector3d(1, 2, 3));

  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      EXPECT_DOUBLE_EQ(mat(i, j), 0.0);
      EXPECT_DOUBLE_EQ(mat2(i, j), 0.0);
    }
  }
  EXPECT_DOUBLE_EQ(mat(3, 3), 0.0);
  EXPECT_DOUBLE_EQ(mat2(3, 3), 0.0);
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, RotationDiagZero)
{
  math::Matrix4d mat;
  mat(0, 0) = 0;
  mat(0, 1) = 0.2;
  mat(0, 2) = 0.3;
  mat(0, 3) = 0.4;

  mat(1, 0) = 0.5;
  mat(1, 1) = 0;
  mat(1, 2) = 0.7;
  mat(1, 3) = 0.8;

  mat(2, 0) = 0.9;
  mat(2, 1) = 1.0;
  mat(2, 2) = 0;
  mat(2, 3) = 1.2;

  mat(3, 0) = 1.3;
  mat(3, 1) = 1.4;
  mat(3, 2) = 1.5;
  mat(3, 3) = 1.0;

  math::Quaterniond quat = mat.Rotation();
  EXPECT_NEAR(quat.X(), 0.5, 1e-6);
  EXPECT_NEAR(quat.Y(), 0.35, 1e-6);
  EXPECT_NEAR(quat.Z(), 0.6, 1e-6);
  EXPECT_NEAR(quat.W(), 0.15, 1e-6);

  math::Vector3d euler = mat.EulerRotation(true);
  EXPECT_EQ(euler, math::Vector3d(1.5708, -1.11977, 1.5708));

  euler = mat.EulerRotation(false);
  EXPECT_EQ(euler, math::Vector3d(-1.5708, 4.26136, -1.5708));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, RotationDiagLessThanZero)
{
  math::Matrix4d mat;
  mat(0, 0) = -0.1;
  mat(0, 1) = 0.2;
  mat(0, 2) = 0.3;
  mat(0, 3) = 0.4;

  mat(1, 0) = 0.5;
  mat(1, 1) = 0;
  mat(1, 2) = 0.7;
  mat(1, 3) = 0.8;

  mat(2, 0) = 0.9;
  mat(2, 1) = 1.0;
  mat(2, 2) = 0;
  mat(2, 3) = 1.2;

  mat(3, 0) = 1.3;
  mat(3, 1) = 1.4;
  mat(3, 2) = 1.5;
  mat(3, 3) = 1.0;

  {
    math::Quaterniond quat = mat.Rotation();
    EXPECT_NEAR(quat.X(), 0.333712, 1e-6);
    EXPECT_NEAR(quat.Y(), 0.524404, 1e-6);
    EXPECT_NEAR(quat.Z(), 0.810443, 1e-6);
    EXPECT_NEAR(quat.W(), -0.286039, 1e-6);

    math::Vector3d euler = mat.EulerRotation(true);
    EXPECT_EQ(euler, math::Vector3d(1.5708, -1.11977, 1.76819));

    euler = mat.EulerRotation(false);
    EXPECT_EQ(euler, math::Vector3d(-1.5708, 4.26136, -1.3734));
  }


  {
    mat(0, 0) = -0.1;
    mat(1, 1) = -0.2;
    mat(2, 2) = 0.0;

    math::Quaterniond quat = mat.Rotation();
    EXPECT_NEAR(quat.X(), 0.526235, 1e-6);
    EXPECT_NEAR(quat.Y(), 0.745499, 1e-6);
    EXPECT_NEAR(quat.Z(), 0.570088, 1e-6);
    EXPECT_NEAR(quat.W(), 0.131559, 1e-6);

    math::Vector3d euler = mat.EulerRotation(true);
    EXPECT_EQ(euler, math::Vector3d(1.5708, -1.11977, 1.76819));

    euler = mat.EulerRotation(false);
    EXPECT_EQ(euler, math::Vector3d(-1.5708, 4.26136, -1.3734));
  }
}


/////////////////////////////////////////////////
TEST(Matrix4dTest, Rotation)
{
  math::Matrix4d mat;
  mat(0, 0) = 0.1;
  mat(0, 1) = 0.2;
  mat(0, 2) = 0.3;
  mat(0, 3) = 0.4;

  mat(1, 0) = 0.5;
  mat(1, 1) = 0.6;
  mat(1, 2) = 0.7;
  mat(1, 3) = 0.8;

  mat(2, 0) = 0.9;
  mat(2, 1) = 1.0;
  mat(2, 2) = 1.1;
  mat(2, 3) = 1.2;

  mat(3, 0) = 1.3;
  mat(3, 1) = 1.4;
  mat(3, 2) = 1.5;
  mat(3, 3) = 1.6;

  math::Quaterniond quat = mat.Rotation();
  EXPECT_NEAR(quat.X(), 0.0896421, 1e-6);
  EXPECT_NEAR(quat.Y(), -0.179284, 1e-6);
  EXPECT_NEAR(quat.Z(), 0.0896421, 1e-6);
  EXPECT_NEAR(quat.W(), 0.83666, 1e-6);

  math::Vector3d euler = mat.EulerRotation(true);
  EXPECT_EQ(euler, math::Vector3d(0.737815, -1.11977, 1.3734));

  euler = mat.EulerRotation(false);
  EXPECT_EQ(euler, math::Vector3d(-2.40378, 4.26136, -1.76819));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, EulerRotation2)
{
  math::Matrix4d mat;
  mat(0, 0) = 0.1;
  mat(0, 1) = 0.2;
  mat(0, 2) = 0.3;
  mat(0, 3) = 0.4;

  mat(1, 0) = 0.5;
  mat(1, 1) = 0.6;
  mat(1, 2) = 0.7;
  mat(1, 3) = 0.8;

  mat(2, 0) = 1.9;
  mat(2, 1) = 1.2;
  mat(2, 2) = 1.1;
  mat(2, 3) = 1.2;

  mat(3, 0) = 1.3;
  mat(3, 1) = 1.4;
  mat(3, 2) = 1.5;
  mat(3, 3) = 1.6;

  math::Vector3d euler = mat.EulerRotation(true);
  EXPECT_EQ(euler, math::Vector3d(-2.55359, -1.5708, 0));

  euler = mat.EulerRotation(false);
  EXPECT_EQ(euler, math::Vector3d(-2.55359, -1.5708, 0));

  mat(2, 0) = -1.2;
  euler = mat.EulerRotation(true);
  EXPECT_EQ(euler, math::Vector3d(0.588003, 1.5708, 0));

  euler = mat.EulerRotation(false);
  EXPECT_EQ(euler, math::Vector3d(0.588003, 1.5708, 0));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, AffineTransform)
{
  math::Matrix4d mat = math::Matrix4d::Zero;
  math::Vector3d vec(1, 2, 3);

  math::Vector3d v;
  EXPECT_NO_THROW(mat.TransformAffine(vec, v));
  EXPECT_FALSE(mat.TransformAffine(vec, v));

  mat = math::Matrix4d::Identity;
  EXPECT_NO_THROW(mat.TransformAffine(vec, v));
  EXPECT_TRUE(mat.TransformAffine(vec, v));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, NoIndexException)
{
  math::Matrix4d mat = math::Matrix4d::Zero;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      EXPECT_NO_THROW(mat(i, j));

  EXPECT_NO_THROW(math::equal(mat(4, 0), 0.0));
  EXPECT_NO_THROW(math::equal(mat(0, 4), 0.0));
  EXPECT_NO_THROW(math::equal(mat(4, 4), 0.0));

  EXPECT_NO_THROW(mat(4, 0) = 0);
  EXPECT_NO_THROW(mat(0, 4) = 0);
  EXPECT_NO_THROW(mat(4, 4) = 0);

  const math::Matrix4d constMat(math::Matrix4d::Zero);

  EXPECT_NO_THROW(math::equal(constMat(4, 0), 0.0));
  EXPECT_NO_THROW(math::equal(constMat(0, 4), 0.0));
  EXPECT_NO_THROW(math::equal(constMat(4, 4), 0.0));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, OperatorStreamOut)
{
  math::Matrix4d matA(1, 2, 3, 4,
                      5, 6, 7, 8,
                      9, 10, 11, 12,
                      13, 14, 15, 16);

  std::ostringstream stream;
  stream << matA;
  EXPECT_EQ(stream.str(), "1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16");
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, OperatorStreamIn)
{
  math::Matrix4d mat;
  EXPECT_EQ(mat, math::Matrix4d::Zero);

  std::istringstream stream("1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16");
  stream >> mat;
  EXPECT_EQ(mat, math::Matrix4d(1, 2, 3, 4,
                                5, 6, 7, 8,
                                9, 10, 11, 12,
                                13, 14, 15, 16));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, NotEqual)
{
  {
    math::Matrix4d matrix1;
    math::Matrix4d matrix2;
    EXPECT_TRUE(matrix1 == matrix2);
    EXPECT_FALSE(matrix1 != matrix2);
  }

  {
    math::Matrix4d matrix1(1, 2, 3, 4,
                           5, 6, 7, 8,
                           9, 10, 11, 12,
                           13, 14, 15, 16);
    math::Matrix4d matrix2(matrix1);

    EXPECT_FALSE(matrix1 != matrix1);

    matrix2(0, 0) = 1.00001;
    EXPECT_TRUE(matrix1 != matrix2);

    matrix2(0, 0) = 1.000001;
    EXPECT_FALSE(matrix1 != matrix2);
  }
}

/////////////////////////////////////////////////
// Test Equal function with specified tolerance
TEST(Matrix4Test, EqualTolerance)
{
  EXPECT_FALSE(math::Matrix4d::Zero.Equal(math::Matrix4d::Identity, 1e-6));
  EXPECT_FALSE(math::Matrix4d::Zero.Equal(math::Matrix4d::Identity, 1e-3));
  EXPECT_FALSE(math::Matrix4d::Zero.Equal(math::Matrix4d::Identity, 1e-1));
  EXPECT_TRUE(math::Matrix4d::Zero.Equal(math::Matrix4d::Identity, 1));
  EXPECT_TRUE(math::Matrix4d::Zero.Equal(math::Matrix4d::Identity, 1.1));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, Determinant)
{
  // |Zero matrix| = 0.0
  EXPECT_DOUBLE_EQ(0.0, math::Matrix4d::Zero.Determinant());

  // |Identity matrix| = 1.0
  EXPECT_DOUBLE_EQ(1.0, math::Matrix4d::Identity.Determinant());

  // Determinant of arbitrary matrix
  math::Matrix4d m(2, 3, 0.1, -5, 1, 0, 3.2, 1,
                   0, 2, -3, 2.1, 0, 2, 3.2, 1);
  EXPECT_DOUBLE_EQ(129.82, m.Determinant());
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, Transpose)
{
  // Transpose of zero matrix is itself
  EXPECT_EQ(math::Matrix4d::Zero, math::Matrix4d::Zero.Transposed());

  // Transpose of identity matrix is itself
  EXPECT_EQ(math::Matrix4d::Identity, math::Matrix4d::Identity.Transposed());

  // Matrix and expected transpose
  math::Matrix4d m(-2, 4,  0, -3.5,
                  0.1, 9, 55,  1.2,
                   -7, 1, 26, 11.5,
                   .2, 3, -5, -0.1);
  math::Matrix4d mT(-2, 0.1,   -7, .2,
                     4,   9,    1, 3,
                     0,  55,   26, -5,
                  -3.5, 1.2, 11.5, -0.1);
  EXPECT_NE(m, mT);
  EXPECT_EQ(m.Transposed(), mT);
  EXPECT_DOUBLE_EQ(m.Determinant(), m.Transposed().Determinant());

  mT.Transpose();
  EXPECT_EQ(m, mT);
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, LookAt)
{
  EXPECT_EQ(math::Matrix4d::LookAt(-math::Vector3d::UnitX,
                                    math::Vector3d::Zero).Pose(),
            math::Pose3d(-1, 0, 0, 0, 0, 0));

  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d(3, 2, 0),
                                   math::Vector3d(0, 2, 0)).Pose(),
            math::Pose3d(3, 2, 0, 0, 0, IGN_PI));

  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d(1, 6, 1),
                                   math::Vector3d::One).Pose(),
            math::Pose3d(1, 6, 1, 0, 0, -IGN_PI_2));

  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d(-1, -1, 0),
                                   math::Vector3d(1, 1, 0)).Pose(),
            math::Pose3d(-1, -1, 0, 0, 0, IGN_PI_4));

  // Default up is Z
  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d(0.1, -5, 222),
                                   math::Vector3d(999, -0.6, 0)),
            math::Matrix4d::LookAt(math::Vector3d(0.1, -5, 222),
                                   math::Vector3d(999, -0.6, 0),
                                   math::Vector3d::UnitZ));

  // up == zero, default up = +Z
  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d(1.23, 456, 0.7),
                                   math::Vector3d(0, 8.9, -10),
                                   math::Vector3d::Zero),
            math::Matrix4d::LookAt(math::Vector3d(1.23, 456, 0.7),
                                   math::Vector3d(0, 8.9, -10)));

  // up == +X, default up = +Z
  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d(0.25, 9, -5),
                                   math::Vector3d(-6, 0, 0.4),
                                   math::Vector3d::UnitX),
            math::Matrix4d::LookAt(math::Vector3d(0.25, 9, -5),
                                   math::Vector3d(-6, 0, 0.4)));

  // up == -X, default up = +Z
  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d(0, 0, 0.2),
                                   math::Vector3d(-8, 0, -6),
                                   -math::Vector3d::UnitX),
            math::Matrix4d::LookAt(math::Vector3d(0, 0, 0.2),
                                   math::Vector3d(-8, 0, -6)));

  // eye == target, default direction = +X
  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d::One,
                                   math::Vector3d::One),
            math::Matrix4d::LookAt(math::Vector3d::One,
                                   math::Vector3d(1.0001, 1, 1)));

  // Not possible to keep _up on +Z
  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d(-1, 0, 10),
                                   math::Vector3d(-1, 0, 0)),
            math::Matrix4d::LookAt(math::Vector3d(-1, 0, 10),
                                   math::Vector3d(-1, 0, 0),
                                   -math::Vector3d::UnitX));

  // Different ups
  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d::One,
                                   math::Vector3d(0, 1, 1),
                                   math::Vector3d::UnitY).Pose(),
            math::Pose3d(1, 1, 1, IGN_PI_2, 0, IGN_PI));

  EXPECT_EQ(math::Matrix4d::LookAt(math::Vector3d::One,
                                   math::Vector3d(0, 1, 1),
                                   math::Vector3d(0, 1, 1)).Pose(),
            math::Pose3d(1, 1, 1, IGN_PI_4, 0, IGN_PI));
}

