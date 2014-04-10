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

#include <gtest/gtest.h>

#include "ignition/math/Pose3d.hh"
#include "ignition/math/Quaterniond.hh"
#include "ignition/math/Matrix4d.hh"
#include "ignition/math/Vector3d.hh"

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
TEST(Matrix4dTest, Scale)
{
  math::Matrix4d mat, mat2;
  mat.SetScale(math::Vector3d(1, 2, 3));
  mat2.SetScale(1, 2, 3);

  EXPECT_EQ(mat, mat2);

  EXPECT_DOUBLE_EQ(mat(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(mat(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(mat(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(mat(3, 3), 1.0);

  EXPECT_DOUBLE_EQ(mat2(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(mat2(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(mat2(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(mat2(3, 3), 1.0);

  EXPECT_EQ(mat.GetScale(), mat2.GetScale());
  EXPECT_EQ(mat.GetScale(), math::Vector3d(1, 2, 3));

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
TEST(Matrix4dTest, Multiply3)
{
  math::Matrix4d mat;
  math::Matrix3d mat1;

  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      mat(i, j) = i-j;
      if (i < 3 && j < 3)
        mat1(j, i) = i+j;
    }
  }

  math::Matrix4d mat3(
      -5, -8, -11, -3,
      -2, -2, -2, -2,
      1, 4, 7, -1,
      3, 2, 1, 0);

  math::Matrix4d mat2 = mat * mat1;
  EXPECT_EQ(mat2, mat3);
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
  math::Pose3d pose = mat.GetAsPose();

  EXPECT_EQ(pose,
      math::Pose3d(5, 1, 2, -0.204124, 1.22474, 0.816497, 0.204124));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, Translate)
{
  math::Matrix4d mat, mat2;
  mat.SetTranslate(math::Vector3d(1, 2, 3));
  mat2.SetTranslate(1, 2, 3);

  EXPECT_EQ(mat, mat2);

  EXPECT_DOUBLE_EQ(mat(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(mat(1, 3), 2.0);
  EXPECT_DOUBLE_EQ(mat(2, 3), 3.0);

  EXPECT_DOUBLE_EQ(mat2(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(mat2(1, 3), 2.0);
  EXPECT_DOUBLE_EQ(mat2(2, 3), 3.0);

  EXPECT_EQ(mat.GetTranslation(), mat2.GetTranslation());
  EXPECT_EQ(mat.GetTranslation(), math::Vector3d(1, 2, 3));

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

  math::Quaterniond quat = mat.GetRotation();
  EXPECT_NEAR(quat.x(), 0.5, 1e-6);
  EXPECT_NEAR(quat.y(), 0.35, 1e-6);
  EXPECT_NEAR(quat.z(), 0.6, 1e-6);
  EXPECT_NEAR(quat.w(), 0.15, 1e-6);

  math::Vector3d euler = mat.GetEulerRotation(true);
  EXPECT_EQ(euler, math::Vector3d(1.5708, -1.11977, 1.5708));

  euler = mat.GetEulerRotation(false);
  EXPECT_EQ(euler, math::Vector3d(-1.5708, 4.26136, -1.5708));
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

  math::Quaterniond quat = mat.GetRotation();
  EXPECT_NEAR(quat.x(), 0.0896421, 1e-6);
  EXPECT_NEAR(quat.y(), -0.179284, 1e-6);
  EXPECT_NEAR(quat.z(), 0.0896421, 1e-6);
  EXPECT_NEAR(quat.w(), 0.83666, 1e-6);

  math::Vector3d euler = mat.GetEulerRotation(true);
  EXPECT_EQ(euler, math::Vector3d(0.737815, -1.11977, 1.3734));

  euler = mat.GetEulerRotation(false);
  EXPECT_EQ(euler, math::Vector3d(-2.40378, 4.26136, -1.76819));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, AffineTransform)
{
  math::Matrix4d mat = math::Matrix4d::Zero;
  math::Vector3d vec(1, 2, 3);

  EXPECT_THROW(mat.TransformAffine(vec), ignition::math::AffineException);

  mat = math::Matrix4d::Identity;
  EXPECT_NO_THROW(mat.TransformAffine(vec));
}

/////////////////////////////////////////////////
TEST(Matrix4dTest, IndexException)
{
  math::Matrix4d mat = math::Matrix4d::Zero;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      EXPECT_NO_THROW(mat(i, j));

  EXPECT_THROW(math::equal(mat(4, 0), 0.0), math::IndexException);
  EXPECT_THROW(math::equal(mat(0, 4), 0.0), math::IndexException);
  EXPECT_THROW(math::equal(mat(4, 4), 0.0), math::IndexException);

  EXPECT_THROW(mat(4, 0) = 0, math::IndexException);
  EXPECT_THROW(mat(0, 4) = 0, math::IndexException);
  EXPECT_THROW(mat(4, 4) = 0, math::IndexException);

  const math::Matrix4d constMat(math::Matrix4d::Zero);

  EXPECT_THROW(math::equal(constMat(4, 0), 0.0), math::IndexException);
  EXPECT_THROW(math::equal(constMat(0, 4), 0.0), math::IndexException);
  EXPECT_THROW(math::equal(constMat(4, 4), 0.0), math::IndexException);
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
