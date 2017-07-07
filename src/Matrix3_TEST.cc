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

#include "ignition/math/Helpers.hh"
#include "ignition/math/Matrix3.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Matrix3dTest, Matrix3d)
{
  {
    math::Matrix3d matrix;
    EXPECT_TRUE(matrix == math::Matrix3d(0, 0, 0, 0, 0, 0, 0, 0, 0));
  }

  {
    math::Matrix3d matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    EXPECT_TRUE(matrix == math::Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9));

    math::Matrix3d matrix1(matrix);
    EXPECT_TRUE(matrix1 == math::Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9));
  }

  math::Matrix3d matrix;
  matrix.Axes(math::Vector3d(1, 1, 1), math::Vector3d(2, 2, 2),
                     math::Vector3d(3, 3, 3));
  EXPECT_TRUE(matrix == math::Matrix3d(1, 2, 3, 1, 2, 3, 1, 2, 3));

  matrix.Axis(math::Vector3d(1, 1, 1), IGN_PI);
  EXPECT_TRUE(matrix == math::Matrix3d(1, 2, 2, 2, 1, 2, 2, 2, 1));

  matrix.Col(0, math::Vector3d(3, 4, 5));
  EXPECT_TRUE(matrix == math::Matrix3d(3, 2, 2, 4, 1, 2, 5, 2, 1));

  EXPECT_NO_THROW(matrix.Col(3, math::Vector3d(1, 1, 1)));
  EXPECT_TRUE(matrix == math::Matrix3d(3, 2, 1, 4, 1, 1, 5, 2, 1));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, NoIndexException)
{
  math::Matrix3d mat = math::Matrix3d::Zero;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NO_THROW(mat(i, j));

  EXPECT_NO_THROW(math::equal(mat(3, 0), 0.0));
  EXPECT_NO_THROW(math::equal(mat(0, 3), 0.0));
  EXPECT_NO_THROW(math::equal(mat(3, 3), 0.0));

  EXPECT_NO_THROW(mat(3, 0) = 0);
  EXPECT_NO_THROW(mat(0, 3) = 0);
  EXPECT_NO_THROW(mat(3, 3) = 0);

  const math::Matrix3d constMat(math::Matrix3d::Zero);

  EXPECT_NO_THROW(math::equal(constMat(3, 0), 0.0));
  EXPECT_NO_THROW(math::equal(constMat(0, 3), 0.0));
  EXPECT_NO_THROW(math::equal(constMat(3, 3), 0.0));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorSubtract)
{
  math::Matrix3d matZero = math::Matrix3d::Zero;
  math::Matrix3d matIdent = math::Matrix3d::Identity;

  math::Matrix3d mat = matIdent - matZero;
  EXPECT_EQ(mat, matIdent);

  math::Matrix3d matA(1, 2, 3,
                      4, 5, 6,
                      7, 8, 9);

  math::Matrix3d matB(10, 20, 30,
                      40, 50, 60,
                      70, 80, 90);

  mat = matB - matA;
  EXPECT_EQ(mat, math::Matrix3d(9, 18, 27, 36, 45, 54, 63, 72, 81));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorAdd)
{
  math::Matrix3d matZero = math::Matrix3d::Zero;
  math::Matrix3d matIdent = math::Matrix3d::Identity;

  math::Matrix3d mat = matIdent + matZero;
  EXPECT_EQ(mat, matIdent);

  math::Matrix3d matA(1, 2, 3,
                      4, 5, 6,
                      7, 8, 9);

  math::Matrix3d matB(10, 20, 30,
                      40, 50, 60,
                      70, 80, 90);

  mat = matB + matA;
  EXPECT_EQ(mat, math::Matrix3d(11, 22, 33, 44, 55, 66, 77, 88, 99));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorMul)
{
  math::Matrix3d matZero = math::Matrix3d::Zero;
  math::Matrix3d matIdent = math::Matrix3d::Identity;

  math::Matrix3d mat = matIdent * matZero;
  EXPECT_EQ(mat, matZero);

  math::Matrix3d matA(1, 2, 3,
                      4, 5, 6,
                      7, 8, 9);

  math::Matrix3d matB(10, 20, 30,
                      40, 50, 60,
                      70, 80, 90);

  mat = matA * matB;
  EXPECT_EQ(mat, math::Matrix3d(300, 360, 420,
                                660, 810, 960,
                                1020, 1260, 1500));

  mat = matB * matA;
  EXPECT_EQ(mat, math::Matrix3d(300, 360, 420,
                                660, 810, 960,
                                1020, 1260, 1500));

  mat = mat * 2.0;
  EXPECT_EQ(mat, math::Matrix3d(600, 720, 840,
                                1320, 1620, 1920,
                                2040, 2520, 3000));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorStreamOut)
{
  math::Matrix3d matA(1, 2, 3,
                      4, 5, 6,
                      7, 8, 9);

  std::ostringstream stream;
  stream << matA;
  EXPECT_EQ(stream.str(), "1 2 3 4 5 6 7 8 9");
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorStreamIn)
{
  math::Matrix3d mat;
  EXPECT_EQ(mat, math::Matrix3d::Zero);

  std::istringstream stream("1 2 3 4 5 6 7 8 9");
  stream >> mat;
  EXPECT_EQ(mat, math::Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, Vector3Multiplication)
{
  {
    // Multiply arbitrary matrix by zeros of different sizes
    math::Matrix3d matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);

    // Scalar 0
    EXPECT_EQ(math::Matrix3d::Zero, matrix * 0);
    EXPECT_EQ(math::Matrix3d::Zero, 0 * matrix);

    // Vector3::Zero
    EXPECT_EQ(math::Vector3d::Zero, matrix * math::Vector3d::Zero);
    EXPECT_EQ(math::Vector3d::Zero, math::Vector3d::Zero * matrix);
    // left multiply with Vector3 not implemented

    // Matrix3::Zero
    EXPECT_EQ(math::Matrix3d::Zero, matrix * math::Matrix3d::Zero);
    EXPECT_EQ(math::Matrix3d::Zero, math::Matrix3d::Zero * matrix);
  }

  {
    // Multiply arbitrary matrix by identity values
    math::Matrix3d matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);

    // scalar 1.0
    EXPECT_EQ(matrix, matrix * 1.0);
    EXPECT_EQ(matrix, 1.0 * matrix);

    // Vector3::Unit[X|Y|Z]
    // right multiply
    EXPECT_EQ(math::Vector3d(matrix(0, 0), matrix(1, 0), matrix(2, 0)),
              matrix * math::Vector3d::UnitX);
    EXPECT_EQ(math::Vector3d(matrix(0, 1), matrix(1, 1), matrix(2, 1)),
              matrix * math::Vector3d::UnitY);
    EXPECT_EQ(math::Vector3d(matrix(0, 2), matrix(1, 2), matrix(2, 2)),
              matrix * math::Vector3d::UnitZ);
    // left multiply
    EXPECT_EQ(math::Vector3d(matrix(0, 0), matrix(0, 1), matrix(0, 2)),
              math::Vector3d::UnitX * matrix);
    EXPECT_EQ(math::Vector3d(matrix(1, 0), matrix(1, 1), matrix(1, 2)),
              math::Vector3d::UnitY * matrix);
    EXPECT_EQ(math::Vector3d(matrix(2, 0), matrix(2, 1), matrix(2, 2)),
              math::Vector3d::UnitZ * matrix);

    // Matrix3::IDENTITY
    EXPECT_EQ(matrix, matrix * math::Matrix3d::Identity);
    EXPECT_EQ(matrix, math::Matrix3d::Identity * matrix);
  }

  {
    // Multiply arbitrary matrix by itself
    math::Matrix3d matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    math::Matrix3d matrix2(30,  36,  42,
                           66,  81,  96,
                           102, 126, 150);

    EXPECT_EQ(matrix * matrix, matrix2);
  }
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, NotEqual)
{
  {
    math::Matrix3d matrix1;
    math::Matrix3d matrix2;
    EXPECT_TRUE(matrix1 == matrix2);
    EXPECT_FALSE(matrix1 != matrix2);
  }

  {
    math::Matrix3d matrix1(1, 2, 3, 4, 5, 6, 7, 8, 9);
    math::Matrix3d matrix2(matrix1);

    EXPECT_FALSE(matrix1 != matrix1);

    matrix2(0, 0) = 1.00001;
    EXPECT_TRUE(matrix1 != matrix2);

    matrix2(0, 0) = 1.000001;
    EXPECT_FALSE(matrix1 != matrix2);
  }
}

/////////////////////////////////////////////////
// Test Equal function with specified tolerance
TEST(Matrix3Test, EqualTolerance)
{
  EXPECT_FALSE(math::Matrix3d::Zero.Equal(math::Matrix3d::Identity, 1e-6));
  EXPECT_FALSE(math::Matrix3d::Zero.Equal(math::Matrix3d::Identity, 1e-3));
  EXPECT_FALSE(math::Matrix3d::Zero.Equal(math::Matrix3d::Identity, 1e-1));
  EXPECT_TRUE(math::Matrix3d::Zero.Equal(math::Matrix3d::Identity, 1));
  EXPECT_TRUE(math::Matrix3d::Zero.Equal(math::Matrix3d::Identity, 1.1));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, Inverse)
{
  // Inverse of identity matrix is itself
  EXPECT_EQ(math::Matrix3d::Identity, math::Matrix3d::Identity.Inverse());

  // Matrix multiplied by its inverse results in the identity matrix
  math::Matrix3d matrix1(-2, 4, 0, 0.1, 9, 55, -7, 1, 26);
  math::Matrix3d matrix2 = matrix1.Inverse();
  EXPECT_EQ(matrix1 * matrix2, math::Matrix3d::Identity);
  EXPECT_EQ(matrix2 * matrix1, math::Matrix3d::Identity);

  // Inverse of inverse results in the same matrix
  EXPECT_EQ((matrix1.Inverse()).Inverse(), matrix1);

  // Invert multiplication by scalar
  double scalar = 2.5;
  EXPECT_EQ((matrix1 * scalar).Inverse(), matrix1.Inverse() * (1.0/scalar));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, Determinant)
{
  // |Zero matrix| = 0.0
  EXPECT_DOUBLE_EQ(0.0, math::Matrix3d::Zero.Determinant());

  // |Identity matrix| = 1.0
  EXPECT_DOUBLE_EQ(1.0, math::Matrix3d::Identity.Determinant());

  // Determinant of arbitrary matrix
  math::Matrix3d m(-2, 4, 0, 0.1, 9, 55, -7, 1, 26);
  EXPECT_DOUBLE_EQ(-1908.4, m.Determinant());
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, Transpose)
{
  // Transpose of zero matrix is itself
  EXPECT_EQ(math::Matrix3d::Zero, math::Matrix3d::Zero.Transposed());

  // Transpose of identity matrix is itself
  EXPECT_EQ(math::Matrix3d::Identity, math::Matrix3d::Identity.Transposed());

  // Matrix and expected transpose
  math::Matrix3d m(-2, 4, 0,
                  0.1, 9, 55,
                   -7, 1, 26);
  math::Matrix3d mT(-2, 0.1, -7,
                     4,   9, 1,
                     0,  55, 26);
  EXPECT_NE(m, mT);
  EXPECT_EQ(m.Transposed(), mT);
  EXPECT_DOUBLE_EQ(m.Determinant(), m.Transposed().Determinant());

  mT.Transpose();
  EXPECT_EQ(m, mT);
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, From2Axes)
{
  math::Vector3d v1(1.0, 0.0, 0.0);
  math::Vector3d v2(0.0, 1.0, 0.0);

  math::Matrix3d m1;
  m1.From2Axes(v1, v2);

  math::Matrix3d m2;
  m2.From2Axes(v2, v1);

  math::Matrix3d m1Correct(0, -1, 0,
                           1, 0, 0,
                           0, 0, 1);
  math::Matrix3d m2Correct(m1Correct);
  m2Correct.Transpose();

  EXPECT_NE(m1, m2);
  EXPECT_EQ(m1Correct, m1);
  EXPECT_EQ(m2Correct, m2);
  EXPECT_EQ(math::Matrix3d::Identity, m1 * m2);
  EXPECT_EQ(v2, m1 * v1);
  EXPECT_EQ(v1, m2 * v2);

  // rotation about 45 degrees
  v1.Set(1.0, 0.0, 0.0);
  v2.Set(1.0, 1.0, 0.0);
  m2.From2Axes(v1, v2);
  // m1 is 90 degrees rotation
  EXPECT_EQ(m1, m2*m2);

  // with non-unit vectors
  v1.Set(0.5, 0.5, 0);
  v2.Set(-0.5, 0.5, 0);

  m1.From2Axes(v1, v2);
  m2.From2Axes(v2, v1);

  EXPECT_NE(m1, m2);
  EXPECT_EQ(m1Correct, m1);
  EXPECT_EQ(m2Correct, m2);
  EXPECT_EQ(math::Matrix3d::Identity, m1 * m2);
  EXPECT_EQ(v2, m1 * v1);
  EXPECT_EQ(v1, m2 * v2);

  // For zero-length vectors, a unit matrix is returned
  v1.Set(0, 0, 0);
  v2.Set(-0.5, 0.5, 0);
  m1.From2Axes(v1, v2);
  EXPECT_EQ(math::Matrix3d::Identity, m1);

  // For zero-length vectors, a unit matrix is returned
  v1.Set(-0.5, 0.5, 0);
  v2.Set(0, 0, 0);
  m1.From2Axes(v1, v2);
  EXPECT_EQ(math::Matrix3d::Identity, m1);

  // Parallel vectors
  v1.Set(1, 0, 0);
  v2.Set(2, 0, 0);
  m1.From2Axes(v1, v2);
  EXPECT_EQ(math::Matrix3d::Identity, m1);

  // Opposite vectors
  v1.Set(1, 0, 0);
  v2.Set(-2, 0, 0);
  m1.From2Axes(v1, v2);
  EXPECT_EQ(math::Matrix3d::Zero - math::Matrix3d::Identity, m1);
}
