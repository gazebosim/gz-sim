/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <gz/math/MassMatrix3.hh>

#include "MeshInertiaCalculator.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
TEST(MeshInertiaCalculator, CorrectMassMatrix)
{
  // Verify a mass matrix with a small error can be corrected
  math::MassMatrix3d massMatrix(55.0,
                                math::Vector3d(7, 15, 23),
                                math::Vector3d::Zero);
  EXPECT_FALSE(massMatrix.IsValid());
  EXPECT_TRUE(massMatrix.IsPositive());
  // Set a small tolerance for the triangle inequality test and expect that
  // the mass matrix can not be corrected.
  EXPECT_FALSE(MeshInertiaCalculator::CorrectMassMatrix(massMatrix, 0.01));
  // Verify successful correction with default tolerance.
  EXPECT_TRUE(MeshInertiaCalculator::CorrectMassMatrix(massMatrix));
  EXPECT_TRUE(massMatrix.IsValid());

  // Verify a mass matrix with unordered diagonal moments and a small error
  // can be corrected, and that the elements in the corrected principal moments
  // maintain the same order.
  math::Vector3d ixxyyzz(15, 7, 23);
  massMatrix = math::MassMatrix3d(55.0,
                                  ixxyyzz,
                                  math::Vector3d::Zero);
  EXPECT_FALSE(massMatrix.IsValid());
  EXPECT_TRUE(massMatrix.IsPositive());
  EXPECT_TRUE(MeshInertiaCalculator::CorrectMassMatrix(massMatrix));
  EXPECT_TRUE(massMatrix.IsValid());
  EXPECT_NE(ixxyyzz, massMatrix.DiagonalMoments());
  EXPECT_LT(massMatrix.PrincipalMoments()[1], massMatrix.PrincipalMoments()[0]);
  EXPECT_LT(massMatrix.PrincipalMoments()[0], massMatrix.PrincipalMoments()[2]);

  // Verify a mass matrix with non-zero off-diagonal moments can be corrected
  massMatrix = math::MassMatrix3d(1.0,
                                  math::Vector3d(2.0, 2.0, 2.0),
                                  math::Vector3d(-1, 0, -0.1));
  EXPECT_FALSE(massMatrix.IsValid());
  EXPECT_TRUE(massMatrix.IsPositive());
  EXPECT_TRUE(MeshInertiaCalculator::CorrectMassMatrix(massMatrix));
  EXPECT_TRUE(massMatrix.IsValid());

  // Verify a mass matrix with a large error can not be corrected.
  massMatrix = math::MassMatrix3d(15.0,
                                  math::Vector3d(1, 2, 23),
                                  math::Vector3d::Zero);
  EXPECT_FALSE(massMatrix.IsValid());
  EXPECT_TRUE(massMatrix.IsPositive());
  EXPECT_FALSE(MeshInertiaCalculator::CorrectMassMatrix(massMatrix));
  EXPECT_FALSE(massMatrix.IsValid());

  // Verify a mass matrix with non positive-definite inertia matrix can not
  // be corrected.
  massMatrix = math::MassMatrix3d(15.0,
                                  math::Vector3d(12, -15, 23),
                                  math::Vector3d::Zero);
  EXPECT_FALSE(massMatrix.IsValid());
  EXPECT_FALSE(massMatrix.IsPositive());
  EXPECT_FALSE(MeshInertiaCalculator::CorrectMassMatrix(massMatrix));
  EXPECT_FALSE(massMatrix.IsPositive());
  EXPECT_FALSE(massMatrix.IsValid());

  // Verify that CorrectMassMatrix returns true when given a valid mass matrix
  // and does not change the mass matrix data.
  massMatrix = math::MassMatrix3d(15.0,
                                  math::Vector3d(12, 15, 23),
                                  math::Vector3d::Zero);
  math::MassMatrix3d originalMassMatrix = massMatrix;
  EXPECT_TRUE(massMatrix.IsValid());
  EXPECT_TRUE(MeshInertiaCalculator::CorrectMassMatrix(massMatrix));
  EXPECT_TRUE(massMatrix.IsValid());
  EXPECT_DOUBLE_EQ(originalMassMatrix.Mass(),
              massMatrix.Mass());
  EXPECT_EQ(originalMassMatrix.DiagonalMoments(),
            massMatrix.DiagonalMoments());
  EXPECT_EQ(originalMassMatrix.OffDiagonalMoments(),
            massMatrix.OffDiagonalMoments());
}
