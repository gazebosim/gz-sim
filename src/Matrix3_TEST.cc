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

#include "ignition/math/Matrix3d.hh"

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
  matrix.SetFromAxes(math::Vector3d(1, 1, 1), math::Vector3d(2, 2, 2),
                     math::Vector3d(3, 3, 3));
  EXPECT_TRUE(matrix == math::Matrix3d(1, 2, 3, 1, 2, 3, 1, 2, 3));

  matrix.SetFromAxis(math::Vector3d(1, 1, 1), M_PI);
  EXPECT_TRUE(matrix == math::Matrix3d(1, 2, 2, 2, 1, 2, 2, 2, 1));

  matrix.SetCol(0, math::Vector3d(3, 4, 5));
  EXPECT_TRUE(matrix == math::Matrix3d(3, 2, 2, 4, 1, 2, 5, 2, 1));

  EXPECT_THROW(matrix.SetCol(3, math::Vector3d(1, 1, 1)),
      ignition::math::IndexException);
}
