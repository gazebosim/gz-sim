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

#define _USE_MATH_DEFINES
#include <gtest/gtest.h>

#include "ignition/math/Helpers.hh"
#include "ignition/math/MassMatrix3.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, Constructors)
{
  // Simple constructor, test default values
  {
    math::MassMatrix3d m;
    EXPECT_DOUBLE_EQ(m.Mass(), 0.0);
    EXPECT_DOUBLE_EQ(m.IXX(), 0.0);
    EXPECT_DOUBLE_EQ(m.IYY(), 0.0);
    EXPECT_DOUBLE_EQ(m.IZZ(), 0.0);
    EXPECT_DOUBLE_EQ(m.IXY(), 0.0);
    EXPECT_DOUBLE_EQ(m.IXZ(), 0.0);
    EXPECT_DOUBLE_EQ(m.IYZ(), 0.0);
    EXPECT_EQ(m.DiagonalMoments(), math::Vector3d::Zero);
    EXPECT_EQ(m.OffDiagonalMoments(), math::Vector3d::Zero);
    EXPECT_EQ(m.MOI(), math::Matrix3d::Zero);
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }

  // Constructor with default arguments
  // Should match simple constructor and with copy constructor
  {
    math::MassMatrix3d m(0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_EQ(m, math::MassMatrix3d());
    EXPECT_EQ(m, math::MassMatrix3d(m));
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }

  // Constructor with non-default arguments
  {
    const double mass = 5.0;
    const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
    const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
    const math::Matrix3d MOI(2.0, 0.2, 0.3,
                             0.2, 3.0, 0.4,
                             0.3, 0.4, 4.0);
    math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);

    // Should not match simple constructor
    EXPECT_NE(m, math::MassMatrix3d());

    // Should match with copy constructor
    EXPECT_EQ(m, math::MassMatrix3d(m));

    // Test accessors
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
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
  const math::Matrix3d MOI(2.0, 0.2, 0.3,
                           0.2, 3.0, 0.4,
                           0.3, 0.4, 4.0);

  // Scalar setters with simple constructor
  // MassMatrix3 won't be valid until enough properties are set
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());

    // Initially invalid
    EXPECT_FALSE(m.Mass(mass));
    EXPECT_FALSE(m.IXX(Ixxyyzz[0]));
    EXPECT_FALSE(m.IYY(Ixxyyzz[1]));

    // Valid once enough properties are set
    EXPECT_TRUE(m.IZZ(Ixxyyzz[2]));
    EXPECT_TRUE(m.IXY(Ixyxzyz[0]));
    EXPECT_TRUE(m.IXZ(Ixyxzyz[1]));
    EXPECT_TRUE(m.IYZ(Ixyxzyz[2]));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid again if an invalid inertia is set
    EXPECT_FALSE(m.Mass(-1));
  }

  // Test vector setters for moment of inertia
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());

    // Initially invalid
    EXPECT_FALSE(m.Mass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.DiagonalMoments(Ixxyyzz));
    EXPECT_TRUE(m.OffDiagonalMoments(Ixyxzyz));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid if an invalid inertia is set
    EXPECT_FALSE(m.IXX(-1));
  }

  // Test Matrix3 setter for moment of inertia
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());

    // Initially invalid
    EXPECT_FALSE(m.Mass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.MOI(MOI));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid if an excessive off-diagonal inertia is set
    EXPECT_FALSE(m.IXY(1e3));
  }

  // // Test atomic InertiaMatrix setter
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());

    // Initially invalid
    EXPECT_FALSE(m.Mass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.InertiaMatrix(2, 3, 4, 0.2, 0.3, 0.4));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }
}

