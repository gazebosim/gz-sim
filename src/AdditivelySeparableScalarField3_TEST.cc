/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <functional>
#include <ostream>

#include "gz/math/AdditivelySeparableScalarField3.hh"
#include "gz/math/Polynomial3.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(AdditivelySeparableScalarField3Test, Evaluate)
{
  using ScalarFunctionT = std::function<double(double)>;
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<ScalarFunctionT>;
  const double kConstant = 1.;
  auto xyzFunc = [](double x) { return x; };
  const AdditivelySeparableScalarField3dT scalarField(
      kConstant, xyzFunc, xyzFunc, xyzFunc);
  EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::Zero), 0.);
  EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::One), 3.);
  EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::UnitX), 1.);
  EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::UnitY), 1.);
  EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::UnitZ), 1.);
  const math::Vector3d INF_V(
      math::INF_D, math::INF_D, math::INF_D);
  EXPECT_DOUBLE_EQ(scalarField(INF_V), math::INF_D);
}

/////////////////////////////////////////////////
TEST(AdditivelySeparableScalarField3Test, Minimum)
{
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<math::Polynomial3d>;
  constexpr double kConstant = 1. / 3.;
  const math::Polynomial3d xyzPoly(math::Vector4d(0., 1., 1., 1));
  const AdditivelySeparableScalarField3dT scalarField(
      kConstant, xyzPoly, xyzPoly, xyzPoly);
  {
    const math::Region3d region =
        math::Region3d::Open(0., 0., 0., 0., 0., 0.);
    math::Vector3d pMin = math::Vector3d::Zero;
    EXPECT_TRUE(std::isnan(scalarField.Minimum(region)));
    EXPECT_TRUE(std::isnan(scalarField.Minimum(region, pMin)));
    EXPECT_TRUE(std::isnan(pMin.X()));
    EXPECT_TRUE(std::isnan(pMin.Y()));
    EXPECT_TRUE(std::isnan(pMin.Z()));
  }
  {
    math::Vector3d pMin = math::Vector3d::NaN;
    EXPECT_DOUBLE_EQ(scalarField.Minimum(), 0.75);
    EXPECT_DOUBLE_EQ(scalarField.Minimum(pMin), 0.75);
    EXPECT_EQ(pMin, -0.5 * math::Vector3d::One);
  }
  {
    const math::Region3d region =
        math::Region3d::Open(1., 1., 1., 2., 2., 2.);
    math::Vector3d pMin = math::Vector3d::NaN;
    EXPECT_DOUBLE_EQ(scalarField.Minimum(region), 3.);
    EXPECT_DOUBLE_EQ(scalarField.Minimum(region, pMin), 3.);
    EXPECT_EQ(pMin, math::Vector3d::One);
  }
}

/////////////////////////////////////////////////
TEST(AdditivelySeparableScalarField3Test, Stream)
{
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<math::Polynomial3d>;
  {
    constexpr double kConstant = 1.;
    const math::Polynomial3d xyzPoly(math::Vector4d(0., 1., 0., 1));
    std::ostringstream os;
    os << AdditivelySeparableScalarField3dT(
        kConstant, xyzPoly, xyzPoly, xyzPoly);
    EXPECT_EQ(os.str(), "[(x^2 + 1) + (y^2 + 1) + (z^2 + 1)]");
  }
  {
    constexpr double kConstant = -1.;
    const math::Polynomial3d xyzPoly(math::Vector4d(1., 0., 1., 0));
    std::ostringstream os;
    os << AdditivelySeparableScalarField3dT(
        kConstant, xyzPoly, xyzPoly, xyzPoly);
    EXPECT_EQ(os.str(), "-[(x^3 + x) + (y^3 + y) + (z^3 + z)]");
  }
  {
    constexpr double kConstant = 3.;
    const math::Polynomial3d xPoly(math::Vector4d(0., 1., 0., 0));
    const math::Polynomial3d yPoly(math::Vector4d(-1., 0., 0., 0));
    const math::Polynomial3d zPoly(math::Vector4d(0., 0., 0., 1));
    std::ostringstream os;
    os << AdditivelySeparableScalarField3dT(
        kConstant, xPoly, yPoly, zPoly);
    EXPECT_EQ(os.str(), "3 [(x^2) + (-y^3) + (1)]");
  }
}
