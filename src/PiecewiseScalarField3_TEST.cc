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

#include <cmath>
#include <functional>

#include "gz/math/AdditivelySeparableScalarField3.hh"
#include "gz/math/PiecewiseScalarField3.hh"
#include "gz/math/Polynomial3.hh"
#include "gz/math/Vector3.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(PiecewiseScalarField3Test, Evaluate)
{
  using ScalarField3dT = std::function<double(const math::Vector3d&)>;
  using PiecewiseScalarField3dT = math::PiecewiseScalarField3d<ScalarField3dT>;
  {
    const PiecewiseScalarField3dT scalarField;
    EXPECT_TRUE(std::isnan(scalarField(math::Vector3d::Zero)));
  }
  {
    const PiecewiseScalarField3dT scalarField =
        PiecewiseScalarField3dT::Throughout(
            [](const math::Vector3d& v) { return v.X(); });
    EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::Zero), 0.);
    EXPECT_DOUBLE_EQ(scalarField(math::Vector3d(0.5, 0.5, 0.5)), 0.5);
    EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::One), 1.);
    EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::UnitX), 1.);
  }
  {
    const math::Region3d region0 =
        math::Region3d::Open(0., 0., 0., 0., 0., 0.);
    auto field0 = [](const math::Vector3d&) { return 1.; };
    const math::Region3d region1 =
        math::Region3d::Closed(0., 0., 0., 1., 1., 1.);
    auto field1 = [](const math::Vector3d& v) { return v.X(); };
    const PiecewiseScalarField3dT scalarField({
        {region0, field0}, {region1, field1}});
    EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::Zero), 0.);
  }
  {
    const math::Region3d region0 =
        math::Region3d::Closed(0., 0., 0., 0., 0., 0.);
    auto field0 = [](const math::Vector3d&) { return 1.; };
    const math::Region3d region1 =
        math::Region3d::Closed(0., 0., 0., 1., 1., 1.);
    auto field1 = [](const math::Vector3d& v) { return v.X(); };
    const PiecewiseScalarField3dT scalarField({
        {region0, field0}, {region1, field1}});
    EXPECT_DOUBLE_EQ(scalarField(math::Vector3d::Zero), 1.);
  }
  {
    const math::Region3d region0 =
        math::Region3d::Open(0., 0., 0., 1., 1., 1.);
    auto field0 = [](const math::Vector3d& v) { return v.X(); };
    const math::Region3d region1 =
        math::Region3d::Open(-1., -1., -1., 0., 0., 0.);
    auto field1 = [](const math::Vector3d& v) { return v.Y(); };
    const PiecewiseScalarField3dT scalarField({
        {region0, field0}, {region1, field1}});
    EXPECT_DOUBLE_EQ(scalarField(math::Vector3d(0.5, 0.25, 0.5)), 0.5);
    EXPECT_DOUBLE_EQ(scalarField(math::Vector3d(-0.5, -0.25, -0.5)), -0.25);
    EXPECT_TRUE(std::isnan(scalarField(math::Vector3d(0.5, -0.25, 0.5))));
    EXPECT_TRUE(std::isnan(scalarField(math::Vector3d(-0.5, 0.25, -0.5))));
  }
}

/////////////////////////////////////////////////
TEST(PiecewiseScalarField3Test, Minimum)
{
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<math::Polynomial3d>;
  using PiecewiseScalarField3dT =
      math::PiecewiseScalarField3d<AdditivelySeparableScalarField3dT>;
  {
    const PiecewiseScalarField3dT scalarField;
    math::Vector3d pMin = math::Vector3d::Zero;
    EXPECT_TRUE(std::isnan(scalarField.Minimum()));
    EXPECT_TRUE(std::isnan(scalarField.Minimum(pMin)));
    EXPECT_TRUE(std::isnan(pMin.X()));
    EXPECT_TRUE(std::isnan(pMin.Y()));
    EXPECT_TRUE(std::isnan(pMin.Z()));
  }
  {
    const PiecewiseScalarField3dT scalarField =
        PiecewiseScalarField3dT::Throughout(
            AdditivelySeparableScalarField3dT(
                1.,
                math::Polynomial3d::Constant(0.),
                math::Polynomial3d::Constant(1.),
                math::Polynomial3d::Constant(0.)));
    math::Vector3d pMin = math::Vector3d::NaN;
    EXPECT_DOUBLE_EQ(scalarField.Minimum(), 1.);
    EXPECT_DOUBLE_EQ(scalarField.Minimum(pMin), 1.);
    EXPECT_FALSE(std::isnan(pMin.X()));
    EXPECT_FALSE(std::isnan(pMin.Y()));
    EXPECT_FALSE(std::isnan(pMin.Z()));
  }
  {
    const math::Region3d region0 =
        math::Region3d::Open(0., 0., 0., 1., 1., 1.);
    const AdditivelySeparableScalarField3dT field0(
        1.,
        math::Polynomial3d(
            math::Vector4d(0., 1., 0., 0.)),
        math::Polynomial3d::Constant(0.),
        math::Polynomial3d::Constant(0.));
    const math::Region3d region1 =
        math::Region3d::Open(-1., -1., -1., 0., 0., 0.);
    const AdditivelySeparableScalarField3dT field1(
        1.,
        math::Polynomial3d::Constant(0.),
        math::Polynomial3d(
            math::Vector4d(0., 1., 0., 1.)),
        math::Polynomial3d::Constant(0.));
    const PiecewiseScalarField3dT scalarField({
        {region0, field0}, {region1, field1}});
    math::Vector3d pMin = math::Vector3d::NaN;
    EXPECT_DOUBLE_EQ(scalarField.Minimum(), 0.);
    EXPECT_DOUBLE_EQ(scalarField.Minimum(pMin), 0.);
    EXPECT_EQ(pMin, math::Vector3d::Zero);
  }
}


/////////////////////////////////////////////////
TEST(PiecewiseScalarField3Test, Stream)
{
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<math::Polynomial3d>;
  using PiecewiseScalarField3dT =
      math::PiecewiseScalarField3d<AdditivelySeparableScalarField3dT>;
  {
    std::ostringstream output;
    output << PiecewiseScalarField3dT();
    EXPECT_EQ(output.str(), "undefined");
  }
  {
    std::ostringstream output;
    std::ostringstream expected;
    const math::Region3d region0(
        math::Intervald::Unbounded,
        math::Intervald::Unbounded,
        math::Intervald::Open(-math::INF_D, 0.));
    const AdditivelySeparableScalarField3dT field0(
        1.,
        math::Polynomial3d(math::Vector4d(0., 1., 0., 0.)),
        math::Polynomial3d(math::Vector4d(0., 1., 0., 0.)),
        math::Polynomial3d(math::Vector4d(0., 1., 0., 0.)));
    expected << "[(x^2) + (y^2) + (z^2)] if (x, y, z) "
             << "in (-inf, inf) x (-inf, inf) x (-inf, 0); ";
    const math::Region3d region1(
        math::Intervald::Unbounded,
        math::Intervald::Unbounded,
        math::Intervald::LeftClosed(0., math::INF_D));
    const AdditivelySeparableScalarField3dT field1(
        -1.,
        math::Polynomial3d(math::Vector4d(0., 0., 1., 0.)),
        math::Polynomial3d(math::Vector4d(0., 0., 1., 0.)),
        math::Polynomial3d(math::Vector4d(0., 0., 1., 0.)));
    expected << "-[(x) + (y) + (z)] if (x, y, z) "
             << "in (-inf, inf) x (-inf, inf) x [0, inf)";
    const PiecewiseScalarField3dT scalarField({
        {region0, field0}, {region1, field1}});
    output << scalarField;
    EXPECT_EQ(output.str(), expected.str());
  }
}
