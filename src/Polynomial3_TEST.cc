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
#include <ostream>

#include "gz/math/Polynomial3.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(Polynomial3Test, DefaultConstructor)
{
  const math::Polynomial3d poly;
  EXPECT_EQ(poly.Coeffs(), math::Vector4d::Zero);
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, Constructor)
{
  const math::Polynomial3d poly(math::Vector4d::One);
  EXPECT_EQ(poly.Coeffs(), math::Vector4d::One);
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, ConstructionHelpers)
{
  const math::Polynomial3d poly = math::Polynomial3d::Constant(1.);
  EXPECT_EQ(poly.Coeffs(), math::Vector4d(0., 0., 0., 1.));
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, Evaluate)
{
  {
    const math::Polynomial3d p =
        math::Polynomial3d::Constant(1.);
    EXPECT_DOUBLE_EQ(p(-1.), 1.);
    EXPECT_DOUBLE_EQ(p(0.), 1.);
    EXPECT_DOUBLE_EQ(p(1.), 1.);
    EXPECT_DOUBLE_EQ(p(math::INF_D), 1.);
    EXPECT_TRUE(std::isnan(p(math::NAN_D)));
  }
  {
    const math::Polynomial3d p(math::Vector4d::One);
    EXPECT_DOUBLE_EQ(p(-1.), 0.);
    EXPECT_DOUBLE_EQ(p(0.), 1.);
    EXPECT_DOUBLE_EQ(p(1.), 4.);
    EXPECT_DOUBLE_EQ(p(-math::INF_D), -math::INF_D);
    EXPECT_TRUE(std::isnan(p(math::NAN_D)));
  }
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, Minimum)
{
  {
    const math::Polynomial3d p =
        math::Polynomial3d::Constant(1.);
    const math::Intervald xInterval =
        math::Intervald::Open(0., 0.);
    double xMin = 0.;
    EXPECT_TRUE(std::isnan(p.Minimum(xInterval)));
    EXPECT_TRUE(std::isnan(p.Minimum(xInterval, xMin)));
    EXPECT_TRUE(std::isnan(xMin));
  }
  {
    const math::Polynomial3d p =
        math::Polynomial3d::Constant(1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(), 1.);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), 1.);
    EXPECT_FALSE(std::isnan(xMin));
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., 0., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(0., 1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 1.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 1.);
    EXPECT_DOUBLE_EQ(xMin, 0.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., 0., 1., 1.));
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), -math::INF_D);
    EXPECT_DOUBLE_EQ(xMin, -math::INF_D);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., 0., -1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(0., 1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 0.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 0.);
    EXPECT_DOUBLE_EQ(xMin, 1.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., 0., -1., 1.));
    double xMin = 0.;
    EXPECT_DOUBLE_EQ(p.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), -math::INF_D);
    EXPECT_DOUBLE_EQ(xMin, math::INF_D);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., 1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(1., 2.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 3.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 3.);
    EXPECT_DOUBLE_EQ(xMin, 1.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., 1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-1., 0.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 0.75);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 0.75);
    EXPECT_DOUBLE_EQ(xMin, -0.5);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., 1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-3., -2.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 3.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 3.);
    EXPECT_DOUBLE_EQ(xMin, -2.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., 1., 1., 1.));
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(), 0.75);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), 0.75);
    EXPECT_DOUBLE_EQ(xMin, -0.5);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., -1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(1., 2.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), -1.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), -1.);
    EXPECT_DOUBLE_EQ(xMin, 2.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., -1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-2., -1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), -5.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), -5.);
    EXPECT_DOUBLE_EQ(xMin, -2.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., -1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(0., 1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 1.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 1.);
    EXPECT_DOUBLE_EQ(xMin, 0.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(0., -1., 1., 1.));
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), -math::INF_D);
    EXPECT_DOUBLE_EQ(xMin, -math::INF_D);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., 1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-1., 1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 0.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 0.);
    EXPECT_DOUBLE_EQ(xMin, -1.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., 1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-2., -1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), -5.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), -5.);
    EXPECT_DOUBLE_EQ(xMin, -2.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., 1., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(2., 3.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 15.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 15.);
    EXPECT_DOUBLE_EQ(xMin, 2.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., 1., 1., 1.));
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), -math::INF_D);
    EXPECT_DOUBLE_EQ(xMin, -math::INF_D);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(-1., 2., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-1., 1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 0.8873882090776197);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 0.8873882090776197);
    EXPECT_DOUBLE_EQ(xMin, -0.2152504370215302);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(-1., 2., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-3., -2.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 15.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 15.);
    EXPECT_DOUBLE_EQ(xMin, -2.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(-1., 2., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(1., 2.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 3.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 3.);
    EXPECT_DOUBLE_EQ(xMin, 1.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(-1., 2., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(2., 3.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), -5.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), -5.);
    EXPECT_DOUBLE_EQ(xMin, 3.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(-1., 2., 1., 1.));
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), -math::INF_D);
    EXPECT_DOUBLE_EQ(xMin, math::INF_D);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., -2., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-1., 1.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), -3.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), -3.);
    EXPECT_DOUBLE_EQ(xMin, -1.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., -2., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(0., 2.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 1.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 1.);
    EXPECT_DOUBLE_EQ(xMin, 0.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., -2., 1., 1.));
    const math::Intervald xInterval =
        math::Intervald::Open(2., 3.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), 3.);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), 3.);
    EXPECT_DOUBLE_EQ(xMin, 2.);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., -2., 1., 1.));
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), -math::INF_D);
    EXPECT_DOUBLE_EQ(xMin, -math::INF_D);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., -4., -2., -1.));
    const math::Intervald xInterval =
        math::Intervald::Open(-1., 6.);
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval), -16.051047904897441);
    EXPECT_DOUBLE_EQ(p.Minimum(xInterval, xMin), -16.051047904897441);
    EXPECT_DOUBLE_EQ(xMin, 2.8968052532744766);
    EXPECT_DOUBLE_EQ(p.Minimum(), -math::INF_D);
  }
  {
    const math::Polynomial3d p(
        math::Vector4d(1., -4., -2., -1.));
    double xMin = math::NAN_D;
    EXPECT_DOUBLE_EQ(p.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p.Minimum(xMin), -math::INF_D);
    EXPECT_DOUBLE_EQ(xMin, -math::INF_D);
  }
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, PolynomialStreaming)
{
  {
    std::ostringstream os;
    os << math::Polynomial3d(math::Vector4d::Zero);
    EXPECT_EQ(os.str(), "0");
  }
  {
    std::ostringstream os;
    os << math::Polynomial3d(math::Vector4d::One);
    EXPECT_EQ(os.str(), "x^3 + x^2 + x + 1");
  }
  {
    std::ostringstream os;
    os << math::Polynomial3d(
        math::Vector4d(1., 0., 1., 0.));
    EXPECT_EQ(os.str(), "x^3 + x");
  }
  {
    std::ostringstream os;
    os << math::Polynomial3d(
        math::Vector4d(0., 1., 0., -1.));
    EXPECT_EQ(os.str(), "x^2 - 1");
  }
  {
    std::ostringstream os;
    os << math::Polynomial3d(
        math::Vector4d(-1., 2., -2., 0.));
    EXPECT_EQ(os.str(), "-x^3 + 2 x^2 - 2 x");
  }
}
