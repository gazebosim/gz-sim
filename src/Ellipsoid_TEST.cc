/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "gz/math/Ellipsoid.hh"
#include "gz/math/Helpers.hh"
#include "gz/math/Vector3.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(EllipsoidTest, Constructor)
{
  // Default constructor
  {
    math::Ellipsoidd ellipsoid;
    EXPECT_EQ(math::Vector3d::Zero, ellipsoid.Radii());
    EXPECT_EQ(math::Material(), ellipsoid.Mat());

    math::Ellipsoidd ellipsoid2;
    EXPECT_EQ(ellipsoid, ellipsoid2);
  }

  // Vector3 of radii constructor
  {
    const math::Vector3d expectedRadii(1.0, 2.0, 3.0);
    math::Ellipsoidd ellipsoid(expectedRadii);
    EXPECT_EQ(expectedRadii, ellipsoid.Radii());
    EXPECT_EQ(math::Material(), ellipsoid.Mat());

    math::Ellipsoidd ellipsoid2(expectedRadii);
    EXPECT_EQ(ellipsoid, ellipsoid2);
  }

  // Vector3 of radii and material
  {
    const math::Vector3d expectedRadii(1.0, 2.0, 3.0);
    const math::Material expectedMaterial(math::MaterialType::WOOD);
    math::Ellipsoidd ellipsoid(expectedRadii, expectedMaterial);
    EXPECT_EQ(expectedRadii, ellipsoid.Radii());
    EXPECT_EQ(expectedMaterial, ellipsoid.Mat());

    math::Ellipsoidd ellipsoid2(expectedRadii, expectedMaterial);
    EXPECT_EQ(ellipsoid, ellipsoid2);
  }
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, Mutators)
{
  math::Ellipsoidd ellipsoid;
  EXPECT_EQ(math::Vector3d::Zero, ellipsoid.Radii());
  EXPECT_EQ(math::Material(), ellipsoid.Mat());

  const math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  const math::Material expectedMaterial(math::MaterialType::PINE);
  ellipsoid.SetMat(expectedMaterial);

  EXPECT_EQ(expectedRadii, ellipsoid.Radii());
  EXPECT_EQ(expectedMaterial, ellipsoid.Mat());
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, VolumeAndDensity)
{
  double mass = 1.0;
  // Basic sphere
  math::Ellipsoidd ellipsoid(2. * math::Vector3d::One);

  double expectedVolume = (4. / 3.) * GZ_PI * std::pow(2.0, 3);
  EXPECT_DOUBLE_EQ(expectedVolume, ellipsoid.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, ellipsoid.DensityFromMass(mass));

  math::Ellipsoidd ellipsoid2(math::Vector3d(1, 10, 100));
  expectedVolume = (4. / 3.) * GZ_PI * 1. * 10. * 100.;
  EXPECT_DOUBLE_EQ(expectedVolume, ellipsoid2.Volume());

  expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, ellipsoid2.DensityFromMass(mass));

  // Check bad cases
  math::Ellipsoidd ellipsoid3(math::Vector3d::Zero);
  EXPECT_FALSE(ellipsoid3.SetDensityFromMass(mass));

  math::Ellipsoidd ellipsoid4(-math::Vector3d::One);
  EXPECT_FALSE(ellipsoid4.SetDensityFromMass(mass));

  math::Ellipsoidd ellipsoid5(math::Vector3d(-1, 1, 1));
  EXPECT_FALSE(ellipsoid5.SetDensityFromMass(mass));

  math::Ellipsoidd ellipsoid6(math::Vector3d(-1, -1, 1));
  EXPECT_FALSE(ellipsoid6.SetDensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, Mass)
{
  const double mass = 2.0;
  math::Ellipsoidd ellipsoid(math::Vector3d(1, 10, 100));
  ellipsoid.SetDensityFromMass(mass);

  const double ixx = (mass / 5.0) * (10. * 10. + 100. * 100.);
  const double iyy = (mass / 5.0) * (1. * 1. + 100. * 100.);
  const double izz = (mass / 5.0) * (1. * 1. + 10. * 10.);
  math::MassMatrix3d expectedMassMat(
    mass, math::Vector3d(ixx, iyy, izz), math::Vector3d::Zero);

  const auto massMat = ellipsoid.MassMatrix();
  ASSERT_NE(std::nullopt, massMat);
  EXPECT_EQ(expectedMassMat, *massMat);
  EXPECT_EQ(expectedMassMat.DiagonalMoments(), massMat->DiagonalMoments());
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat->Mass());

  // Zero case
  const math::Ellipsoidd ellipsoid2;
  EXPECT_EQ(std::nullopt, ellipsoid2.MassMatrix());

  // Check bad cases
  const math::Ellipsoidd ellipsoid3(-math::Vector3d::One);
  EXPECT_EQ(std::nullopt, ellipsoid3.MassMatrix());

  const math::Ellipsoidd ellipsoid4(math::Vector3d(-1, 1, 1));
  EXPECT_EQ(std::nullopt, ellipsoid4.MassMatrix());

  const math::Ellipsoidd ellipsoid5(math::Vector3d(-1, -1, 1));
  EXPECT_EQ(std::nullopt, ellipsoid5.MassMatrix());
}
