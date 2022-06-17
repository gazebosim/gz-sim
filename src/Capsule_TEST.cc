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

#include "gz/math/Capsule.hh"
#include "gz/math/Helpers.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(CapsuleTest, Constructor)
{
  // Default constructor
  {
    math::Capsuled capsule;
    EXPECT_DOUBLE_EQ(0.0, capsule.Length());
    EXPECT_DOUBLE_EQ(0.0, capsule.Radius());
    EXPECT_EQ(math::Material(), capsule.Mat());

    math::Capsuled capsule2;
    EXPECT_EQ(capsule, capsule2);
  }

  // Length and radius constructor
  {
    math::Capsuled capsule(1.0, 2.0);
    EXPECT_DOUBLE_EQ(1.0, capsule.Length());
    EXPECT_DOUBLE_EQ(2.0, capsule.Radius());
    EXPECT_EQ(math::Material(), capsule.Mat());

    math::Capsuled capsule2(1.0, 2.0);
    EXPECT_EQ(capsule, capsule2);
  }

  // Length, radius, mat
  {
    math::Capsuled capsule(1.0, 2.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_DOUBLE_EQ(1.0, capsule.Length());
    EXPECT_DOUBLE_EQ(2.0, capsule.Radius());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), capsule.Mat());

    math::Capsuled capsule2(1.0, 2.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(capsule, capsule2);
  }
}

//////////////////////////////////////////////////
TEST(CapsuleTest, Mutators)
{
  math::Capsuled capsule;
  EXPECT_DOUBLE_EQ(0.0, capsule.Length());
  EXPECT_DOUBLE_EQ(0.0, capsule.Radius());
  EXPECT_EQ(math::Material(), capsule.Mat());

  capsule.SetLength(100.1);
  capsule.SetRadius(.123);
  capsule.SetMat(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(100.1, capsule.Length());
  EXPECT_DOUBLE_EQ(.123, capsule.Radius());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), capsule.Mat());
}

//////////////////////////////////////////////////
TEST(CapsuleTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Capsuled capsule(1.0, 0.001);
  double expectedVolume = (GZ_PI * std::pow(0.001, 2) * (1.0 + 4./3. * 0.001));
  EXPECT_DOUBLE_EQ(expectedVolume, capsule.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, capsule.DensityFromMass(mass));

  // Bad density
  math::Capsuled capsule2;
  EXPECT_TRUE(math::isnan(capsule2.DensityFromMass(mass)));
}

//////////////////////////////////////////////////
TEST(CapsuleTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double r = 0.1;
  math::Capsuled capsule(l, r);
  capsule.SetDensityFromMass(mass);

  const double cylinderVolume = GZ_PI * r*r * l;
  const double sphereVolume = GZ_PI * 4. / 3. * r*r*r;
  const double volume = cylinderVolume + sphereVolume;
  const double cylinderMass = mass * cylinderVolume / volume;
  const double sphereMass = mass * sphereVolume / volume;

  // expected values based on formula used in Open Dynamics Engine
  // https://bitbucket.org/odedevs/ode/src/0.16.2/ode/src/mass.cpp#lines-148:153
  // and the following article:
  // https://www.gamedev.net/tutorials/_/technical/math-and-physics/capsule-inertia-tensor-r3856/
  double ixxIyy = (1/12.0) * cylinderMass * (3*r*r + l*l)
    + sphereMass * (0.4*r*r + 0.375*r*l + 0.25*l*l);
  double izz = r*r * (0.5 * cylinderMass + 0.4 * sphereMass);

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixxIyy, ixxIyy, izz, 0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  auto massMat = capsule.MassMatrix();
  ASSERT_NE(std::nullopt, massMat);
  EXPECT_EQ(expectedMassMat, *massMat);
  EXPECT_EQ(expectedMassMat.DiagonalMoments(), massMat->DiagonalMoments());
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat->Mass());
}
