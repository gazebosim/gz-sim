/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <iostream>

#include "ignition/math/Sphere.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(SphereTest, Constructor)
{
  // Default constructor
  {
    math::Sphered sphere;
    EXPECT_DOUBLE_EQ(0.0, sphere.Radius());
    EXPECT_EQ(math::Material(), sphere.Material());

    math::Sphered sphere2;
    EXPECT_EQ(sphere, sphere2);
  }

  // Radius constructor
  {
    math::Sphered sphere(1.0);
    EXPECT_DOUBLE_EQ(1.0, sphere.Radius());
    EXPECT_EQ(math::Material(), sphere.Material());

    math::Sphered sphere2(1.0);
    EXPECT_EQ(sphere, sphere2);
  }

  // Radius and mat
  {
    math::Sphered sphere(1.0, math::Material(math::MaterialType::WOOD));
    EXPECT_DOUBLE_EQ(1.0, sphere.Radius());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), sphere.Material());

    math::Sphered sphere2(1.0, math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(sphere, sphere2);
  }
}

//////////////////////////////////////////////////
TEST(SphereTest, Comparison)
{
  const math::Sphered wood(0.1, math::Material(math::MaterialType::WOOD));
  {
    math::Sphered modified = wood;
    EXPECT_EQ(wood, modified);

    modified.SetRadius(1.0);
    EXPECT_NE(wood, modified);
  }

  {
    math::Sphered modified = wood;
    EXPECT_EQ(wood, modified);

    modified.SetMaterial(math::Material(math::MaterialType::PINE));
    EXPECT_NE(wood, modified);
  }
}

//////////////////////////////////////////////////
TEST(SphereTest, Mutators)
{
  math::Sphered sphere;
  EXPECT_DOUBLE_EQ(0.0, sphere.Radius());
  EXPECT_EQ(math::Material(), sphere.Material());

  sphere.SetRadius(.123);
  sphere.SetMaterial(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(.123, sphere.Radius());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), sphere.Material());
}

//////////////////////////////////////////////////
TEST(SphereTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Sphered sphere(0.001);
  double expectedVolume = (4.0/3.0) * IGN_PI * std::pow(0.001, 3);
  EXPECT_DOUBLE_EQ(expectedVolume, sphere.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, sphere.DensityFromMass(mass));

  // Bad density
  math::Sphered sphere2;
  EXPECT_GT(0.0, sphere2.DensityFromMass(mass));
  sphere2.SetRadius(1.0);
  EXPECT_GT(0.0, sphere2.DensityFromMass(0.0));
  EXPECT_FALSE(sphere.SetDensityFromMass(0.0));
}

//////////////////////////////////////////////////
TEST(SphereTest, Mass)
{
  double mass = 2.0;
  double r = 0.1;
  math::Sphered sphere(r);
  EXPECT_TRUE(sphere.SetDensityFromMass(mass));

  math::MassMatrix3d massMat;
  double ixxIyyIzz = 0.4 * mass * r * r;

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixxIyyIzz, ixxIyyIzz, ixxIyyIzz,
      0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  sphere.MassMatrix(massMat);
  EXPECT_EQ(expectedMassMat, massMat);
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat.Mass());
}
