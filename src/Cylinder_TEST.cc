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

#include "ignition/math/Cylinder.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(CylinderTest, Constructor)
{
  // Default constructor
  {
    math::Cylinderd cylinder;
    EXPECT_DOUBLE_EQ(0.0, cylinder.Length());
    EXPECT_DOUBLE_EQ(0.0, cylinder.Radius());
    EXPECT_EQ(math::Quaterniond::Identity, cylinder.RotationalOffset());
    EXPECT_EQ(math::Material(), cylinder.Mat());

    math::Cylinderd cylinder2;
    EXPECT_EQ(cylinder, cylinder2);
  }

  // Length and radius constructor
  {
    math::Cylinderd cylinder(1.0, 2.0);
    EXPECT_DOUBLE_EQ(1.0, cylinder.Length());
    EXPECT_DOUBLE_EQ(2.0, cylinder.Radius());
    EXPECT_EQ(math::Quaterniond::Identity, cylinder.RotationalOffset());
    EXPECT_EQ(math::Material(), cylinder.Mat());

    math::Cylinderd cylinder2(1.0, 2.0);
    EXPECT_EQ(cylinder, cylinder2);
  }

  // Length, radius, and rot constructor
  {
    math::Cylinderd cylinder(1.0, 2.0, math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_DOUBLE_EQ(1.0, cylinder.Length());
    EXPECT_DOUBLE_EQ(2.0, cylinder.Radius());
    EXPECT_EQ(math::Quaterniond(0.1, 0.2, 0.3),
        cylinder.RotationalOffset());
    EXPECT_EQ(math::Material(), cylinder.Mat());

    math::Cylinderd cylinder2(1.0, 2.0, math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_EQ(cylinder, cylinder2);
  }

  // Length, radius, mat and rot constructor
  {
    math::Cylinderd cylinder(1.0, 2.0,
        math::Material(math::MaterialType::WOOD),
        math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_DOUBLE_EQ(1.0, cylinder.Length());
    EXPECT_DOUBLE_EQ(2.0, cylinder.Radius());
    EXPECT_EQ(math::Quaterniond(0.1, 0.2, 0.3),
        cylinder.RotationalOffset());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), cylinder.Mat());

    math::Cylinderd cylinder2(1.0, 2.0,
        math::Material(math::MaterialType::WOOD),
        math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_EQ(cylinder, cylinder2);
  }
}

//////////////////////////////////////////////////
TEST(CylinderTest, Mutators)
{
  math::Cylinderd cylinder;
  EXPECT_DOUBLE_EQ(0.0, cylinder.Length());
  EXPECT_DOUBLE_EQ(0.0, cylinder.Radius());
  EXPECT_EQ(math::Quaterniond::Identity, cylinder.RotationalOffset());
  EXPECT_EQ(math::Material(), cylinder.Mat());

  cylinder.SetLength(100.1);
  cylinder.SetRadius(.123);
  cylinder.SetRotationalOffset(math::Quaterniond(1.2, 2.3, 3.4));
  cylinder.SetMat(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(100.1, cylinder.Length());
  EXPECT_DOUBLE_EQ(.123, cylinder.Radius());
  EXPECT_EQ(math::Quaterniond(1.2, 2.3, 3.4),
    cylinder.RotationalOffset());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), cylinder.Mat());
}

//////////////////////////////////////////////////
TEST(CylinderTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Cylinderd cylinder(1.0, 0.001);
  double expectedVolume = (IGN_PI * std::pow(0.001, 2) * 1.0);
  EXPECT_DOUBLE_EQ(expectedVolume, cylinder.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, cylinder.DensityFromMass(mass));

  // Bad density
  math::Cylinderd cylinder2;
  EXPECT_GT(0.0, cylinder2.DensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(CylinderTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double r = 0.1;
  math::Cylinderd cylinder(l, r);
  cylinder.SetDensityFromMass(mass);

  math::MassMatrix3d massMat;
  double ixxIyy = (1/12.0) * mass * (3*r*r + l*l);
  double izz = 0.5 * mass * r * r;

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixxIyy, ixxIyy, izz, 0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  cylinder.MassMatrix(massMat);
  EXPECT_EQ(expectedMassMat, massMat);
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat.Mass());
}
