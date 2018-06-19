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

#include "ignition/math/Box.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(BoxTest, Constructor)
{
  // Default constructor
  {
    math::Boxd box;
    EXPECT_EQ(math::Vector3d::Zero, box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2;
    EXPECT_EQ(box, box2);
  }

  // Individual dimension constructor
  {
    math::Boxd box(1.0, 2.0, 3.0);
    EXPECT_EQ(math::Vector3d(1.0, 2.0, 3.0), box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2(1.0, 2.0, 3.0);
    EXPECT_EQ(box, box2);
  }

  // Vector dimension constructor
  {
    math::Boxd box(math::Vector3d(1.3, 2.5, 4.6));
    EXPECT_EQ(math::Vector3d(1.3, 2.5, 4.6), box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2(math::Vector3d(1.3, 2.5, 4.6));
    EXPECT_EQ(box, box2);
  }

  // Dimension and mat constructor
  {
    math::Boxd box(1.0, 2.0, 5.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(math::Vector3d(1.0, 2.0, 5.0), box.Size());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), box.Material());

    math::Boxd box2(1.0, 2.0, 5.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(box, box2);
  }

  // Vector Dimension and mat constructor
  {
    math::Boxd box(math::Vector3d(2.2, 2.0, 10.0),
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(math::Vector3d(2.2, 2.0, 10.0), box.Size());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), box.Material());

    math::Boxd box2(math::Vector3d(2.2, 2.0, 10.0),
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(box, box2);
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, Mutators)
{
  math::Boxd box;
  box.SetSize(100.1, 2.3, 5.6);
  box.SetMaterial(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(100.1, box.Size().X());
  EXPECT_DOUBLE_EQ(2.3, box.Size().Y());
  EXPECT_DOUBLE_EQ(5.6, box.Size().Z());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), box.Material());

  box.SetSize(math::Vector3d(3.4, 1.2, 0.5));
  EXPECT_DOUBLE_EQ(3.4, box.Size().X());
  EXPECT_DOUBLE_EQ(1.2, box.Size().Y());
  EXPECT_DOUBLE_EQ(0.5, box.Size().Z());
}

//////////////////////////////////////////////////
TEST(BoxTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Boxd box(1.0, 0.1, 10.4);
  double expectedVolume = 1.0 * 0.1 * 10.4;
  EXPECT_DOUBLE_EQ(expectedVolume, box.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, box.DensityFromMass(mass));

  // Bad density
  math::Boxd box2;
  EXPECT_GT(0.0, box2.DensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(BoxTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double w = 0.1;
  double h = 34.12;
  math::Boxd box(l, w, h);
  box.SetDensityFromMass(mass);

  math::MassMatrix3d massMat;
  double ixx = (1.0/12.0) * mass * (w*w + h*h);
  double iyy = (1.0/12.0) * mass * (l*l + h*h);
  double izz = (1.0/12.0) * mass * (l*l + w*w);

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixx, iyy, izz, 0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  box.MassMatrix(massMat);
  EXPECT_EQ(expectedMassMat, massMat);
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat.Mass());
}
