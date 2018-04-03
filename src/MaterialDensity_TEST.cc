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
#include "ignition/math/MaterialDensity.hh"
#include "ignition/math/Helpers.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(MaterialDensityTest, Init)
{
  const std::map<MaterialType, Material> &mats = MaterialDensity::Materials();
  EXPECT_FALSE(mats.empty());

  // Make sure that the number of elements in the MaterialType enum matches
  // the number of elements in the MaterialDensity::materials map.
  EXPECT_EQ(static_cast<size_t>(MaterialType::INVALID), mats.size());

  // Iterate over each element in the enum. Check the that enum value
  // matches the type value in the mats map.
  for (size_t i = 0; i < static_cast<size_t>(MaterialType::INVALID); ++i)
  {
    // Get the type of the material for MaterialType i.
    MaterialType type = mats.find(static_cast<MaterialType>(i))->second.type;
    EXPECT_EQ(i, static_cast<size_t>(type));

    // The name should not be empty
    EXPECT_FALSE(mats.find(static_cast<MaterialType>(i))->second.name.empty());

    // The density should be less than the max double value and greater than
    // zero.
    EXPECT_GT(MAX_D, mats.find(static_cast<MaterialType>(i))->second.density);
    EXPECT_LT(0.0, mats.find(static_cast<MaterialType>(i))->second.density);
  }
}

/////////////////////////////////////////////////
TEST(MaterialDensityTest, Accessors)
{
  {
    double density = MaterialDensity::Density("Aluminum");
    double density1 = MaterialDensity::Density("aluminum");
    double density2 = MaterialDensity::Density(MaterialType::ALUMINUM);

    EXPECT_DOUBLE_EQ(density, 2700);
    EXPECT_DOUBLE_EQ(density, density1);
    EXPECT_DOUBLE_EQ(density1, density2);
  }

  {
    double density = MaterialDensity::Density("Notfoundium");
    EXPECT_LT(density, 0.0);
  }

  {
    Material material = MaterialDensity::Nearest(19300.0);
    EXPECT_EQ(MaterialType::TUNGSTEN, material.type);
    EXPECT_DOUBLE_EQ(19300.0, material.density);
  }
  {
    Material material = MaterialDensity::Nearest(1001001.001, 1e-3);
    EXPECT_EQ(MaterialType::INVALID, material.type);
    EXPECT_LT(material.density, 0.0);
  }
  {
    Material material = MaterialDensity::Nearest(1001001.001);
    EXPECT_EQ(MaterialType::TUNGSTEN, material.type);
    EXPECT_DOUBLE_EQ(19300, material.density);
  }
}
