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
#include "ignition/math/Material.hh"
#include "ignition/math/Helpers.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(MaterialTest, Init)
{
  const std::map<MaterialType, Material> &mats = Material::Materials();
  EXPECT_FALSE(mats.empty());

  // Make sure that the number of elements in the MaterialType enum matches
  // the number of elements in the MaterialDensity::materials map.
  EXPECT_EQ(static_cast<size_t>(MaterialType::INVALID), mats.size());

  // Iterate over each element in the enum. Check the that enum value
  // matches the type value in the mats map.
  for (size_t i = 0; i < static_cast<size_t>(MaterialType::INVALID); ++i)
  {
    // Get the type of the material for MaterialType i.
    MaterialType type = mats.find(static_cast<MaterialType>(i))->second.Type();
    EXPECT_EQ(i, static_cast<size_t>(type));

    // The name should not be empty
    EXPECT_FALSE(
        mats.find(static_cast<MaterialType>(i))->second.Name().empty());

    // The density should be less than the max double value and greater than
    // zero.
    EXPECT_GT(MAX_D, mats.find(static_cast<MaterialType>(i))->second.Density());
    EXPECT_LT(0.0, mats.find(static_cast<MaterialType>(i))->second.Density());
  }
}

/////////////////////////////////////////////////
TEST(MaterialTest, Accessors)
{
  {
    Material mat("Aluminum");
    Material mat1("aluminum");
    Material mat2(MaterialType::ALUMINUM);
    Material mat3(mat2);

    EXPECT_DOUBLE_EQ(2700.0, mat.Density());
    EXPECT_EQ(mat, mat1);
    EXPECT_EQ(mat1, mat2);
    EXPECT_EQ(mat2, mat3);

    // Test move constructor
    Material mat4(std::move(mat3));
    EXPECT_EQ(mat2, mat4);
    Material defaultMat;
    EXPECT_EQ(defaultMat, mat3);

    // Test move operator
    Material mat5;
    mat5 = std::move(mat4);
    EXPECT_EQ(mat2, mat5);
    EXPECT_EQ(defaultMat, mat4);
  }

  {
    Material mat("Notfoundium");
    EXPECT_GT(0.0, mat.Density());
    EXPECT_EQ(MaterialType::INVALID, mat.Type());
    EXPECT_TRUE(mat.Name().empty());
  }

  {
    Material material;
    material.SetToNearestDensity(19300.0);
    EXPECT_EQ(MaterialType::TUNGSTEN, material.Type());
    EXPECT_DOUBLE_EQ(19300.0, material.Density());
  }

  {
    Material material;
    material.SetToNearestDensity(1001001.001, 1e-3);
    EXPECT_EQ(MaterialType::INVALID, material.Type());
    EXPECT_GT(0.0, material.Density());
  }
  {
    Material material;
    material.SetToNearestDensity(1001001.001);
    EXPECT_EQ(MaterialType::TUNGSTEN, material.Type());
    EXPECT_DOUBLE_EQ(19300, material.Density());
  }
}
