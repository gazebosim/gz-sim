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

#include "ignition/gazebo/ComponentType.hh"

using namespace ignition;

class TestComponentType : public gazebo::ComponentType
{
  public: explicit TestComponentType(const gazebo::ComponentTypeId &_typeId)
          : ComponentType(_typeId)
  {
  }

  public: const std::string &Name() const override final
  {
    return this->name;
  }
  private: std::string name{"TestComponentType"};
};

/////////////////////////////////////////////////
TEST(ComponentType, Constructor)
{
  TestComponentType type(gazebo::kComponentTypeIdInvalid);
  EXPECT_EQ(gazebo::kComponentTypeIdInvalid, type.TypeId());
  EXPECT_EQ("TestComponentType", type.Name());
  EXPECT_FALSE(type.Valid());

  TestComponentType type2(1);
  EXPECT_EQ(1, type2.TypeId());
  EXPECT_TRUE(type2.Valid());
}
