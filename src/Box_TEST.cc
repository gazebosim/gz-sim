/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "ignition/math/Box.hh"

using namespace ignition;

class myBox : public math::Box
{
  public: myBox()
          : math::Box()
  {}
};

class ExampleBox : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
       box = math::Box(math::Vector3d(0, 1, 2), math::Vector3d(1, 2, 3));
    }

    math::Box box;
};

/////////////////////////////////////////////////
TEST(BoxTest, Inherit)
{
  myBox *box = NULL;

  {
    box = new myBox();
    EXPECT_TRUE(box != NULL);
  }

  EXPECT_TRUE(box->min == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(box->max == math::Vector3d(0, 0, 0));

  {
    delete box;
    box = NULL;
  }
}

/////////////////////////////////////////////////
TEST(BoxTest, EmptyConstructorNew)
{
  math::Box *box = NULL;

  {
    box = new math::Box;
    EXPECT_TRUE(box != NULL);
  }

  EXPECT_TRUE(box->min == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(box->max == math::Vector3d(0, 0, 0));

  {
    delete box;
    box = NULL;
  }
}

/////////////////////////////////////////////////
TEST(BoxTest, EmptyConstructor)
{
  math::Box box;
  EXPECT_TRUE(box.min == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(box.max == math::Vector3d(0, 0, 0));
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, Constructor)
{
  EXPECT_TRUE(box.min == math::Vector3d(0, 1, 2));
  EXPECT_TRUE(box.max == math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, CopyConstructor)
{
  math::Box box1(box);
  EXPECT_TRUE(box1.min == box.min);
  EXPECT_TRUE(box1.max == box.max);
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, GetLength)
{
  EXPECT_DOUBLE_EQ(box.GetXLength(), 1);
  EXPECT_DOUBLE_EQ(box.GetYLength(), 1);
  EXPECT_DOUBLE_EQ(box.GetZLength(), 1);
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, GetSize)
{
  EXPECT_TRUE(box.GetSize() == math::Vector3d(1, 1, 1));
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, GetCenter)
{
  EXPECT_TRUE(box.GetCenter() == math::Vector3d(0.5, 1.5, 2.5));
}

/////////////////////////////////////////////////
TEST(BoxTest, MergeEmpty)
{
  math::Box box1;
  math::Box box2;

  box1.Merge(box2);
  EXPECT_NEAR(box1.min.x(), 0, 1e-6);
  EXPECT_NEAR(box1.min.y(), 0, 1e-6);
  EXPECT_NEAR(box1.min.z(), 0, 1e-6);

  EXPECT_NEAR(box1.max.x(), 0, 1e-6);
  EXPECT_NEAR(box1.max.y(), 0, 1e-6);
  EXPECT_NEAR(box1.max.z(), 0, 1e-6);
}

/////////////////////////////////////////////////
TEST(BoxTest, Minus)
{
  math::Box box1(1, 2, 3, 4, 5, 6);
  math::Vector3d sub(1, 1, 1);

  math::Box box2 = box1 - sub;
  EXPECT_EQ(box2.min, box1.min - sub);
  EXPECT_EQ(box2.max, box1.max - sub);
}

/////////////////////////////////////////////////
TEST(BoxTest, PlusEmpty)
{
  math::Box box1;
  math::Box box2;

  box1 += box2;
  EXPECT_NEAR(box1.min.x(), 0, 1e-6);
  EXPECT_NEAR(box1.min.y(), 0, 1e-6);
  EXPECT_NEAR(box1.min.z(), 0, 1e-6);

  EXPECT_NEAR(box1.max.x(), 0, 1e-6);
  EXPECT_NEAR(box1.max.y(), 0, 1e-6);
  EXPECT_NEAR(box1.max.z(), 0, 1e-6);

  math::Box box3 = box2 + box1;
  EXPECT_NEAR(box3.min.x(), 0, 1e-6);
  EXPECT_NEAR(box3.min.y(), 0, 1e-6);
  EXPECT_NEAR(box3.min.z(), 0, 1e-6);

  EXPECT_NEAR(box3.max.x(), 0, 1e-6);
  EXPECT_NEAR(box3.max.y(), 0, 1e-6);
  EXPECT_NEAR(box3.max.z(), 0, 1e-6);
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, Merge)
{
  box.Merge(math::Box(math::Vector3d(-1, -1, -1), math::Vector3d(2, 2, 2)));
  EXPECT_TRUE(box == math::Box(math::Vector3d(-1, -1, -1),
                               math::Vector3d(2, 2, 3)));
}

/////////////////////////////////////////////////
TEST(BoxTest, OperatorEqual)
{
  math::Box box = math::Box(math::Vector3d(1, 1, 1), math::Vector3d(3, 3, 3));
  math::Box box2 = math::Box(math::Vector3d(1, 1, 1), math::Vector3d(1, 3, 3));
  math::Box box3 = math::Box(math::Vector3d(0, 1, 1), math::Vector3d(1, 3, 3));
  EXPECT_TRUE(box == math::Box(math::Vector3d(1, 1, 1),
        math::Vector3d(3, 3, 3)));
  EXPECT_FALSE(box == box2);
  EXPECT_FALSE(box3 == box);
}

/////////////////////////////////////////////////
TEST(BoxTest, OperatorPlusEqual)
{
  math::Box box = math::Box(math::Vector3d(1, 1, 1), math::Vector3d(3, 3, 3));
  box += math::Box(math::Vector3d(2, 2, 2), math::Vector3d(4, 4, 4));
  EXPECT_TRUE(box == math::Box(math::Vector3d(1, 1, 1),
        math::Vector3d(4, 4, 4)));
}

/////////////////////////////////////////////////
TEST(BoxTest, OperatorPlus)
{
  math::Box box = math::Box(math::Vector3d(1, 1, 1), math::Vector3d(4, 4, 4));
  box = box + math::Box(math::Vector3d(-2, -2, -2), math::Vector3d(4, 4, 4));
  EXPECT_TRUE(box == math::Box(math::Vector3d(-2, -2, -2),
                               math::Vector3d(4, 4, 4)));
}
