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
#include <cmath>

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
       box = math::Box(math::Vector3d(0, -1, 2), math::Vector3d(1, -2, 3));
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

  EXPECT_TRUE(box->Min() == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(box->Max() == math::Vector3d(0, 0, 0));

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

  EXPECT_TRUE(box->Min() == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(box->Max() == math::Vector3d(0, 0, 0));

  {
    delete box;
    box = NULL;
  }
}

/////////////////////////////////////////////////
TEST(BoxTest, EmptyConstructor)
{
  math::Box box;
  EXPECT_TRUE(box.Min() == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(box.Max() == math::Vector3d(0, 0, 0));
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, Constructor)
{
  EXPECT_EQ(box.Min(), math::Vector3d(0, -2, 2));
  EXPECT_EQ(box.Max(), math::Vector3d(1, -1, 3));
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, CopyConstructor)
{
  math::Box box1(box);
  EXPECT_TRUE(box1.Min() == box.Min());
  EXPECT_TRUE(box1.Max() == box.Max());
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, Length)
{
  EXPECT_DOUBLE_EQ(box.XLength(), 1);
  EXPECT_DOUBLE_EQ(box.YLength(), 1);
  EXPECT_DOUBLE_EQ(box.ZLength(), 1);
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, Size)
{
  EXPECT_TRUE(box.Size() == math::Vector3d(1, 1, 1));
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, Center)
{
  EXPECT_TRUE(box.Center() == math::Vector3d(0.5, -1.5, 2.5));
}

/////////////////////////////////////////////////
TEST(BoxTest, MergeEmpty)
{
  math::Box box1;
  math::Box box2;

  box1.Merge(box2);
  EXPECT_NEAR(box1.Min().X(), 0, 1e-6);
  EXPECT_NEAR(box1.Min().Y(), 0, 1e-6);
  EXPECT_NEAR(box1.Min().Z(), 0, 1e-6);

  EXPECT_NEAR(box1.Max().X(), 0, 1e-6);
  EXPECT_NEAR(box1.Max().Y(), 0, 1e-6);
  EXPECT_NEAR(box1.Max().Z(), 0, 1e-6);
}

/////////////////////////////////////////////////
TEST(BoxTest, Minus)
{
  math::Box box1(1, 2, 3, 4, 5, 6);
  math::Vector3d sub(1, 1, 1);

  math::Box box2 = box1 - sub;
  EXPECT_EQ(box2.Min(), box1.Min() - sub);
  EXPECT_EQ(box2.Max(), box1.Max() - sub);
}

/////////////////////////////////////////////////
TEST(BoxTest, PlusEmpty)
{
  math::Box box1;
  math::Box box2;

  box1 += box2;
  EXPECT_NEAR(box1.Min().X(), 0, 1e-6);
  EXPECT_NEAR(box1.Min().Y(), 0, 1e-6);
  EXPECT_NEAR(box1.Min().Z(), 0, 1e-6);

  EXPECT_NEAR(box1.Max().X(), 0, 1e-6);
  EXPECT_NEAR(box1.Max().Y(), 0, 1e-6);
  EXPECT_NEAR(box1.Max().Z(), 0, 1e-6);

  math::Box box3 = box2 + box1;
  EXPECT_NEAR(box3.Min().X(), 0, 1e-6);
  EXPECT_NEAR(box3.Min().Y(), 0, 1e-6);
  EXPECT_NEAR(box3.Min().Z(), 0, 1e-6);

  EXPECT_NEAR(box3.Max().X(), 0, 1e-6);
  EXPECT_NEAR(box3.Max().Y(), 0, 1e-6);
  EXPECT_NEAR(box3.Max().Z(), 0, 1e-6);
}

/////////////////////////////////////////////////
TEST_F(ExampleBox, Merge)
{
  box.Merge(math::Box(math::Vector3d(-1, -1, -1), math::Vector3d(2, 2, 2)));
  EXPECT_TRUE(box == math::Box(math::Vector3d(-1, -2, -1),
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
TEST(BoxTest, OperatorNotEqual)
{
  math::Box box = math::Box(math::Vector3d(1, 1, 1), math::Vector3d(3, 3, 3));
  math::Box box2 = math::Box(math::Vector3d(1, 1, 1), math::Vector3d(1, 3, 3));
  math::Box box3 = math::Box(math::Vector3d(0, 1, 1), math::Vector3d(1, 3, 3));
  EXPECT_FALSE(box != math::Box(math::Vector3d(1, 1, 1),
        math::Vector3d(3, 3, 3)));
  EXPECT_TRUE(box != box2);
  EXPECT_TRUE(box3 != box);
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

/////////////////////////////////////////////////
TEST(BoxTest, Intersects)
{
  math::Box box = math::Box(math::Vector3d(0, 0, 0), math::Vector3d(1, 1, 1));

  EXPECT_FALSE(box.Intersects(math::Box(
          math::Vector3d(1.1, 0, 0), math::Vector3d(2, 1, 1))));

  EXPECT_FALSE(box.Intersects(math::Box(
          math::Vector3d(0, 1.1, 0), math::Vector3d(1, 2, 1))));

  EXPECT_FALSE(box.Intersects(math::Box(
          math::Vector3d(0, 0, 1.1), math::Vector3d(1, 1, 2))));


  EXPECT_FALSE(box.Intersects(math::Box(
          math::Vector3d(-1, -1, -1), math::Vector3d(-0.1, 0, 0))));

  EXPECT_FALSE(box.Intersects(math::Box(
          math::Vector3d(-1, -1, -1), math::Vector3d(0, -0.1, 0))));

  EXPECT_FALSE(box.Intersects(math::Box(
          math::Vector3d(-1, -1, -1), math::Vector3d(0, 0, -0.1))));

  EXPECT_TRUE(box.Intersects(math::Box(
          math::Vector3d(0, 0, 0), math::Vector3d(1, 1, 1))));
}

/////////////////////////////////////////////////
TEST(BoxTest, Contains)
{
  math::Box box = math::Box(math::Vector3d(0, 0, 0), math::Vector3d(1, 1, 1));

  EXPECT_TRUE(box.Contains(math::Vector3d(0, 0, 0)));
  EXPECT_TRUE(box.Contains(math::Vector3d(0, 0, 1)));
  EXPECT_TRUE(box.Contains(math::Vector3d(0, 1, 1)));
  EXPECT_TRUE(box.Contains(math::Vector3d(1, 1, 1)));
  EXPECT_TRUE(box.Contains(math::Vector3d(1, 1, 0)));
  EXPECT_TRUE(box.Contains(math::Vector3d(1, 0, 0)));
  EXPECT_TRUE(box.Contains(math::Vector3d(0.5, 0.5, 0.5)));

  EXPECT_FALSE(box.Contains(math::Vector3d(0, 0, -1)));
  EXPECT_FALSE(box.Contains(math::Vector3d(0, -1, -1)));
  EXPECT_FALSE(box.Contains(math::Vector3d(-1, -1, -1)));
  EXPECT_FALSE(box.Contains(math::Vector3d(-1, -1, 0)));
  EXPECT_FALSE(box.Contains(math::Vector3d(-1, 0, 0)));

  EXPECT_FALSE(box.Contains(math::Vector3d(0.5, 0.5, -0.5)));
  EXPECT_FALSE(box.Contains(math::Vector3d(0.5, -0.5, 0.5)));
  EXPECT_FALSE(box.Contains(math::Vector3d(-0.5, 0.5, 0.5)));
  EXPECT_FALSE(box.Contains(math::Vector3d(-0.5, -0.5, 0.5)));
  EXPECT_FALSE(box.Contains(math::Vector3d(-0.5, -0.5, -0.5)));

  EXPECT_FALSE(box.Contains(math::Vector3d(0, 0, -0.01)));
  EXPECT_FALSE(box.Contains(math::Vector3d(0, -0.01, 0)));
  EXPECT_FALSE(box.Contains(math::Vector3d(-0.01, 0, 0)));
}

/////////////////////////////////////////////////
TEST(BoxTest, OperatorStreamOut)
{
  math::Box b(0.1, 1.2, 2.3, 1.1, 2.2, 4.3);
  std::ostringstream stream;
  stream << b;
  EXPECT_EQ(stream.str(), "Min[0.1 1.2 2.3] Max[1.1 2.2 4.3]");
}

/////////////////////////////////////////////////
TEST(BoxTest, Intersect)
{
  math::Box b(0, 0, 0, 1, 1, 1);

  bool intersect = false;
  double dist = 0;
  math::Vector3d pt;

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(-1, 0, 0),
      math::Vector3d(1, 0, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(-1, 0, 0),
      math::Vector3d(1, 0, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 1);
  EXPECT_DOUBLE_EQ(std::get<1>(b.Intersect(math::Vector3d(-1, 0, 0),
          math::Vector3d(1, 0, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(1, 0, 0),
      math::Vector3d(-1, 0, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(1, 0, 0),
      math::Vector3d(-1, 0, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(1, 0, 0),
      math::Vector3d(-1, 0, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d(1, 0, 0));

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(2, 2, 0),
      math::Vector3d(-1, -1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(2, 2, 0),
      math::Vector3d(-1, -1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, IGN_SQRT2);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(2, 2, 0),
      math::Vector3d(-1, -1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d(1, 1, 0));

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(-10, -10, 0),
      math::Vector3d(1, 1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(-10, -10, 0),
      math::Vector3d(1, 1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, std::sqrt(200));
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(-10, -10, 0),
      math::Vector3d(1, 1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(-1, -2, 0),
      math::Vector3d(1, 1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(-1, -2, 0),
      math::Vector3d(1, 1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 2*IGN_SQRT2);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(-1, -2, 0),
      math::Vector3d(1, 1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d(1, 0, 0));

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(2, 1, 0),
      math::Vector3d(-1, -1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(2, 1, 0),
      math::Vector3d(-1, -1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, IGN_SQRT2);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(2, 1, 0),
      math::Vector3d(-1, -1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d(1, 0, 0));

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0.5, 0.5, 2),
      math::Vector3d(0, 0, -1), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(0.5, 0.5, 2),
      math::Vector3d(0, 0, -1), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 1);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(0.5, 0.5, 2),
      math::Vector3d(0, 0, -1), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d(0.5, 0.5, 1));

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0.5, 0.5, 2),
      math::Vector3d(0, 0, 1), 0, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(math::Vector3d(0.5, 0.5, 2),
      math::Vector3d(0, 0, 1), 0, 1000));
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(-1, -1, 1),
      math::Vector3d(0, 0, -1), 0, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(math::Vector3d(-1, -1, 1),
      math::Vector3d(0, 0, -1), 0, 1000));
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(2, 2, 0),
      math::Vector3d(1, 1, 0), 0, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(math::Vector3d(2, 2, 0),
      math::Vector3d(1, 1, 0), 0, 1000));
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(2, 2, 0),
      math::Vector3d(0, 1, 0), 0, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(math::Vector3d(2, 2, 0),
      math::Vector3d(0, 1, 0), 0, 1000));
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0.1, 0.1, 200),
      math::Vector3d(0, 0, -1), 0, 100);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(math::Vector3d(0.1, 0.1, 200),
      math::Vector3d(0, 0, -1), 0, 100));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(0.1, 0.1, 200),
      math::Vector3d(0, 0, -1), 0, 100)), dist);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0.1, 0.1, 1),
      math::Vector3d(0, 0, -1), 1.0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(0.1, 0.1, 1),
      math::Vector3d(0, 0, -1), 1.0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0.0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(0.1, 0.1, 1),
      math::Vector3d(0, 0, -1), 1.0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d(0.1, 0.1, 0));

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0.1, 0.1, 1),
      math::Vector3d(0, 0, -1), 1.1, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(math::Vector3d(0.1, 0.1, 1),
      math::Vector3d(0, 0, -1), 1.1, 1000));
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0.1, 0.1, 10),
      math::Vector3d(0, 0, -1), 1.1, 5);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(math::Vector3d(0.1, 0.1, 10),
      math::Vector3d(0, 0, -1), 1.1, 5));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(0.1, 0.1, 10),
      math::Vector3d(0, 0, -1), 1.1, 5)), dist);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(
      math::Line3d(math::Vector3d(4, 0, 0.5), math::Vector3d(0, 10, 0.5)));
  EXPECT_FALSE(intersect);
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(
      math::Line3d(math::Vector3d(1, -1, 1.5), math::Vector3d(0, 1, 1.5)));
  EXPECT_FALSE(intersect);
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0, 0, 1),
      math::Vector3d(0, 0, -1), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(0, 0, 1),
      math::Vector3d(0, 0, -1), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(0, 0, 1),
      math::Vector3d(0, 0, -1), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d(0, 0, 1));

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0, 0, 0),
      math::Vector3d(1, 0, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(0, 0, 0),
      math::Vector3d(1, 0, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(0, 0, 0),
      math::Vector3d(1, 0, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0, 0, 0),
      math::Vector3d(-1, 0, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(0, 0, 0),
      math::Vector3d(-1, 0, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(0, 0, 0),
      math::Vector3d(-1, 0, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0, 0, 0),
      math::Vector3d(0, 1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(0, 0, 0),
      math::Vector3d(0, 1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(math::Vector3d(0, 0, 0),
      math::Vector3d(0, 1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, math::Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(0.5, 0.5, 0.5),
      math::Vector3d(-.707107, 0, -0.707107), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(0.5, 0.5, 0.5),
      math::Vector3d(-.707107, 0, -0.707107), 0, 1000));
  EXPECT_NEAR(dist, 0, 1e-5);
  EXPECT_NEAR(std::get<1>(b.Intersect(math::Vector3d(0.5, 0.5, 0.5),
      math::Vector3d(-.707107, 0, -0.707107), 0, 1000)), dist, 1e-5);
  EXPECT_EQ(pt, math::Vector3d(0.5, 0.5, 0.5));

  std::tie(intersect, dist, pt) = b.Intersect(math::Vector3d(1.2, 0, 0.5),
      math::Vector3d(-0.707107, 0, -0.707107), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(math::Vector3d(1.2, 0, 0.5),
      math::Vector3d(-0.707107, 0, -0.707107), 0, 1000));
  EXPECT_NEAR(dist, 0.28284, 1e-5);
  EXPECT_NEAR(std::get<1>(b.Intersect(math::Vector3d(1.2, 0, 0.5),
      math::Vector3d(-0.707107, 0, -0.707107), 0, 1000)), dist, 1e-5);
  EXPECT_EQ(pt, math::Vector3d(1, 0, 0.3));
}
