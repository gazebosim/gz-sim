/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "ignition/math/AxisAlignedBox.hh"

using namespace ignition;
using namespace math;

class myAxisAlignedBox : public AxisAlignedBox
{
  public: myAxisAlignedBox()
          : AxisAlignedBox()
  {}
};

class ExampleAxisAlignedBox : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
       box = AxisAlignedBox(Vector3d(0, -1, 2), Vector3d(1, -2, 3));
    }

    AxisAlignedBox box;
};

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, Inherit)
{
  myAxisAlignedBox *box = NULL;

  {
    box = new myAxisAlignedBox();
    EXPECT_TRUE(box != NULL);
  }

  EXPECT_TRUE(box->Min() == Vector3d(MAX_D, MAX_D,
      MAX_D));
  EXPECT_TRUE(box->Max() == Vector3d(LOW_D, LOW_D,
      LOW_D));

  {
    delete box;
    box = NULL;
  }
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, EmptyConstructorNew)
{
  AxisAlignedBox *box = NULL;

  {
    box = new AxisAlignedBox;
    EXPECT_TRUE(box != NULL);
  }

  EXPECT_TRUE(box->Min() == Vector3d(MAX_D, MAX_D,
      MAX_D));
  EXPECT_TRUE(box->Max() == Vector3d(LOW_D, LOW_D,
      LOW_D));

  {
    delete box;
    box = NULL;
  }
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, EmptyConstructor)
{
  AxisAlignedBox box;
  EXPECT_TRUE(box.Min() == Vector3d(MAX_D, MAX_D,
      MAX_D));
  EXPECT_TRUE(box.Max() == Vector3d(LOW_D, LOW_D,
      LOW_D));
}

/////////////////////////////////////////////////
TEST_F(ExampleAxisAlignedBox, Constructor)
{
  EXPECT_EQ(box.Min(), Vector3d(0, -2, 2));
  EXPECT_EQ(box.Max(), Vector3d(1, -1, 3));
}

/////////////////////////////////////////////////
TEST_F(ExampleAxisAlignedBox, CopyConstructor)
{
  AxisAlignedBox box1(box);
  EXPECT_TRUE(box1.Min() == box.Min());
  EXPECT_TRUE(box1.Max() == box.Max());
}

/////////////////////////////////////////////////
TEST_F(ExampleAxisAlignedBox, ManuallySet)
{
  AxisAlignedBox box1;
  box1.Min().Set(-2, 2, 3);
  box1.Max().Set(0, 0, 3);

  box1 += box;
  EXPECT_DOUBLE_EQ(box1.Min().X(), -2);
  EXPECT_DOUBLE_EQ(box1.Min().Y(), -2);
  EXPECT_DOUBLE_EQ(box1.Min().Z(),  2);

  EXPECT_DOUBLE_EQ(box1.Max().X(), 1);
  EXPECT_DOUBLE_EQ(box1.Max().Y(), 0);
  EXPECT_DOUBLE_EQ(box1.Max().Z(), 3);

  AxisAlignedBox box2;
  box2.Max().Set(1, 1, 1);
  EXPECT_DOUBLE_EQ(box2.Size().X(), 0);
  EXPECT_DOUBLE_EQ(box2.Size().Y(), 0);
  EXPECT_DOUBLE_EQ(box2.Size().Z(), 0);

  AxisAlignedBox box3;
  box3.Min().Set(-1, -1, -1);
  EXPECT_DOUBLE_EQ(box3.Size().X(), 0);
  EXPECT_DOUBLE_EQ(box3.Size().Y(), 0);
  EXPECT_DOUBLE_EQ(box3.Size().Z(), 0);
}

/////////////////////////////////////////////////
TEST_F(ExampleAxisAlignedBox, Length)
{
  EXPECT_DOUBLE_EQ(box.XLength(), 1);
  EXPECT_DOUBLE_EQ(box.YLength(), 1);
  EXPECT_DOUBLE_EQ(box.ZLength(), 1);
}

/////////////////////////////////////////////////
TEST_F(ExampleAxisAlignedBox, Size)
{
  EXPECT_TRUE(box.Size() == Vector3d(1, 1, 1));
}

/////////////////////////////////////////////////
TEST_F(ExampleAxisAlignedBox, Center)
{
  EXPECT_TRUE(box.Center() == Vector3d(0.5, -1.5, 2.5));
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, MergeEmpty)
{
  AxisAlignedBox box1;
  AxisAlignedBox box2;

  box1.Merge(box2);
  EXPECT_DOUBLE_EQ(box1.Min().X(), MAX_D);
  EXPECT_DOUBLE_EQ(box1.Min().Y(), MAX_D);
  EXPECT_DOUBLE_EQ(box1.Min().Z(), MAX_D);

  EXPECT_DOUBLE_EQ(box1.Max().X(), LOW_D);
  EXPECT_DOUBLE_EQ(box1.Max().Y(), LOW_D);
  EXPECT_DOUBLE_EQ(box1.Max().Z(), LOW_D);
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, DefaultConstructor)
{
  AxisAlignedBox defaultAxisAlignedBox1, defaultAxisAlignedBox2;

  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.Size().X(), 0);
  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.Size().Y(), 0);
  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.Size().Z(), 0);

  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.XLength(), 0);
  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.YLength(), 0);
  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.ZLength(), 0);

  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.Center().X(), 0);
  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.Center().Y(), 0);
  EXPECT_DOUBLE_EQ(defaultAxisAlignedBox1.Center().Z(), 0);

  EXPECT_FALSE(defaultAxisAlignedBox1.Intersects(defaultAxisAlignedBox2));

  EXPECT_FALSE(defaultAxisAlignedBox1.Intersects(AxisAlignedBox(
          Vector3d(0, 0, 0), Vector3d(1, 1, 1))));

  EXPECT_FALSE(defaultAxisAlignedBox2.Contains(Vector3d::Zero));
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, Minus)
{
  AxisAlignedBox box1(1, 2, 3, 4, 5, 6);
  Vector3d sub(1, 1, 1);

  AxisAlignedBox box2 = box1 - sub;
  EXPECT_EQ(box2.Min(), box1.Min() - sub);
  EXPECT_EQ(box2.Max(), box1.Max() - sub);
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, PlusEmpty)
{
  AxisAlignedBox box1;
  AxisAlignedBox box2;

  box1 += box2;
  EXPECT_DOUBLE_EQ(box1.Min().X(), MAX_D);
  EXPECT_DOUBLE_EQ(box1.Min().Y(), MAX_D);
  EXPECT_DOUBLE_EQ(box1.Min().Z(), MAX_D);

  EXPECT_DOUBLE_EQ(box1.Max().X(), LOW_D);
  EXPECT_DOUBLE_EQ(box1.Max().Y(), LOW_D);
  EXPECT_DOUBLE_EQ(box1.Max().Z(), LOW_D);

  AxisAlignedBox box3 = box2 + box1;
  EXPECT_DOUBLE_EQ(box3.Min().X(), MAX_D);
  EXPECT_DOUBLE_EQ(box3.Min().Y(), MAX_D);
  EXPECT_DOUBLE_EQ(box3.Min().Z(), MAX_D);

  EXPECT_DOUBLE_EQ(box3.Max().X(), LOW_D);
  EXPECT_DOUBLE_EQ(box3.Max().Y(), LOW_D);
  EXPECT_DOUBLE_EQ(box3.Max().Z(), LOW_D);
}

/////////////////////////////////////////////////
TEST_F(ExampleAxisAlignedBox, Merge)
{
  box.Merge(AxisAlignedBox(Vector3d(-1, -1, -1), Vector3d(2, 2, 2)));
  EXPECT_TRUE(box == AxisAlignedBox(Vector3d(-1, -2, -1),
                               Vector3d(2, 2, 3)));
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, OperatorEqual)
{
  AxisAlignedBox box = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3));
  AxisAlignedBox box2 = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(1, 3, 3));
  AxisAlignedBox box3 = AxisAlignedBox(Vector3d(0, 1, 1), Vector3d(1, 3, 3));
  EXPECT_TRUE(box == AxisAlignedBox(Vector3d(1, 1, 1),
        Vector3d(3, 3, 3)));
  EXPECT_FALSE(box == box2);
  EXPECT_FALSE(box3 == box);
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, OperatorNotEqual)
{
  AxisAlignedBox box = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3));
  AxisAlignedBox box2 = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(1, 3, 3));
  AxisAlignedBox box3 = AxisAlignedBox(Vector3d(0, 1, 1), Vector3d(1, 3, 3));
  EXPECT_FALSE(box != AxisAlignedBox(Vector3d(1, 1, 1),
        Vector3d(3, 3, 3)));
  EXPECT_TRUE(box != box2);
  EXPECT_TRUE(box3 != box);
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, OperatorPlusEqual)
{
  AxisAlignedBox box = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(3, 3, 3));
  box += AxisAlignedBox(Vector3d(2, 2, 2), Vector3d(4, 4, 4));
  EXPECT_TRUE(box == AxisAlignedBox(Vector3d(1, 1, 1),
        Vector3d(4, 4, 4)));
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, OperatorPlus)
{
  AxisAlignedBox box = AxisAlignedBox(Vector3d(1, 1, 1), Vector3d(4, 4, 4));
  box = box + AxisAlignedBox(Vector3d(-2, -2, -2), Vector3d(4, 4, 4));
  EXPECT_TRUE(box == AxisAlignedBox(Vector3d(-2, -2, -2),
                               Vector3d(4, 4, 4)));
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, Intersects)
{
  AxisAlignedBox box = AxisAlignedBox(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

  EXPECT_FALSE(box.Intersects(AxisAlignedBox(
          Vector3d(1.1, 0, 0), Vector3d(2, 1, 1))));

  EXPECT_FALSE(box.Intersects(AxisAlignedBox(
          Vector3d(0, 1.1, 0), Vector3d(1, 2, 1))));

  EXPECT_FALSE(box.Intersects(AxisAlignedBox(
          Vector3d(0, 0, 1.1), Vector3d(1, 1, 2))));


  EXPECT_FALSE(box.Intersects(AxisAlignedBox(
          Vector3d(-1, -1, -1), Vector3d(-0.1, 0, 0))));

  EXPECT_FALSE(box.Intersects(AxisAlignedBox(
          Vector3d(-1, -1, -1), Vector3d(0, -0.1, 0))));

  EXPECT_FALSE(box.Intersects(AxisAlignedBox(
          Vector3d(-1, -1, -1), Vector3d(0, 0, -0.1))));

  EXPECT_TRUE(box.Intersects(AxisAlignedBox(
          Vector3d(0, 0, 0), Vector3d(1, 1, 1))));
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, Contains)
{
  AxisAlignedBox box = AxisAlignedBox(Vector3d(0, 0, 0), Vector3d(1, 1, 1));

  EXPECT_TRUE(box.Contains(Vector3d(0, 0, 0)));
  EXPECT_TRUE(box.Contains(Vector3d(0, 0, 1)));
  EXPECT_TRUE(box.Contains(Vector3d(0, 1, 1)));
  EXPECT_TRUE(box.Contains(Vector3d(1, 1, 1)));
  EXPECT_TRUE(box.Contains(Vector3d(1, 1, 0)));
  EXPECT_TRUE(box.Contains(Vector3d(1, 0, 0)));
  EXPECT_TRUE(box.Contains(Vector3d(0.5, 0.5, 0.5)));

  EXPECT_FALSE(box.Contains(Vector3d(0, 0, -1)));
  EXPECT_FALSE(box.Contains(Vector3d(0, -1, -1)));
  EXPECT_FALSE(box.Contains(Vector3d(-1, -1, -1)));
  EXPECT_FALSE(box.Contains(Vector3d(-1, -1, 0)));
  EXPECT_FALSE(box.Contains(Vector3d(-1, 0, 0)));

  EXPECT_FALSE(box.Contains(Vector3d(0.5, 0.5, -0.5)));
  EXPECT_FALSE(box.Contains(Vector3d(0.5, -0.5, 0.5)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5, 0.5, 0.5)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5, -0.5, 0.5)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5, -0.5, -0.5)));

  EXPECT_FALSE(box.Contains(Vector3d(0, 0, -0.01)));
  EXPECT_FALSE(box.Contains(Vector3d(0, -0.01, 0)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.01, 0, 0)));
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, OperatorStreamOut)
{
  AxisAlignedBox b(0.1, 1.2, 2.3, 1.1, 2.2, 4.3);
  std::ostringstream stream;
  stream << b;
  EXPECT_EQ(stream.str(), "Min[0.1 1.2 2.3] Max[1.1 2.2 4.3]");
}

/////////////////////////////////////////////////
TEST(AxisAlignedBoxTest, Intersect)
{
  AxisAlignedBox b(0, 0, 0, 1, 1, 1);

  bool intersect = false;
  double dist = 0;
  Vector3d pt;

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(-1, 0, 0),
      Vector3d(1, 0, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(-1, 0, 0),
      Vector3d(1, 0, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 1);
  EXPECT_DOUBLE_EQ(std::get<1>(b.Intersect(Vector3d(-1, 0, 0),
          Vector3d(1, 0, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(1, 0, 0),
      Vector3d(-1, 0, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(1, 0, 0),
      Vector3d(-1, 0, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(1, 0, 0),
      Vector3d(-1, 0, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d(1, 0, 0));

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(2, 2, 0),
      Vector3d(-1, -1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(2, 2, 0),
      Vector3d(-1, -1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, IGN_SQRT2);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(2, 2, 0),
      Vector3d(-1, -1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d(1, 1, 0));

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(-10, -10, 0),
      Vector3d(1, 1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(-10, -10, 0),
      Vector3d(1, 1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, std::sqrt(200));
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(-10, -10, 0),
      Vector3d(1, 1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(-1, -2, 0),
      Vector3d(1, 1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(-1, -2, 0),
      Vector3d(1, 1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 2*IGN_SQRT2);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(-1, -2, 0),
      Vector3d(1, 1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d(1, 0, 0));

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(2, 1, 0),
      Vector3d(-1, -1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(2, 1, 0),
      Vector3d(-1, -1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, IGN_SQRT2);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(2, 1, 0),
      Vector3d(-1, -1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d(1, 0, 0));

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0.5, 0.5, 2),
      Vector3d(0, 0, -1), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(0.5, 0.5, 2),
      Vector3d(0, 0, -1), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 1);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(0.5, 0.5, 2),
      Vector3d(0, 0, -1), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d(0.5, 0.5, 1));

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0.5, 0.5, 2),
      Vector3d(0, 0, 1), 0, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(Vector3d(0.5, 0.5, 2),
      Vector3d(0, 0, 1), 0, 1000));
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(-1, -1, 1),
      Vector3d(0, 0, -1), 0, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(Vector3d(-1, -1, 1),
      Vector3d(0, 0, -1), 0, 1000));
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(2, 2, 0),
      Vector3d(1, 1, 0), 0, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(Vector3d(2, 2, 0),
      Vector3d(1, 1, 0), 0, 1000));
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(2, 2, 0),
      Vector3d(0, 1, 0), 0, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(Vector3d(2, 2, 0),
      Vector3d(0, 1, 0), 0, 1000));
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0.1, 0.1, 200),
      Vector3d(0, 0, -1), 0, 100);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(Vector3d(0.1, 0.1, 200),
      Vector3d(0, 0, -1), 0, 100));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(0.1, 0.1, 200),
      Vector3d(0, 0, -1), 0, 100)), dist);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0.1, 0.1, 1),
      Vector3d(0, 0, -1), 1.0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(0.1, 0.1, 1),
      Vector3d(0, 0, -1), 1.0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0.0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(0.1, 0.1, 1),
      Vector3d(0, 0, -1), 1.0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d(0.1, 0.1, 0));

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0.1, 0.1, 1),
      Vector3d(0, 0, -1), 1.1, 1000);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(Vector3d(0.1, 0.1, 1),
      Vector3d(0, 0, -1), 1.1, 1000));
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0.1, 0.1, 10),
      Vector3d(0, 0, -1), 1.1, 5);
  EXPECT_FALSE(intersect);
  EXPECT_FALSE(b.IntersectCheck(Vector3d(0.1, 0.1, 10),
      Vector3d(0, 0, -1), 1.1, 5));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(0.1, 0.1, 10),
      Vector3d(0, 0, -1), 1.1, 5)), dist);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(
      Line3d(Vector3d(4, 0, 0.5), Vector3d(0, 10, 0.5)));
  EXPECT_FALSE(intersect);
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(
      Line3d(Vector3d(1, -1, 1.5), Vector3d(0, 1, 1.5)));
  EXPECT_FALSE(intersect);
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0, 0, 1),
      Vector3d(0, 0, -1), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(0, 0, 1),
      Vector3d(0, 0, -1), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(0, 0, 1),
      Vector3d(0, 0, -1), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d(0, 0, 1));

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0, 0, 0),
      Vector3d(1, 0, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(0, 0, 0),
      Vector3d(1, 0, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(0, 0, 0),
      Vector3d(1, 0, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0, 0, 0),
      Vector3d(-1, 0, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(0, 0, 0),
      Vector3d(-1, 0, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(0, 0, 0),
      Vector3d(-1, 0, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0, 0, 0),
      Vector3d(0, 1, 0), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(0, 0, 0),
      Vector3d(0, 1, 0), 0, 1000));
  EXPECT_DOUBLE_EQ(dist, 0);
  EXPECT_DOUBLE_EQ(std::get<1>(b.IntersectDist(Vector3d(0, 0, 0),
      Vector3d(0, 1, 0), 0, 1000)), dist);
  EXPECT_EQ(pt, Vector3d::Zero);

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(0.5, 0.5, 0.5),
      Vector3d(-.707107, 0, -0.707107), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(0.5, 0.5, 0.5),
      Vector3d(-.707107, 0, -0.707107), 0, 1000));
  EXPECT_NEAR(dist, 0, 1e-5);
  EXPECT_NEAR(std::get<1>(b.Intersect(Vector3d(0.5, 0.5, 0.5),
      Vector3d(-.707107, 0, -0.707107), 0, 1000)), dist, 1e-5);
  EXPECT_EQ(pt, Vector3d(0.5, 0.5, 0.5));

  std::tie(intersect, dist, pt) = b.Intersect(Vector3d(1.2, 0, 0.5),
      Vector3d(-0.707107, 0, -0.707107), 0, 1000);
  EXPECT_TRUE(intersect);
  EXPECT_TRUE(b.IntersectCheck(Vector3d(1.2, 0, 0.5),
      Vector3d(-0.707107, 0, -0.707107), 0, 1000));
  EXPECT_NEAR(dist, 0.28284, 1e-5);
  EXPECT_NEAR(std::get<1>(b.Intersect(Vector3d(1.2, 0, 0.5),
      Vector3d(-0.707107, 0, -0.707107), 0, 1000)), dist, 1e-5);
  EXPECT_EQ(pt, Vector3d(1, 0, 0.3));
}
