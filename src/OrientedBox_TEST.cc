/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "ignition/math/Angle.hh"
#include "ignition/math/OrientedBox.hh"

using namespace ignition;
using namespace math;

auto g_tolerance = 1e-6;

/////////////////////////////////////////////////
TEST(OrientedBoxTest, EmptyConstructorNew)
{
  OrientedBoxd *box = nullptr;

  {
    box = new OrientedBoxd;
    EXPECT_TRUE(box != nullptr);
  }

  EXPECT_TRUE(box->Size() == Vector3d::Zero);
  EXPECT_TRUE(box->Pose() == Pose3d::Zero);
  EXPECT_EQ(math::Material(), box->Material());

  {
    delete box;
    box = nullptr;
  }
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, EmptyConstructor)
{
  OrientedBoxd box;
  EXPECT_TRUE(box.Size() == Vector3d::Zero);
  EXPECT_TRUE(box.Pose() == Pose3d::Zero);
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, SizeOnlyConstructor)
{
  OrientedBoxd box(Vector3d(1, 2, 3));
  EXPECT_EQ(box.Size(), Vector3d(1, 2, 3));
  EXPECT_EQ(box.Pose(), Pose3d::Zero);
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, NegativeSizeConstructor)
{
  OrientedBoxd box(Vector3d(-1, 0, -3));
  EXPECT_EQ(box.Size(), Vector3d(1, 0, 3));
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, SizePoseConstructor)
{
  OrientedBoxi box(Vector3i(1, 2, 3), Pose3i(-1, -2, -3, 0, 1, 2));
  EXPECT_EQ(box.Size(), Vector3i(1, 2, 3));
  EXPECT_EQ(box.Pose(), Pose3i(-1, -2, -3, 0, 1, 2));
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, CopyConstructor)
{
  OrientedBoxf box1(Vector3f(0.1f, 0.2f, 0.3f),
                     Pose3f(-0.1f, -0.2f, 0.0f, 1.1f, 1.2f, 1.3f));
  OrientedBoxf box2(box1);

  EXPECT_EQ(box2.Size(), Vector3f(0.1f, 0.2f, 0.3f));
  EXPECT_EQ(box2.Pose(), Pose3f(-0.1f, -0.2f, 0.0f, 1.1f, 1.2f, 1.3f));
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, Length)
{
  OrientedBoxd box(Vector3d(0.1, -2.1, 0.0));
  EXPECT_DOUBLE_EQ(box.XLength(), 0.1);
  EXPECT_DOUBLE_EQ(box.YLength(), 2.1);
  EXPECT_DOUBLE_EQ(box.ZLength(), 0.0);
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, OperatorEqual)
{
  OrientedBoxd box = OrientedBoxd(Vector3d(1, 1, 1));
  OrientedBoxd box2 = OrientedBoxd(Vector3d(1, 1, 1),
                                     Pose3d(1, 2, 3, 4, 5, 6));
  OrientedBoxd box3 = OrientedBoxd(Vector3d(0, 0, 0),
                                     Pose3d(1, 2, 3, 4, 5, 6));
  EXPECT_TRUE(box == OrientedBoxd(Vector3d(1, 1, 1)));
  EXPECT_FALSE(box == box2);
  EXPECT_FALSE(box3 == box);
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, ContainsZeroBox)
{
  OrientedBoxd box;

  EXPECT_TRUE(box.Contains(Vector3d(0, 0, 0)));
  EXPECT_FALSE(box.Contains(Vector3d(0, 0, 0.0001)));
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, ContainsZeroPose)
{
  OrientedBoxd box(Vector3d(1, 2, 3));

  // Vertices
  EXPECT_TRUE(box.Contains(Vector3d(-0.5, -1.0, -1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5, -1.0, +1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5, +1.0, -1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5, +1.0, +1.5)));

  EXPECT_TRUE(box.Contains(Vector3d(+0.5, -1.0, -1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5, -1.0, +1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5, +1.0, -1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5, +1.0, +1.5)));

  // Edges
  EXPECT_TRUE(box.Contains(Vector3d(0.0, -1.0, -1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(0.0, -1.0, +1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(0.0, +1.0, -1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(0.0, +1.0, +1.5)));

  EXPECT_TRUE(box.Contains(Vector3d(-0.5, -1.0, 0.0)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5, +1.0, 0.0)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5, -1.0, 0.0)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5, +1.0, 0.0)));

  EXPECT_TRUE(box.Contains(Vector3d(-0.5, 0.0, -1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5, 0.0, +1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5, 0.0, -1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5, 0.0, +1.5)));

  // Inside
  EXPECT_TRUE(box.Contains(Vector3d(-0.5+g_tolerance,
                                    -1.0+g_tolerance,
                                    -1.5+g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5+g_tolerance,
                                    -1.0+g_tolerance,
                                    +1.5-g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5+g_tolerance,
                                    +1.0-g_tolerance,
                                    -1.5+g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5+g_tolerance,
                                    +1.0-g_tolerance,
                                    +1.5-g_tolerance)));

  EXPECT_TRUE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    -1.0+g_tolerance,
                                    -1.5+g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    -1.0+g_tolerance,
                                    +1.5-g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    +1.0-g_tolerance,
                                    -1.5+g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    +1.0-g_tolerance,
                                    +1.5-g_tolerance)));

  // Outside
  EXPECT_FALSE(box.Contains(Vector3d(-0.5-g_tolerance,
                                    -1.0-g_tolerance,
                                    -1.5-g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5-g_tolerance,
                                    -1.0-g_tolerance,
                                    +1.5+g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5-g_tolerance,
                                    +1.0+g_tolerance,
                                    -1.5-g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5-g_tolerance,
                                    +1.0+g_tolerance,
                                    +1.5+g_tolerance)));

  EXPECT_FALSE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    -1.0-g_tolerance,
                                    -1.5-g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    -1.0-g_tolerance,
                                    +1.5+g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    +1.0+g_tolerance,
                                    -1.5-g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    +1.0+g_tolerance,
                                    +1.5+g_tolerance)));
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, ContainsOrientedPosition)
{
  OrientedBoxd box(Vector3d(1, 2, 3),
                    Pose3d(10, 20, 30, 0, 0, 0));

  // Vertices
  EXPECT_TRUE(box.Contains(Vector3d(10-0.5, 20-1.0, 30-1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10-0.5, 20-1.0, 30+1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10-0.5, 20+1.0, 30-1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10-0.5, 20+1.0, 30+1.5)));

  EXPECT_TRUE(box.Contains(Vector3d(10+0.5, 20-1.0, 30-1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10+0.5, 20-1.0, 30+1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10+0.5, 20+1.0, 30-1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10+0.5, 20+1.0, 30+1.5)));

  // Edges
  EXPECT_TRUE(box.Contains(Vector3d(10.0, 20-1.0, 30-1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10.0, 20-1.0, 30+1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10.0, 20+1.0, 30-1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10.0, 20+1.0, 30+1.5)));

  EXPECT_TRUE(box.Contains(Vector3d(10-0.5, 20-1.0, 30.0)));
  EXPECT_TRUE(box.Contains(Vector3d(10-0.5, 20+1.0, 30.0)));
  EXPECT_TRUE(box.Contains(Vector3d(10+0.5, 20-1.0, 30.0)));
  EXPECT_TRUE(box.Contains(Vector3d(10+0.5, 20+1.0, 30.0)));

  EXPECT_TRUE(box.Contains(Vector3d(10-0.5, 20.0, 30-1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10-0.5, 20.0, 30+1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10+0.5, 20.0, 30-1.5)));
  EXPECT_TRUE(box.Contains(Vector3d(10+0.5, 20.0, 30+1.5)));

  // Inside
  EXPECT_TRUE(box.Contains(Vector3d(10, 20, 30)));
  EXPECT_TRUE(box.Contains(Vector3d(10-0.25, 20-0.5, 30-0.75)));
  EXPECT_TRUE(box.Contains(Vector3d(10+0.25, 20+0.5, 30+0.75)));

  // Outside
  EXPECT_FALSE(box.Contains(Vector3d(10-1.0, 20-1.0, 30-1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(10-0.5, 20-2.0, 30-1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(10-0.5, 20-1.0, 30-2.0)));

  EXPECT_FALSE(box.Contains(Vector3d(10+1.0, 20+1.0, 30+1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(10+0.5, 20+2.0, 30+1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(10+0.5, 20+1.0, 30+2.0)));
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, ContainsOrientedRotation)
{
  // Rotate PI/2 about +x: swap Z and Y
  OrientedBoxd box(Vector3d(1, 2, 3), Pose3d(0, 0, 0, IGN_PI*0.5, 0, 0));

  // Doesn't contain non-rotated vertices
  EXPECT_FALSE(box.Contains(Vector3d(-0.5, -1.0, -1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5, -1.0, +1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5, +1.0, -1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5, +1.0, +1.5)));

  EXPECT_FALSE(box.Contains(Vector3d(+0.5, -1.0, -1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5, -1.0, +1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5, +1.0, -1.5)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5, +1.0, +1.5)));

  // Inside
  EXPECT_TRUE(box.Contains(Vector3d(-0.5+g_tolerance,
                                    -1.5+g_tolerance,
                                    -1.0+g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5+g_tolerance,
                                    -1.5+g_tolerance,
                                    +1.0-g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5+g_tolerance,
                                    +1.5-g_tolerance,
                                    -1.0+g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(-0.5+g_tolerance,
                                    +1.5-g_tolerance,
                                    +1.0-g_tolerance)));

  EXPECT_TRUE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    -1.5+g_tolerance,
                                    -1.0+g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    -1.5+g_tolerance,
                                    +1.0-g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    +1.5-g_tolerance, -1.0+g_tolerance)));
  EXPECT_TRUE(box.Contains(Vector3d(+0.5-g_tolerance,
                                    +1.5-g_tolerance,
                                    +1.0-g_tolerance)));

  // Outside
  EXPECT_FALSE(box.Contains(Vector3d(-0.5-g_tolerance,
                                     -1.5-g_tolerance,
                                     -1.0-g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5-g_tolerance,
                                     -1.5-g_tolerance,
                                     +1.0+g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5-g_tolerance,
                                     +1.5+g_tolerance,
                                     -1.0-g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(-0.5-g_tolerance,
                                     +1.5+g_tolerance,
                                     +1.0+g_tolerance)));

  EXPECT_FALSE(box.Contains(Vector3d(+0.5-g_tolerance,
                                     -1.5-g_tolerance,
                                     -1.0-g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5-g_tolerance,
                                     -1.5-g_tolerance,
                                     +1.0+g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5-g_tolerance,
                                     +1.5+g_tolerance,
                                     -1.0-g_tolerance)));
  EXPECT_FALSE(box.Contains(Vector3d(+0.5-g_tolerance,
                                     +1.5+g_tolerance,
                                     +1.0+g_tolerance)));
}

/////////////////////////////////////////////////
TEST(OrientedBoxTest, OperatorStreamOut)
{
  OrientedBoxd b(Vector3d(0.1, 1.2, 2.3),
                  Pose3d(3.4, 4.5, 5.6, 0.0, -0.1, 0.2));
  std::ostringstream stream;
  stream << b;
  EXPECT_EQ(stream.str(),
      "Size[0.1 1.2 2.3] Pose[3.4 4.5 5.6 0 -0.1 0.2] Material[]");
}

//////////////////////////////////////////////////
TEST(OrientedBoxTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::OrientedBoxd box(Vector3d(1.0, 0.1, 10.4));
  double expectedVolume = 1.0 * 0.1 * 10.4;
  EXPECT_DOUBLE_EQ(expectedVolume, box.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, box.DensityFromMass(mass));

  // Bad density
  math::OrientedBoxd box2;
  EXPECT_GT(0.0, box2.DensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(OrientedBoxTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double w = 0.1;
  double h = 34.12;
  math::OrientedBoxd box(Vector3d(l, w, h));
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
