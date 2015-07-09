/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "ignition/math/Helpers.hh"
#include "ignition/math/Frustum.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(FrustumTest, CopyConstructor)
{
  // Frustum pointing down the +x axis
  Frustum frustum(
      // Near distance
      1,
      // Far distance
      10,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/240.0,
      // Pose
      Pose3d(0, 0, 0, 0, 0, 0));

  Frustum frustum2(frustum);

  EXPECT_EQ(frustum.FOV(), frustum2.FOV());
  EXPECT_EQ(frustum.Near(), frustum2.Near());
  EXPECT_EQ(frustum.Far(), frustum2.Far());
  EXPECT_EQ(frustum.AspectRatio(), frustum2.AspectRatio());
  EXPECT_EQ(frustum.AspectRatio(), frustum2.AspectRatio());

  EXPECT_EQ(frustum.Plane(Frustum::FRUSTUM_PLANE_NEAR).Normal(),
            frustum2.Plane(Frustum::FRUSTUM_PLANE_NEAR).Normal());

  EXPECT_EQ(frustum.Plane(Frustum::FRUSTUM_PLANE_FAR).Normal(),
            frustum2.Plane(Frustum::FRUSTUM_PLANE_FAR).Normal());

  EXPECT_EQ(frustum.Plane(Frustum::FRUSTUM_PLANE_LEFT).Normal(),
            frustum2.Plane(Frustum::FRUSTUM_PLANE_LEFT).Normal());

  EXPECT_EQ(frustum.Plane(Frustum::FRUSTUM_PLANE_RIGHT).Normal(),
            frustum2.Plane(Frustum::FRUSTUM_PLANE_RIGHT).Normal());

  EXPECT_EQ(frustum.Plane(Frustum::FRUSTUM_PLANE_TOP).Normal(),
            frustum2.Plane(Frustum::FRUSTUM_PLANE_TOP).Normal());

  EXPECT_EQ(frustum.Plane(Frustum::FRUSTUM_PLANE_BOTTOM).Normal(),
            frustum2.Plane(Frustum::FRUSTUM_PLANE_BOTTOM).Normal());
}

/////////////////////////////////////////////////
TEST(FrustumTest, PyramidXAxisPos)
{
  // Frustum pointing down the +x axis
  Frustum frustum(
      // Near distance
      1,
      // Far distance
      10,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/240.0,
      // Pose
      Pose3d(0, 0, 0, 0, 0, 0));

  EXPECT_FALSE(frustum.Contains(Vector3d(0, 0, 0)));
  EXPECT_TRUE(frustum.Contains(Vector3d(1, 0, 0)));

  EXPECT_TRUE(frustum.Contains(Vector3d(2, 0, 0)));
  EXPECT_TRUE(frustum.Contains(Vector3d(10, 0, 0)));
  EXPECT_FALSE(frustum.Contains(Vector3d(10.1, 0, 0)));

  EXPECT_TRUE(frustum.Contains(Box(Vector3d(1, 0, 0), Vector3d(5, 5, 5))));
  EXPECT_FALSE(frustum.Contains(Box(Vector3d(-1, 0, 0), Vector3d(.1, .2, .3))));
}

/////////////////////////////////////////////////
TEST(FrustumTest, PyramidXAxisNeg)
{
  // Frustum pointing down the -x axis
  Frustum frustum(
      // Near distance
      1,
      // Far distance
      10,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/240.0,
      // Pose
      Pose3d(0, 0, 0, 0, 0, M_PI));

  EXPECT_FALSE(frustum.Contains(Vector3d(0, 0, 0)));
  EXPECT_FALSE(frustum.Contains(Vector3d(-0.5, 0, 0)));
  EXPECT_FALSE(frustum.Contains(Vector3d(-10.1, 0, 0)));

  EXPECT_TRUE(frustum.Contains(Vector3d(-1, 0, 0)));
  EXPECT_TRUE(frustum.Contains(Vector3d(-2, 0, 0)));
  EXPECT_TRUE(frustum.Contains(Vector3d(-10, 0, 0)));

  EXPECT_FALSE(frustum.Contains(Box(Vector3d(1, 0, 0), Vector3d(5, 5, 5))));
  EXPECT_TRUE(frustum.Contains(Box(Vector3d(-1, 0, 0), Vector3d(.1, .2, .3))));
}

/////////////////////////////////////////////////
TEST(FrustumTest, PyramidYAxis)
{
  // Frustum pointing down the +y axis
  Frustum frustum(
      // Near distance
      .1,
      // Far distance
      5,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/320.0,
      // Pose
      Pose3d(0, 0, 0, 0, 0, M_PI*0.5));

  EXPECT_FALSE(frustum.Contains(Vector3d(0, 0, 0)));
  EXPECT_FALSE(frustum.Contains(Vector3d(1, 0, 0)));
  EXPECT_FALSE(frustum.Contains(Vector3d(.05, 0, 0)));

  EXPECT_TRUE(frustum.Contains(Vector3d(0, .1, 0)));
  EXPECT_TRUE(frustum.Contains(Vector3d(0, 1, 0)));
  EXPECT_TRUE(frustum.Contains(Vector3d(0, 5, 0)));

  EXPECT_TRUE(frustum.Contains(Box(Vector3d(0, 1, 0), Vector3d(5, 5, 5))));
  EXPECT_FALSE(frustum.Contains(Box(Vector3d(0, -1, 0), Vector3d(.1, 0, .3))));
}

/////////////////////////////////////////////////
TEST(FrustumTest, PyramidZAxis)
{
  // Frustum pointing down the -z axis
  Frustum frustum(
      // Near distance
      1,
      // Far distance
      10,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/320.0,
      // Pose
      Pose3d(0, 0, 0, 0, M_PI*0.5, 0));

  EXPECT_FALSE(frustum.Contains(Vector3d(0, 0, 0)));
  EXPECT_FALSE(frustum.Contains(Vector3d(0, 0, -0.9)));
  EXPECT_FALSE(frustum.Contains(Vector3d(0, 0, -10.5)));
  EXPECT_FALSE(frustum.Contains(Vector3d(0, 0, 0.9)));
  EXPECT_FALSE(frustum.Contains(Vector3d(0, 0, 10.5)));

  EXPECT_TRUE(frustum.Contains(Vector3d(0, 0, -1.1)));
  EXPECT_TRUE(frustum.Contains(Vector3d(0.5, 0.5, -5.5)));
  EXPECT_TRUE(frustum.Contains(Vector3d(0, 0, -10)));

  EXPECT_FALSE(frustum.Contains(Box(Vector3d(0, 0, 0), Vector3d(5, 5, 5))));
  EXPECT_TRUE(frustum.Contains(Box(Vector3d(0, 0, -1), Vector3d(.1, 0, .3))));
}

/////////////////////////////////////////////////
TEST(FrustumTest, NearFar)
{
  Frustum frustum(
      // Near distance
      1,
      // Far distance
      10,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/320.0,
      // Pose
      Pose3d(0, 0, 0, 0, M_PI*0.5, 0));

  EXPECT_DOUBLE_EQ(frustum.Near(), 1.0);
  EXPECT_DOUBLE_EQ(frustum.Far(), 10.0);

  frustum.SetNear(-1.0);
  frustum.SetFar(-10.0);

  EXPECT_DOUBLE_EQ(frustum.Near(), -1.0);
  EXPECT_DOUBLE_EQ(frustum.Far(), -10.0);
}

/////////////////////////////////////////////////
TEST(FrustumTest, FOV)
{
  Frustum frustum(
      // Near distance
      1,
      // Far distance
      10,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/320.0,
      // Pose
      Pose3d(0, 0, 0, 0, M_PI*0.5, 0));

  EXPECT_EQ(frustum.FOV(), math::Angle(IGN_DTOR(45)));

  frustum.SetFOV(1.5707);

  EXPECT_EQ(frustum.FOV(), math::Angle(1.5707));
}

/////////////////////////////////////////////////
TEST(FrustumTest, AspectRatio)
{
  Frustum frustum(
      // Near distance
      1,
      // Far distance
      10,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/320.0,
      // Pose
      Pose3d(0, 0, 0, 0, M_PI*0.5, 0));

  EXPECT_DOUBLE_EQ(frustum.AspectRatio(), 320.0/320.0);

  frustum.SetAspectRatio(1.3434);

  EXPECT_DOUBLE_EQ(frustum.AspectRatio(), 1.3434);
}

/////////////////////////////////////////////////
TEST(FrustumTest, Pose)
{
  Frustum frustum(
      // Near distance
      1,
      // Far distance
      10,
      // Field of view
      Angle(IGN_DTOR(45)),
      // Aspect ratio
      320.0/320.0,
      // Pose
      Pose3d(0, 0, 0, 0, M_PI*0.5, 0));

  EXPECT_EQ(frustum.Pose(), Pose3d(0, 0, 0, 0, M_PI*0.5, 0));

  frustum.SetPose(Pose3d(1, 2, 3, M_PI, 0, 0));

  EXPECT_EQ(frustum.Pose(), Pose3d(1, 2, 3, M_PI, 0, 0));
}
