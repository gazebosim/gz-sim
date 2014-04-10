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

#include "ignition/math/Helpers.hh"
#include "ignition/math/Pose3d.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(PoseTest, Pose)
{
  {
    // test hypothesis that if
    // A is the transform from O to P specified in frame O
    // B is the transform from P to Q specified in frame P
    // then, B + A is the transform from O to Q specified in frame O
    math::Pose3d A(math::Vector3d(1, 0, 0), math::Quaterniond(0, 0, M_PI/4.0));
    math::Pose3d B(math::Vector3d(1, 0, 0), math::Quaterniond(0, 0, M_PI/2.0));
    EXPECT_TRUE(math::equal((B + A).pos().x(), 1.0 + 1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((B + A).pos().y(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((B + A).pos().z(),               0.0));
    EXPECT_TRUE(math::equal((B + A).rot().GetAsEuler().x(),  0.0));
    EXPECT_TRUE(math::equal((B + A).rot().GetAsEuler().y(),  0.0));
    EXPECT_TRUE(math::equal((B + A).rot().GetAsEuler().z(), 3.0*M_PI/4.0));
  }
  {
    // If:
    // A is the transform from O to P in frame O
    // B is the transform from O to Q in frame O
    // then -A is transform from P to O specified in frame P
    math::Pose3d A(math::Vector3d(1, 0, 0), math::Quaterniond(0, 0, M_PI/4.0));
    EXPECT_TRUE(math::equal((math::Pose3d() - A).pos().x(),      -1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((math::Pose3d() - A).pos().y(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((math::Pose3d() - A).pos().z(),               0.0));
    EXPECT_TRUE(math::equal((math::Pose3d() - A).rot().GetAsEuler().x(),  0.0));
    EXPECT_TRUE(math::equal((math::Pose3d() - A).rot().GetAsEuler().y(),  0.0));
    EXPECT_TRUE(
        math::equal((math::Pose3d() - A).rot().GetAsEuler().z(), -M_PI/4));

    // test negation operator
    EXPECT_TRUE(math::equal((-A).pos().x(),      -1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((-A).pos().y(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((-A).pos().z(),               0.0));
    EXPECT_TRUE(math::equal((-A).rot().GetAsEuler().x(),  0.0));
    EXPECT_TRUE(math::equal((-A).rot().GetAsEuler().y(),  0.0));
    EXPECT_TRUE(math::equal((-A).rot().GetAsEuler().z(), -M_PI/4.0));
  }
  {
    // If:
    // A is the transform from O to P in frame O
    // B is the transform from O to Q in frame O
    // B - A is the transform from P to Q in frame P
    math::Pose3d A(math::Vector3d(1, 0, 0), math::Quaterniond(0, 0, M_PI/4.0));
    math::Pose3d B(math::Vector3d(1, 1, 0), math::Quaterniond(0, 0, M_PI/2.0));
    EXPECT_TRUE(math::equal((B - A).pos().x(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((B - A).pos().y(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((B - A).pos().z(),               0.0));
    EXPECT_TRUE(math::equal((B - A).rot().GetAsEuler().x(),  0.0));
    EXPECT_TRUE(math::equal((B - A).rot().GetAsEuler().y(),  0.0));
    EXPECT_TRUE(math::equal((B - A).rot().GetAsEuler().z(), M_PI/4.0));
  }
  {
    math::Pose3d pose;
    EXPECT_TRUE(pose.pos() == math::Vector3d(0, 0, 0));
    EXPECT_TRUE(pose.rot() == math::Quaterniond(0, 0, 0));
  }

  math::Pose3d pose(math::Vector3d(1, 2, 3), math::Quaterniond(.1, .2, .3));
  EXPECT_TRUE(pose.pos() == math::Vector3d(1, 2, 3));
  EXPECT_TRUE(pose.rot() == math::Quaterniond(.1, .2, .3));

  math::Pose3d pose1(pose);
  EXPECT_TRUE(pose1 == pose);

  pose.Set(math::Vector3d(2, 3, 4), math::Quaterniond(.3, .4, .5));
  EXPECT_TRUE(pose.pos() == math::Vector3d(2, 3, 4));
  EXPECT_TRUE(pose.rot() == math::Quaterniond(.3, .4, .5));
  EXPECT_TRUE(pose.IsFinite());

  pose1 = pose.GetInverse();
  EXPECT_TRUE(pose1.pos() == math::Vector3d(-1.38368, -3.05541, -4.21306));
  EXPECT_TRUE(pose1.rot() ==
      math::Quaterniond(0.946281, -0.0933066, -0.226566, -0.210984));

  pose = math::Pose3d(1, 2, 3, .1, .2, .3) + math::Pose3d(4, 5, 6, .4, .5, .6);
  EXPECT_TRUE(pose ==
      math::Pose3d(5.74534, 7.01053, 8.62899, 0.675732, 0.535753, 1.01174));

  pose += pose;
  EXPECT_TRUE(pose ==
      math::Pose3d(11.314, 16.0487, 15.2559, 1.49463, 0.184295, 2.13932));

  pose -= math::Pose3d(pose);
  EXPECT_TRUE(pose ==
      math::Pose3d(0, 0, 0, 0, 0, 0));

  pose.pos().Set(5, 6, 7);
  pose.rot().SetFromEuler(math::Vector3d(.4, .6, 0));

  EXPECT_TRUE(pose.CoordPositionAdd(math::Vector3d(1, 2, 3)) ==
      math::Vector3d(7.82531, 6.67387, 9.35871));

  EXPECT_TRUE(pose.CoordPositionAdd(pose1) ==
      math::Vector3d(2.58141, 2.4262, 3.8013));
  EXPECT_TRUE(pose.CoordRotationAdd(math::Quaterniond(0.1, 0, 0.2)) ==
      math::Quaterniond(0.520975, 0.596586, 0.268194));
  EXPECT_TRUE(pose.CoordPoseSolve(pose1) ==
      math::Pose3d(-0.130957, -11.552, -10.2329,
                 -0.462955, -1.15624, -0.00158047));

  EXPECT_TRUE(pose.RotatePositionAboutOrigin(math::Quaterniond(0.1, 0, 0.2)) ==
      math::Pose3d(6.09235, 5.56147, 6.47714, 0.4, 0.6, 0));

  pose.Reset();
  EXPECT_TRUE(pose.pos() == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(pose.rot() == math::Quaterniond(0, 0, 0));
}

/////////////////////////////////////////////////
TEST(PoseTest, ConstPose)
{
  const math::Pose3d pose(0, 1, 2, 0, 0, 0);

  EXPECT_TRUE(pose.pos() == math::Vector3d(0, 1, 2));
  EXPECT_TRUE(pose.rot() == math::Quaterniond(0, 0, 0));
}
