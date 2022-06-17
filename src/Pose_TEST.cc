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

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/utils/SuppressWarning.hh>

using namespace gz;

/////////////////////////////////////////////////
TEST(PoseTest, Construction)
{
  math::Pose3d pose(1, 0, 0, 0, 0, 0);

  // Copy constructor
  math::Pose3d pose2(pose);
  EXPECT_EQ(pose2, pose);

  // Copy operator
  math::Pose3d pose3;
  pose3 = pose;
  EXPECT_EQ(pose3, pose);

  // Move constructor
  math::Pose3d pose4(std::move(pose));
  EXPECT_EQ(pose4, pose2);
  pose = pose4;
  EXPECT_EQ(pose, pose2);

  // Move operator
  math::Pose3d pose5;
  pose5 = std::move(pose2);
  EXPECT_EQ(pose5, pose3);
  pose2 = pose5;
  EXPECT_EQ(pose2, pose3);

  // Inequality
  math::Pose3d pose6;
  EXPECT_NE(pose6, pose3);
}

/////////////////////////////////////////////////
TEST(PoseTest, Pose)
{
  {
    // test hypothesis that if
    // A is the transform from O to P specified in frame O
    // B is the transform from P to Q specified in frame P
    // then, A * B is the transform from O to Q specified in frame O
    math::Pose3d A(math::Vector3d(1, 0, 0),
                   math::Quaterniond(0, 0, GZ_PI/4.0));
    math::Pose3d B(math::Vector3d(1, 0, 0),
                   math::Quaterniond(0, 0, GZ_PI/2.0));
    EXPECT_TRUE(math::equal((A * B).Pos().X(), 1.0 + 1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((A * B).Pos().Y(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((A * B).Pos().Z(),               0.0));
    EXPECT_TRUE(math::equal((A * B).Rot().Euler().X(),  0.0));
    EXPECT_TRUE(math::equal((A * B).Rot().Euler().Y(),  0.0));
    EXPECT_TRUE(math::equal((A * B).Rot().Euler().Z(), 3.0*GZ_PI/4.0));

    GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
    // Coverage for + operator
    EXPECT_EQ(A * B, B + A);
    EXPECT_NE(A * B, A + B);

    // Coverage for += operator
    math::Pose3d C(B);
    C += A;
    EXPECT_EQ(C, A * B);
    GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  }
  {
    // If:
    // A is the transform from O to P in frame O
    // B is the transform from O to Q in frame O
    // then -A is transform from P to O specified in frame P
    math::Pose3d A(math::Vector3d(1, 0, 0),
        math::Quaterniond(0, 0, GZ_PI/4.0));
    EXPECT_TRUE(math::equal(
        (A.Inverse() * math::Pose3d()).Pos().X(),      -1.0/sqrt(2)));
    EXPECT_TRUE(math::equal(
        (A.Inverse() * math::Pose3d()).Pos().Y(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal(
        (A.Inverse() * math::Pose3d()).Pos().Z(),               0.0));
    EXPECT_TRUE(math::equal(
        (A.Inverse() * math::Pose3d()).Rot().Euler().X(),  0.0));
    EXPECT_TRUE(math::equal(
        (A.Inverse() * math::Pose3d()).Rot().Euler().Y(),  0.0));
    EXPECT_TRUE(math::equal(
        (A.Inverse() * math::Pose3d()).Rot().Euler().Z(), -GZ_PI/4));

    GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
    // Coverage for unitary - operator
    // test negation operator
    EXPECT_TRUE(math::equal((-A).Pos().X(),      -1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((-A).Pos().Y(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((-A).Pos().Z(),               0.0));
    EXPECT_TRUE(math::equal((-A).Rot().Euler().X(),  0.0));
    EXPECT_TRUE(math::equal((-A).Rot().Euler().Y(),  0.0));
    EXPECT_TRUE(math::equal((-A).Rot().Euler().Z(), -GZ_PI/4.0));
    GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  }
  {
    // If:
    // A is the transform from O to P in frame O
    // B is the transform from O to Q in frame O
    // B - A is the transform from P to Q in frame P
    math::Pose3d A(math::Vector3d(1, 0, 0),
        math::Quaterniond(0, 0, GZ_PI/4.0));
    math::Pose3d B(math::Vector3d(1, 1, 0),
        math::Quaterniond(0, 0, GZ_PI/2.0));
    EXPECT_TRUE(math::equal((A.Inverse() * B).Pos().X(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((A.Inverse() * B).Pos().Y(),       1.0/sqrt(2)));
    EXPECT_TRUE(math::equal((A.Inverse() * B).Pos().Z(),               0.0));
    EXPECT_TRUE(math::equal((A.Inverse() * B).Rot().Euler().X(),  0.0));
    EXPECT_TRUE(math::equal((A.Inverse() * B).Rot().Euler().Y(),  0.0));
    EXPECT_TRUE(math::equal((A.Inverse() * B).Rot().Euler().Z(), GZ_PI/4.0));

    GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
    // Coverage for - operator
    EXPECT_EQ(A.Inverse() * B, B - A);

    // Coverage for -= operator
    math::Pose3d C(B);
    C -= A;
    EXPECT_EQ(C, A.Inverse() * B);
    GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
  }
  {
    math::Pose3d pose;
    EXPECT_TRUE(pose.Pos() == math::Vector3d(0, 0, 0));
    EXPECT_TRUE(pose.Rot() == math::Quaterniond(0, 0, 0));
  }

  math::Pose3d pose(math::Vector3d(1, 2, 3), math::Quaterniond(.1, .2, .3));
  EXPECT_TRUE(pose.Pos() == math::Vector3d(1, 2, 3));
  EXPECT_TRUE(pose.Rot() == math::Quaterniond(.1, .2, .3));

  math::Pose3d pose1(pose);
  EXPECT_TRUE(pose1 == pose);

  pose.Set(math::Vector3d(2, 3, 4), math::Quaterniond(.3, .4, .5));
  EXPECT_TRUE(pose.Pos() == math::Vector3d(2, 3, 4));
  EXPECT_TRUE(pose.Rot() == math::Quaterniond(.3, .4, .5));
  EXPECT_TRUE(pose.IsFinite());

  pose1 = pose.Inverse();
  EXPECT_TRUE(pose1.Pos() == math::Vector3d(-1.38368, -3.05541, -4.21306));
  EXPECT_TRUE(pose1.Rot() ==
      math::Quaterniond(0.946281, -0.0933066, -0.226566, -0.210984));

  pose = math::Pose3d(4, 5, 6, .4, .5, .6) * math::Pose3d(1, 2, 3, .1, .2, .3);
  EXPECT_TRUE(pose ==
      math::Pose3d(5.74534, 7.01053, 8.62899, 0.675732, 0.535753, 1.01174));

  pose *= pose;
  EXPECT_TRUE(pose ==
      math::Pose3d(11.314, 16.0487, 15.2559, 1.49463, 0.184295, 2.13932));

  pose = pose.Inverse() * pose;
  EXPECT_TRUE(pose ==
      math::Pose3d(0, 0, 0, 0, 0, 0));

  pose.Pos().Set(5, 6, 7);
  pose.Rot().SetFromEuler(math::Vector3d(.4, .6, 0));

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
  EXPECT_TRUE(pose.Pos() == math::Vector3d(0, 0, 0));
  EXPECT_TRUE(pose.Rot() == math::Quaterniond(0, 0, 0));
}

/////////////////////////////////////////////////
TEST(PoseTest, ConstPose)
{
  const math::Pose3d pose(0, 1, 2, 1, 0, 0);

  EXPECT_TRUE(pose.Pos() == math::Vector3d(0, 1, 2));
  EXPECT_TRUE(pose.Rot() == math::Quaterniond(1, 0, 0));
}

/////////////////////////////////////////////////
TEST(PoseTest, OperatorStreamOut)
{
  math::Pose3d p(0.1, 1.2, 2.3, 0.0, 0.1, 1.0);
  std::ostringstream stream;
  stream << p;
  EXPECT_EQ(stream.str(), "0.1 1.2 2.3 0 0.1 1");
}

/////////////////////////////////////////////////
TEST(PoseTest, OperatorStreamOutZero)
{
  math::Pose3d p(0.1, 1.2, 2.3, 0, 0, 0);
  std::ostringstream stream;
  stream << p;
  EXPECT_EQ(stream.str(), "0.1 1.2 2.3 0 0 0");
}

/////////////////////////////////////////////////
TEST(PoseTest, MutablePose)
{
  math::Pose3d pose(0, 1, 2, 0, 0, 0);

  EXPECT_TRUE(pose.Pos() == math::Vector3d(0, 1, 2));
  EXPECT_TRUE(pose.Rot() == math::Quaterniond(0, 0, 0));

  pose.Pos() = math::Vector3d(10, 20, 30);
  pose.Rot() = math::Quaterniond(1, 2, 1);

  EXPECT_TRUE(pose.Pos() == math::Vector3d(10, 20, 30));
  EXPECT_TRUE(pose.Rot() == math::Quaterniond(1, 2, 1));
}

/////////////////////////////////////////////////
TEST(PoseTest, ConstPoseElements)
{
  const math::Pose3d pose(0, 1, 2, 1, 1, 2);
  EXPECT_DOUBLE_EQ(pose.X(), 0);
  EXPECT_DOUBLE_EQ(pose.Y(), 1);
  EXPECT_DOUBLE_EQ(pose.Z(), 2);
  EXPECT_DOUBLE_EQ(pose.Roll(), 1);
  EXPECT_DOUBLE_EQ(pose.Pitch(), 1);
  EXPECT_DOUBLE_EQ(pose.Yaw(), 2);
}

/////////////////////////////////////////////////
TEST(PoseTest, SetPoseElements)
{
  math::Pose3d pose(1, 2, 3, 1.57, 1, 2);
  EXPECT_DOUBLE_EQ(pose.X(), 1);
  EXPECT_DOUBLE_EQ(pose.Y(), 2);
  EXPECT_DOUBLE_EQ(pose.Z(), 3);

  pose.SetX(10);
  pose.SetY(12);
  pose.SetZ(13);

  EXPECT_DOUBLE_EQ(pose.X(), 10);
  EXPECT_DOUBLE_EQ(pose.Y(), 12);
  EXPECT_DOUBLE_EQ(pose.Z(), 13);
}
