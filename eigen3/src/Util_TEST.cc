/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/math/eigen3/Util.hh>

using namespace ignition;

/////////////////////////////////////////////////
/// \brief Test the oriented box converted from a set of vertices
TEST(EigenUtil, verticesToOrientedBox)
{
  std::vector<math::Vector3d> vertices;

  vertices.push_back(math::Vector3d(1, 0, 0.5));
  vertices.push_back(math::Vector3d(2, 0.1, 0.4));
  vertices.push_back(math::Vector3d(2, 1, 3));
  vertices.push_back(math::Vector3d(1.6, 0.3, 0.1));
  vertices.push_back(math::Vector3d(1.5, 0.5, 1));
  vertices.push_back(math::Vector3d(1.4, 1, 3));
  vertices.push_back(math::Vector3d(1, 0.4, 0.7));
  vertices.push_back(math::Vector3d(0.9, 1.3, 0));
  vertices.push_back(math::Vector3d(0.6, 4, 2));
  vertices.push_back(math::Vector3d(0, 3, 3));
  vertices.push_back(math::Vector3d(-1, -2, 4));
  vertices.push_back(math::Vector3d(-2, -2, 0.6));

  math::OrientedBoxd box = math::eigen3::verticesToOrientedBox(
    vertices);

  auto position = box.Pose().Pos();
  auto rotation = box.Pose().Rot();
  auto size = box.Size();

  double error = 0.1;

  EXPECT_NEAR(size.X(), 3.09, error);
  EXPECT_NEAR(size.Y(), 4.39, error);
  EXPECT_NEAR(size.Z(), 6.63, error);

  EXPECT_NEAR(position.X(), 0.38, error);
  EXPECT_NEAR(position.Y(), 0.47, error);
  EXPECT_NEAR(position.Z(), 1.96, error);

  EXPECT_NEAR(rotation.Roll(), -1.66, error);
  EXPECT_NEAR(rotation.Pitch(), 0.4, error);
  EXPECT_NEAR(rotation.Yaw(), 2.7, error);
}

/////////////////////////////////////////////////
TEST(EigenUtil, emptyVertices)
{
  std::vector<math::Vector3d> emptyVertices;

  math::OrientedBoxd box = math::eigen3::verticesToOrientedBox(
    emptyVertices);

  auto position = box.Pose().Pos();
  auto rotation = box.Pose().Rot();
  auto size = box.Size();

  EXPECT_DOUBLE_EQ(size.X(), 0);
  EXPECT_DOUBLE_EQ(size.Y(), 0);
  EXPECT_DOUBLE_EQ(size.Z(), 0);

  EXPECT_DOUBLE_EQ(position.X(), 0);
  EXPECT_DOUBLE_EQ(position.Y(), 0);
  EXPECT_DOUBLE_EQ(position.Z(), 0);

  EXPECT_DOUBLE_EQ(rotation.Roll(), 0);
  EXPECT_DOUBLE_EQ(rotation.Pitch(), 0);
  EXPECT_DOUBLE_EQ(rotation.Yaw(), 0);
}

/////////////////////////////////////////////////
TEST(EigenUtil, simpleBox)
{
  std::vector<math::Vector3d> vertices;

  vertices.push_back(math::Vector3d(-1, -1, -1));
  vertices.push_back(math::Vector3d(-1, 1, -1));
  vertices.push_back(math::Vector3d(1, -1, -1));
  vertices.push_back(math::Vector3d(1, 1, -1));
  vertices.push_back(math::Vector3d(-1, -1, 1));
  vertices.push_back(math::Vector3d(-1, 1, 1));
  vertices.push_back(math::Vector3d(1, -1, 1));
  vertices.push_back(math::Vector3d(1, 1, 1));

  math::OrientedBoxd box = math::eigen3::verticesToOrientedBox(
    vertices);

  auto position = box.Pose().Pos();
  auto rotation = box.Pose().Rot();
  auto size = box.Size();

  double error = 0.1;

  EXPECT_NEAR(size.X(), 2, error);
  EXPECT_NEAR(size.Y(), 2, error);
  EXPECT_NEAR(size.Z(), 2, error);

  EXPECT_NEAR(position.X(), 0, error);
  EXPECT_NEAR(position.Y(), 0, error);
  EXPECT_NEAR(position.Z(), 0, error);

  EXPECT_NEAR(rotation.Roll(), 0, error);
  EXPECT_NEAR(rotation.Pitch(), 0, error);
  EXPECT_NEAR(rotation.Yaw(), 0, error);
}

/////////////////////////////////////////////////
TEST(EigenUtil, covarianceTest)
{
  std::vector<math::Vector3d> vertices;

  vertices.push_back(math::Vector3d(1, 0, 0.5));
  vertices.push_back(math::Vector3d(2, 0.1, 0.4));
  vertices.push_back(math::Vector3d(2, 1, 3));
  vertices.push_back(math::Vector3d(1.6, 0.3, 0.1));
  vertices.push_back(math::Vector3d(1.5, 0.5, 1));
  vertices.push_back(math::Vector3d(1.4, 1, 3));
  vertices.push_back(math::Vector3d(1, 0.4, 0.7));

  Eigen::Matrix3d covariance = math::eigen3::covarianceMatrix(
    vertices);

  double error = 0.1;

  EXPECT_NEAR(covariance(0), 0.145714, error);
  EXPECT_NEAR(covariance(1), 0.04, error);
  EXPECT_NEAR(covariance(2), 0.115714, error);
  EXPECT_NEAR(covariance(3), 0.04, error);
  EXPECT_NEAR(covariance(4), 0.136327, error);
  EXPECT_NEAR(covariance(5), 0.392653, error);
  EXPECT_NEAR(covariance(6), 0.115714, error);
  EXPECT_NEAR(covariance(7), 0.392653, error);
  EXPECT_NEAR(covariance(8), 1.29959, error);
}

/////////////////////////////////////////////////
TEST(EigenUtil, covarianceEmptyTest)
{
  std::vector<math::Vector3d> vertices;

  Eigen::Matrix3d covariance = math::eigen3::covarianceMatrix(
    vertices);

  EXPECT_DOUBLE_EQ(covariance(0), 1);
  EXPECT_DOUBLE_EQ(covariance(1), 0);
  EXPECT_DOUBLE_EQ(covariance(2), 0);
  EXPECT_DOUBLE_EQ(covariance(3), 0);
  EXPECT_DOUBLE_EQ(covariance(4), 1);
  EXPECT_DOUBLE_EQ(covariance(5), 0);
  EXPECT_DOUBLE_EQ(covariance(6), 0);
  EXPECT_DOUBLE_EQ(covariance(7), 0);
  EXPECT_DOUBLE_EQ(covariance(8), 1);
}
