#include <gtest/gtest.h>
#include "TransformTypes.hh"

using namespace gz;
using namespace sim;

TEST(TransformationTypes, transform)
{
  // Create a vehicle rotated 90 degrees in the yaw axis
  math::Pose3d pose(math::Vector3d(0,0,0), math::Quaterniond(0, 0, GZ_PI_2));

  // Vehicle should moving at 1m/s along x-axis (In global frame)
  math::Vector3d velocity(1, 0, 0);

  // Imagine a current moving against the vehicle
  math::Vector3d current(-3, 0, 0);

  // Test transforms
  // Global transform should keep things unaffected.
  auto res = transform(TransformType::GLOBAL, pose, velocity, current);
  EXPECT_EQ(res, current);

  res = transform(TransformType::ADD_VELOCITY_GLOBAL, pose, velocity, current);
  EXPECT_EQ(res, math::Vector3d(-4, 0, 0));

  // Tranforming to local frame without accounting for velocity. Considering
  // robot is facing Y axis.
  // Top down view:
  //   vvvvvvvvv    Current direction
  //       ^ (Global X, Robot -Y, Robot Direction of motion)
  //       |
  //       |                                       Robot coords
  //  ----------->  (Global Y, Robot +X heading)   --->+x
  //       |                                       |
  //       |                                       v +y
  //
  res = transform(TransformType::LOCAL, pose, velocity, current);
  EXPECT_LT((res - math::Vector3d(0,3,0)).Length(), 1e-5);

  // Tranforming with vehicle's velocity in account
  res = transform(TransformType::ADD_VELOCITY_LOCAL, pose, velocity, current);
  EXPECT_LT((res - math::Vector3d(0,4,0)).Length(), 1e-5);
}

TEST(TransformationTypes, getTransformType)
{
  EXPECT_EQ(
    getTransformType("ADD_VELOCITY_LOCAL"), TransformType::ADD_VELOCITY_LOCAL);
  EXPECT_EQ(
    getTransformType("ADD_VELOCITY_GLOBAL"),
    TransformType::ADD_VELOCITY_GLOBAL);
  EXPECT_EQ(getTransformType("LOCAL"), TransformType::LOCAL);
  EXPECT_EQ(getTransformType("GLOBAL"), TransformType::GLOBAL);
  EXPECT_EQ(getTransformType("nonsense"), std::nullopt);
}