/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <gz/math/detail/InterpolationPoint.hh>

#include <gtest/gtest.h>

using namespace gz;
using namespace math;

TEST(Interpolation, LinearInterpolate)
{
  std::vector<double> vec{0, 2};

  InterpolationPoint1D<double> first {0.0, 0};
  InterpolationPoint1D<double> last {1.0, 1};

  EXPECT_DOUBLE_EQ(LinearInterpolate(first, last, vec, 0.0), 0);
  EXPECT_DOUBLE_EQ(LinearInterpolate(first, last, vec, 1.0), 2);
  EXPECT_DOUBLE_EQ(LinearInterpolate(first, last, vec, 0.5), 1);

  EXPECT_DOUBLE_EQ(LinearInterpolate(last, first, vec, 0.0), 0);
  EXPECT_DOUBLE_EQ(LinearInterpolate(last, first, vec, 1.0), 2);
  EXPECT_DOUBLE_EQ(LinearInterpolate(last, first, vec, 0.5), 1);
}

TEST(Interpolation, LinearInterpolate3D)
{
  std::vector<double> vec{0, 2};

  InterpolationPoint3D<double> first {Vector3d{0, 0, 0}, 0};
  InterpolationPoint3D<double> last {Vector3d{1, 0, 0}, 1};

  EXPECT_NEAR(
    LinearInterpolate(first, last, vec, Vector3d{0.0, 0, 0}), 0, 1e-3);
  EXPECT_NEAR(
    LinearInterpolate(first, last, vec, Vector3d{1.0, 0, 0}), 2, 1e-3);
  EXPECT_NEAR(
    LinearInterpolate(first, last, vec, Vector3d{0.5, 0, 0}), 1, 1e-3);
}

TEST(Interpolation, BiLinearInterpolate)
{
  std::vector<InterpolationPoint3D<double>> vec {
    InterpolationPoint3D<double>{Vector3d{0, 0, 0}, 0},
    InterpolationPoint3D<double>{Vector3d{1, 0, 0}, 1},
    InterpolationPoint3D<double>{Vector3d{0, 1, 0}, 2},
    InterpolationPoint3D<double>{Vector3d{1, 1, 0}, 3},
    InterpolationPoint3D<double>{Vector3d{0, 0, 1}, 4},
    InterpolationPoint3D<double>{Vector3d{1, 0, 1}, 5},
    InterpolationPoint3D<double>{Vector3d{0, 1, 1}, 6},
    InterpolationPoint3D<double>{Vector3d{1, 1, 1}, 7},
  };
  {
    std::vector<double> values {0, 0, 0, 0, 1, 1, 1, 1};
    auto v0 = BiLinearInterpolate(vec, 0, values, Vector3d{0.5, 0.5, 0});
    auto v1 = BiLinearInterpolate(vec, 4, values, Vector3d{0.5, 0.5, 1});
    EXPECT_NEAR(v0, 0, 1e-3);
    EXPECT_NEAR(v1, 1, 1e-3);
  }
  {
    std::vector<double> values {0, 0, 1, 1};
    auto v0 = BiLinearInterpolate(vec, 0, values, Vector3d{0, 0, 0});
    EXPECT_NEAR(v0, 0, 1e-3);
    auto v1 = BiLinearInterpolate(vec, 0, values, Vector3d{0, 1, 0});
    EXPECT_NEAR(v1, 1, 1e-3);
    auto v2 = BiLinearInterpolate(vec, 0, values, Vector3d{0, 0.5, 0});
    EXPECT_NEAR(v2, 0.5, 1e-3);
  }
}

TEST(Interpolation, TrilinearInterpolate)
{
  std::vector<InterpolationPoint3D<double>> vec {
    InterpolationPoint3D<double>{Vector3d{0, 0, 0}, 0},
    InterpolationPoint3D<double>{Vector3d{1, 0, 0}, 1},
    InterpolationPoint3D<double>{Vector3d{0, 1, 0}, 2},
    InterpolationPoint3D<double>{Vector3d{1, 1, 0}, 3},
    InterpolationPoint3D<double>{Vector3d{0, 0, 1}, 4},
    InterpolationPoint3D<double>{Vector3d{1, 0, 1}, 5},
    InterpolationPoint3D<double>{Vector3d{0, 1, 1}, 6},
    InterpolationPoint3D<double>{Vector3d{1, 1, 1}, 7},
  };

  std::vector<double> values {0, 0, 0, 0, 1, 1, 1, 1};

  auto v0 = TrilinearInterpolate(vec, values, Vector3d{0.5, 0.5, 0.5});
  EXPECT_NEAR(v0, 0.5, 1e-3);

  auto v1 = TrilinearInterpolate(vec, values, Vector3d{0.5, 0.5, 0});
  EXPECT_NEAR(v1, 0, 1e-3);

  auto v2 = TrilinearInterpolate(vec, values, Vector3d{0.5, 0.5, 1});
  EXPECT_NEAR(v2, 1, 1e-3);

  auto v3 = TrilinearInterpolate(vec, values, Vector3d{1, 1, 1});
  EXPECT_NEAR(v3, 1, 1e-3);
}
