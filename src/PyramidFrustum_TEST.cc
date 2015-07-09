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
#include "ignition/math/PyramidFrustum.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(PyramidFrustumTest, PyramidConstructor)
{
  math::PyramidFrustum frustum(1, 10, math::Angle(IGN_DTOR(45)),
      math::Pose3d(0, 0, 0, 0, 0, 0));

  EXPECT_FALSE(frustum.Contains(math::Vector3d(0, 0, 0)));
  std::cout << "_-----------------\n";
  EXPECT_TRUE(frustum.Contains(math::Vector3d(1, 0, 0)));
}
