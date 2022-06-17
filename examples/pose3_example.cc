/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
//! [complete]
#include <iostream>
#include <gz/math/Pose3.hh>

int main(int argc, char **argv)
{
  // Construct a default Pose3d.
  gz::math::Pose3d p;
  std::cout << "A default Pose3d has the following values\n"
    << p << std::endl;

  // Construct a pose at position 1, 2, 3 with a yaw of PI radians.
  gz::math::Pose3d p1(1, 2, 3, 0, 0, GZ_PI);
  std::cout << "A pose3d(1, 2, 3, 0, 0, GZ_PI) has the following values\n"
    << p1 << std::endl;

  // Set the position of a pose to 10, 20, 30
  p.Pos().Set(10, 20, 30);

  // Combine two poses, and store the result in a new pose
  gz::math::Pose3d p3 = p * p1;
  std::cout << "Result of adding two poses together is\n"
   << p3 << std::endl;
}
//! [complete]
