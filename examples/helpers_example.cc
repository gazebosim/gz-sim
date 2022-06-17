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
#include <gz/math/Quaternion.hh>

int main(int argc, char **argv)
{
  std::cout << "The volume of a sphere with r=2 is "
    << GZ_SPHERE_VOLUME(2) << std::endl;

  std::cout << "The volume of a cylinder with r=4 and l=5 is "
    << GZ_CYLINDER_VOLUME(4, 5) << std::endl;

  std::cout << "The volume of a box with x=1, y=2, and z=3 is "
    << GZ_BOX_VOLUME(1, 2, 3) << std::endl;

  std::cout << "The result of clamping 2.4 to the range [1,2] is "
    << gz::math::clamp(2.4f, 1.0f, 2.0f) << std::endl;

  std::vector v{1, 2, 3, 4, 5};
  std::cout << "The mean of a vector containing {1, 2, 3, 4, 5} is "
    << gz::math::mean(v) << std::endl;

  std::cout << "The variance of a vector containing {1, 2, 3, 4, 5} is "
    << gz::math::variance(v) << std::endl;

  std::cout << "The result of rounding up 3 to the next power of two is "
    << gz::math::roundUpPowerOfTwo(3) << std::endl;

}
//! [complete]
