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
#include <gz/math/Matrix3.hh>

int main(int argc, char **argv)
{
  // Construct a default matrix3.
  gz::math::Matrix3d m;
  std::cout << "The default constructed matrix m has the following values.\n\t"
    << m << std::endl;

  // Set the first column of the matrix.
  m.SetCol(0, gz::math::Vector3d(3, 4, 5));
  std::cout << "Setting the first column of the matrix m to 3, 4, 5.\n\t"
    << m << std::endl;

  // Transpose the matrix.
  gz::math::Matrix3d t = m.Transposed();
  std::cout << "The transposed matrix t has the values.\n\t"
    << t << std::endl;

  // Multiply the two matrices.
  std::cout << "m * t = " << m * t << std::endl;
}
//! [complete]
