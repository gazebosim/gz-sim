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
#include <gz/math/Angle.hh>

int main(int argc, char **argv)
{

//! [Create an angle]
  gz::math::Angle a;
//! [Create an angle]

  // A default constructed angle should be zero.
  std::cout << "The angle 'a' should be zero: " << a << std::endl;
//! [constant pi]
  a = gz::math::Angle::HalfPi;
  a = gz::math::Angle::Pi;
//! [constant pi]

//! [Output the angle in radians and degrees.]
  std::cout << "Pi in radians: " << a << std::endl;
  std::cout << "Pi in degrees: " << a.Degree() << std::endl;
//! [Output the angle in radians and degrees.]

//! [The Angle class overloads the +=, and many other, math operators.]
  a += gz::math::Angle::HalfPi;
//! [The Angle class overloads the +=, and many other, math operators.]
  std::cout << "Pi + PI/2 in radians: " << a << std::endl;
//! [normalized]
  std::cout << "Normalized to the range -Pi and Pi: "
    << a.Normalized() << std::endl;
//! [normalized]

}
//! [complete]
