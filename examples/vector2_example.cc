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
//! [complete]
#include <iostream>
#include <gz/math.hh>

int main(int argc, char **argv)
{
  // Create a Vector2 called vec2 of doubles using the typedef Vector2d.
  // The initial x any y values are zero.\n\n";
  // The x and y component of vec2 can be set at anytime.
//! [constructor]
  gz::math::Vector2d vec2;
  vec2.Set(2.0, 4.0);
//! [constructor]

  // The Vector2 class is a template, so you can also create a Vector2 using
  // gz::math::Vector2<double>
  //! [constructor2]
  gz::math::Vector2<double> vec2a;

  vec2a.Set(1.0, 2.0);
  //! [constructor2]

  // It's also possible to set initial values. This time we are using
  // a Vector2 of floats
  //! [constructor3]
  gz::math::Vector2f vec2b(1.2f, 3.4f);
  //! [constructor3]

  // We can output the contents of each vector using std::cout
//! [stdout]
  std::cout << "Vec2: " << vec2 << "\n"
            << "Vec2a: " << vec2a << "\n"
            << "Vec2b: " << vec2b << "\n";
//! [stdout]

  // You can also get access to each component in the vector using the
  // X(), Y() accessors or the [] operator.
  //! [access]
  std::cout << "Vec2: x=" << vec2.X() << " y=" << vec2.Y() << "\n";
  std::cout << "Vec2a: x=" << vec2a[0] << " y=" << vec2a[1] << "\n";
  std::cout << "Vec2b: x=" << vec2b.X() << " y=" << vec2b[1] << "\n";
  //! [access]

  // The [] operator is clamped to the range [0, 1]
  std::cout << vec2[3] << std::endl;

  // The Vector2 class overloads many common operators
//! [operators]
  std::cout << vec2 * vec2a << "\n"
            << vec2 + vec2a << "\n"
            << vec2 - vec2a << "\n"
            << vec2 / vec2a << "\n";
//! [operators]

  // There are also many useful function such as finding the distance
  // between two vectors
//! [distance]
  std::cout << vec2.Distance(vec2a) << std::endl;
//! [distance]

  // There are more functions in Vector2. Take a look at the API:
  // https://gazebosim.org/libs/math
}
//! [complete]
