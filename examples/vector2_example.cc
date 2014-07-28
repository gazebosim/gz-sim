/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <iostream>
#include <ignition/math.hh>

int main(int argc, char **argv)
{
  // Create a Vector2 called vec2 of doubles using the typedef Vector2d.
  // The initial x any y values are zero.\n\n";
  ignition::math::Vector2d vec2;

  // The x and y component of vec2 can be set at anytime.
  vec2.Set(2.0, 4.0);

  // The Vector2 class is a template, so you can also create a Vector2 using
  // ignition::math::Vector2<double>
  ignition::math::Vector2<double> vec2a;

  vec2a.Set(1.0, 2.0);

  // It's also possible to set initial values. This time we are using
  // a Vector2 of floats
  ignition::math::Vector2f vec2b(1.2, 3.4);

  // We can output the contents of each vector using std::cout
  std::cout << "Vec2: " << vec2 << "\n"
            << "Vec2a: " << vec2a << "\n"
            << "Vec2b: " << vec2b << "\n";

  // You can also get access to each component in the vector using the
  // X(), Y() accessors or the [] operator.
  std::cout << "Vec2: x=" << vec2.X() << " y=" << vec2.Y() << "\n";
  std::cout << "Vec2a: x=" << vec2a[0] << " y=" << vec2a[1] << "\n";
  std::cout << "Vec2b: x=" << vec2b.X() << " y=" << vec2b[1] << "\n";

  // An IndexException will be thrown if the [] operator is given a
  // value that is too high
  try
  {
    std::cout << vec2[3] << std::endl;
  } catch(ignition::math::IndexException &_e) {
    std::cerr << _e.what() << std::endl;
  }

  // The Vector2 class overloads many common operators
  std::cout << vec2 * vec2a << "\n"
            << vec2 + vec2a << "\n"
            << vec2 - vec2a << "\n"
            << vec2 / vec2a << "\n";

  // There are also many useful function such as finding the distance
  // between two vectors
  std::cout << vec2.Distance(vec2a) << std::endl;

  // There are more functions in Vector2. Take a look at the API;
  // http://ignitionrobotics.org/libraries/ign_mat/api
}
